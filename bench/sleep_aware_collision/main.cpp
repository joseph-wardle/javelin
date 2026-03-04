import std;

import javelin.bench.cli;
import javelin.bench.json;
import javelin.bench.stats;
import javelin.bench.timer;
import javelin.core.time;
import javelin.core.types;
import javelin.math.quat;
import javelin.math.vec3;
import javelin.physics.aabb;
import javelin.physics.broad_phase;
import javelin.physics.bvh_dynamic;
import javelin.physics.bvh_static;
import javelin.physics.narrow_phase;
import javelin.physics.types;
import javelin.scene.shapes;

using namespace javelin;

namespace {

struct Config final {
    u32 bodies = 4096;
    f32 awake_ratio = 0.25f;
    f32 moved_awake_ratio = 0.10f;
    f32 spacing = 0.92f;
    f32 half_extent = 0.5f;
    f32 center_y = 0.48f;
    f32 moved_offset = 0.125f;
    u32 iterations = 120;
    u32 warmup = 20;
    u32 samples = 5;
    u32 seed = 1337;
    std::string json_out{};
};

struct Dataset final {
    std::vector<Vec3> position{};
    std::vector<Quat> orientation{};
    std::vector<ShapeKind> shape_kind{};
    std::vector<ShapeData> shapes{};
    std::vector<u32> shape_index{};
    std::vector<f32> inv_mass{};
    std::vector<u8> asleep{};
    std::vector<Aabb> bounds{};
    std::vector<u32> dynamic_ids{};
    std::vector<u32> awake_dynamic_ids{};
    std::vector<u32> moved_awake_ids{};
    std::vector<u8> all_query_mask{};
    std::vector<u8> awake_query_mask{};
    std::vector<ContactManifold> previous_manifolds{};
    StaticBvh static_bvh{};
};

struct SampleSummary final {
    std::vector<f64> us_per_iteration{};
    u64 checksum{};
    f64 avg_pairs_per_iteration{};
    f64 avg_manifolds_per_iteration{};
    f64 avg_points_per_iteration{};
};

enum struct Mode : u8 {
    baseline_all_dynamic = 0,
    sleep_aware = 1,
    incremental_moved_carry = 2,
};

void print_usage(const char *exe, const Config &cfg) {
    std::println("Sleep-aware collision pipeline benchmark");
    std::println("Usage: {} [--bodies=N] [--awake-ratio=F] [--spacing=F] "
                 "[--half-extent=F]",
                 exe);
    std::println("             [--center-y=F] [--iterations=N] [--warmup=N] "
                 "[--samples=N] [--seed=N]");
    std::println("             [--moved-awake-ratio=F] [--moved-offset=F]");
    std::println("             [--json-out=PATH]\n");
    std::println("Defaults:");
    std::println("  --bodies={}", cfg.bodies);
    std::println("  --awake-ratio={}", cfg.awake_ratio);
    std::println("  --moved-awake-ratio={}", cfg.moved_awake_ratio);
    std::println("  --moved-offset={}", cfg.moved_offset);
    std::println("  --spacing={}", cfg.spacing);
    std::println("  --half-extent={}", cfg.half_extent);
    std::println("  --center-y={}", cfg.center_y);
    std::println("  --iterations={}", cfg.iterations);
    std::println("  --warmup={}", cfg.warmup);
    std::println("  --samples={}", cfg.samples);
    std::println("  --seed={}", cfg.seed);
}

[[nodiscard]] bool parse_arg(std::string_view arg, Config &cfg) {
    bench::ParsedArg parsed{};
    if (!bench::split_key_value_arg(arg, parsed)) {
        return false;
    }

    if (parsed.key == "bodies") {
        return bench::parse_u32(parsed.value, cfg.bodies);
    }
    if (parsed.key == "awake-ratio") {
        return bench::parse_f32(parsed.value, cfg.awake_ratio);
    }
    if (parsed.key == "moved-awake-ratio") {
        return bench::parse_f32(parsed.value, cfg.moved_awake_ratio);
    }
    if (parsed.key == "moved-offset") {
        return bench::parse_f32(parsed.value, cfg.moved_offset);
    }
    if (parsed.key == "spacing") {
        return bench::parse_f32(parsed.value, cfg.spacing);
    }
    if (parsed.key == "half-extent") {
        return bench::parse_f32(parsed.value, cfg.half_extent);
    }
    if (parsed.key == "center-y") {
        return bench::parse_f32(parsed.value, cfg.center_y);
    }
    if (parsed.key == "iterations") {
        return bench::parse_u32(parsed.value, cfg.iterations);
    }
    if (parsed.key == "warmup") {
        return bench::parse_u32(parsed.value, cfg.warmup);
    }
    if (parsed.key == "samples") {
        return bench::parse_u32(parsed.value, cfg.samples);
    }
    if (parsed.key == "seed") {
        return bench::parse_u32(parsed.value, cfg.seed);
    }
    if (parsed.key == "json-out") {
        cfg.json_out = std::string{parsed.value};
        return !cfg.json_out.empty();
    }
    return false;
}

[[nodiscard]] bool body_pair_less(const BodyPair lhs, const BodyPair rhs) noexcept {
    if (lhs.a != rhs.a) {
        return lhs.a < rhs.a;
    }
    return lhs.b < rhs.b;
}

[[nodiscard]] bool body_pair_equal(const BodyPair lhs, const BodyPair rhs) noexcept {
    return lhs.a == rhs.a && lhs.b == rhs.b;
}

void normalize_pairs(std::vector<BodyPair> &pairs) {
    for (BodyPair &pair : pairs) {
        pair = canonical_body_pair(pair.a, pair.b);
    }
    std::sort(pairs.begin(), pairs.end(), body_pair_less);
    const auto unique_end = std::unique(pairs.begin(), pairs.end(), body_pair_equal);
    pairs.erase(unique_end, pairs.end());
}

[[nodiscard]] u32 contact_point_count(std::span<const ContactManifold> manifolds) noexcept {
    u32 count = 0u;
    for (const ContactManifold &manifold : manifolds) {
        count += manifold.point_count;
    }
    return count;
}

[[nodiscard]] Dataset build_dataset(const Config &cfg) {
    Dataset out{};
    out.position.resize(cfg.bodies);
    out.orientation.resize(cfg.bodies, Quat::identity());
    out.shape_kind.resize(cfg.bodies, ShapeKind::box);
    out.shape_index.resize(cfg.bodies, 0u);
    out.inv_mass.resize(cfg.bodies, 1.0f);
    out.asleep.resize(cfg.bodies, 1u);
    out.bounds.resize(cfg.bodies);
    out.dynamic_ids.resize(cfg.bodies);
    out.all_query_mask.resize(cfg.bodies, 1u);
    out.awake_query_mask.resize(cfg.bodies, 0u);
    out.shapes.push_back(ShapeData::make_box(BoxShape{.half_extents = Vec3{cfg.half_extent}}));

    const u32 cols = static_cast<u32>(std::ceil(std::sqrt(static_cast<f32>(cfg.bodies))));
    for (u32 i = 0; i < cfg.bodies; ++i) {
        const u32 x = i % cols;
        const u32 z = i / cols;
        const Vec3 center{static_cast<f32>(x) * cfg.spacing, cfg.center_y, static_cast<f32>(z) * cfg.spacing};
        out.position[i] = center;
        out.bounds[i] = Aabb{center - Vec3{cfg.half_extent}, center + Vec3{cfg.half_extent}};
        out.dynamic_ids[i] = i;
    }

    std::vector<u32> shuffled_ids = out.dynamic_ids;
    std::mt19937 rng{cfg.seed};
    std::shuffle(shuffled_ids.begin(), shuffled_ids.end(), rng);
    const u32 awake_target =
        std::clamp<u32>(static_cast<u32>(std::round(static_cast<f64>(cfg.bodies) * cfg.awake_ratio)), 0u, cfg.bodies);
    for (u32 i = 0; i < awake_target; ++i) {
        const u32 id = shuffled_ids[i];
        out.asleep[id] = 0u;
        out.awake_query_mask[id] = 1u;
    }

    out.awake_dynamic_ids.reserve(awake_target);
    for (const u32 id : out.dynamic_ids) {
        if (out.asleep[id] == 0u) {
            out.awake_dynamic_ids.push_back(id);
        }
    }

    const u32 moved_awake_target = std::clamp<u32>(
        static_cast<u32>(std::round(static_cast<f64>(out.awake_dynamic_ids.size()) * cfg.moved_awake_ratio)), 0u,
        static_cast<u32>(out.awake_dynamic_ids.size()));
    out.moved_awake_ids.assign(out.awake_dynamic_ids.begin(), out.awake_dynamic_ids.begin() + moved_awake_target);
    return out;
}

[[nodiscard]] DynamicBvh build_dynamic_bvh(const Dataset &dataset) {
    DynamicBvh bvh{};
    bvh.reserve(static_cast<u32>(dataset.dynamic_ids.size()));
    broad_phase_update_dynamic_bvh(dataset.dynamic_ids, bvh, dataset.bounds);
    return bvh;
}

[[nodiscard]] SampleSummary run_mode_samples(const Config &cfg, const Dataset &dataset, const Mode mode) {
    static constexpr f32 kIncrementalFallbackMovedRatio = 0.60f;

    SampleSummary summary{};
    summary.us_per_iteration.reserve(cfg.samples);

    std::vector<BodyPair> pairs{};
    pairs.reserve(static_cast<usize>(cfg.bodies) * 8u);
    std::vector<ContactManifold> previous_manifolds{};
    previous_manifolds.reserve(static_cast<usize>(cfg.bodies) * 4u);
    std::vector<ContactManifold> next_manifolds{};
    next_manifolds.reserve(static_cast<usize>(cfg.bodies) * 4u);
    BroadPhaseScratch scratch{};
    scratch.reserve(cfg.bodies, 2u);
    std::vector<u8> moved_query_mask{};
    moved_query_mask.resize(cfg.bodies, static_cast<u8>(0u));
    std::vector<u32> moved_query_ids{};
    moved_query_ids.reserve(dataset.awake_dynamic_ids.size());
    std::vector<Vec3> position{};
    position.reserve(cfg.bodies);
    std::vector<Aabb> bounds{};
    bounds.reserve(cfg.bodies);

    const auto run_iterations = [&](const u32 iteration_count, u64 &checksum_out, u64 &pair_sum, u64 &manifold_sum,
                                    u64 &point_sum) -> std::chrono::nanoseconds {
        DynamicBvh dynamic_bvh = build_dynamic_bvh(dataset);
        position = dataset.position;
        bounds = dataset.bounds;
        previous_manifolds = dataset.previous_manifolds;

        auto apply_moved_awake_jitter = [&](const u32 iter) {
            if (dataset.moved_awake_ids.empty()) {
                return;
            }
            const f32 offset = ((iter & 1u) == 0u) ? cfg.moved_offset : -cfg.moved_offset;
            for (const u32 id : dataset.moved_awake_ids) {
                Vec3 center = dataset.position[id];
                center.x += offset;
                position[id] = center;
                const Vec3 extents{cfg.half_extent};
                bounds[id] = Aabb{center - extents, center + extents};
            }
        };

        auto append_overlapping_previous_pairs = [&](std::span<const u8> awake_mask) {
            for (const ContactManifold &manifold : previous_manifolds) {
                if (manifold.b == kInvalidBody) {
                    continue;
                }
                const BodyPair pair = canonical_body_pair(manifold.a, manifold.b);
                if (awake_mask[pair.a] == 0u && awake_mask[pair.b] == 0u) {
                    continue;
                }
                if (!overlaps(bounds[pair.a], bounds[pair.b])) {
                    continue;
                }
                pairs.push_back(pair);
            }
        };

        using clock = SteadyClock;
        const auto start = clock::now();
        for (u32 iter = 0; iter < iteration_count; ++iter) {
            apply_moved_awake_jitter(iter);

            std::span<const u32> query_ids{};
            std::span<const u8> query_mask{};
            std::span<const u32> ground_query_ids{};
            bool use_incremental_carry = false;

            if (mode == Mode::baseline_all_dynamic) {
                broad_phase_update_dynamic_bvh(dataset.dynamic_ids, dynamic_bvh, bounds);
                query_ids = dataset.dynamic_ids;
                query_mask = dataset.all_query_mask;
                ground_query_ids = dataset.dynamic_ids;
            } else if (mode == Mode::sleep_aware) {
                broad_phase_update_dynamic_bvh(dataset.awake_dynamic_ids, dynamic_bvh, bounds);
                query_ids = dataset.awake_dynamic_ids;
                query_mask = dataset.awake_query_mask;
                ground_query_ids = dataset.awake_dynamic_ids;
            } else {
                moved_query_ids.clear();
                std::fill(moved_query_mask.begin(), moved_query_mask.end(), static_cast<u8>(0u));
                for (const u32 id : dataset.awake_dynamic_ids) {
                    if (!dynamic_bvh.update(id, bounds[id])) {
                        continue;
                    }
                    moved_query_mask[id] = 1u;
                    moved_query_ids.push_back(id);
                }

                const f32 moved_ratio = dataset.awake_dynamic_ids.empty()
                                            ? 0.0f
                                            : (static_cast<f32>(moved_query_ids.size()) /
                                               static_cast<f32>(dataset.awake_dynamic_ids.size()));
                const bool fallback_to_full_query = moved_ratio >= kIncrementalFallbackMovedRatio;
                query_ids = fallback_to_full_query ? std::span<const u32>{dataset.awake_dynamic_ids}
                                                   : std::span<const u32>{moved_query_ids};
                query_mask = fallback_to_full_query ? std::span<const u8>{dataset.awake_query_mask}
                                                    : std::span<const u8>{moved_query_mask};
                ground_query_ids = dataset.awake_dynamic_ids;
                use_incremental_carry = !fallback_to_full_query;
            }

            broad_phase_generate_pairs(query_ids, dynamic_bvh, dataset.static_bvh, bounds, query_mask, pairs, scratch);
            if (use_incremental_carry) {
                append_overlapping_previous_pairs(dataset.awake_query_mask);
            }
            normalize_pairs(pairs);
            narrow_phase_contacts(position, dataset.orientation, dataset.shape_kind, dataset.shapes,
                                  dataset.shape_index, dataset.inv_mass, pairs, previous_manifolds, ground_query_ids,
                                  next_manifolds);
            previous_manifolds.swap(next_manifolds);

            const u32 point_count = contact_point_count(previous_manifolds);
            pair_sum += pairs.size();
            manifold_sum += previous_manifolds.size();
            point_sum += point_count;
            checksum_out += static_cast<u64>(pairs.size()) * 3u + static_cast<u64>(previous_manifolds.size()) * 5u +
                            static_cast<u64>(point_count) * 7u;
        }
        return std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now() - start);
    };

    {
        u64 warmup_checksum = 0u;
        u64 warmup_pairs = 0u;
        u64 warmup_manifolds = 0u;
        u64 warmup_points = 0u;
        static_cast<void>(run_iterations(cfg.warmup, warmup_checksum, warmup_pairs, warmup_manifolds, warmup_points));
    }

    u64 pair_total = 0u;
    u64 manifold_total = 0u;
    u64 point_total = 0u;
    for (u32 sample = 0; sample < cfg.samples; ++sample) {
        u64 checksum = 0u;
        u64 pair_sum = 0u;
        u64 manifold_sum = 0u;
        u64 point_sum = 0u;
        const auto elapsed = run_iterations(cfg.iterations, checksum, pair_sum, manifold_sum, point_sum);
        summary.checksum += checksum;
        pair_total += pair_sum;
        manifold_total += manifold_sum;
        point_total += point_sum;
        summary.us_per_iteration.push_back(bench::us_per_iteration(elapsed, static_cast<u64>(cfg.iterations)));
    }

    const f64 total_iterations = static_cast<f64>(cfg.samples) * static_cast<f64>(cfg.iterations);
    summary.avg_pairs_per_iteration = static_cast<f64>(pair_total) / total_iterations;
    summary.avg_manifolds_per_iteration = static_cast<f64>(manifold_total) / total_iterations;
    summary.avg_points_per_iteration = static_cast<f64>(point_total) / total_iterations;
    return summary;
}

void print_summary(const std::string_view label, const SampleSummary &summary) {
    std::println("{}:", label);
    for (usize i = 0; i < summary.us_per_iteration.size(); ++i) {
        std::println("  sample {}: {} us/iter", i + 1u, summary.us_per_iteration[i]);
    }
    std::println("  median: {} us/iter", bench::median(summary.us_per_iteration));
    std::println("  p95: {} us/iter", bench::p95(summary.us_per_iteration));
    std::println("  avg pairs/iter: {}", summary.avg_pairs_per_iteration);
    std::println("  avg manifolds/iter: {}", summary.avg_manifolds_per_iteration);
    std::println("  avg points/iter: {}", summary.avg_points_per_iteration);
    std::println("  checksum: {}", summary.checksum);
    std::println("");
}

[[nodiscard]] bool write_json_summary(const std::string &path, const Config &cfg, const SampleSummary &baseline,
                                      const SampleSummary &sleep_aware, const SampleSummary &incremental,
                                      const u32 awake_count, const u32 sleeping_count, const f64 baseline_median_us,
                                      const f64 baseline_p95_us, const f64 sleep_aware_median_us,
                                      const f64 sleep_aware_p95_us, const f64 incremental_median_us,
                                      const f64 incremental_p95_us, const f64 speedup_sleep_aware,
                                      const f64 speedup_incremental, const f64 incremental_speedup_vs_sleep_aware,
                                      const f64 pair_reduction_sleep_aware, const f64 manifold_reduction_sleep_aware,
                                      const f64 point_reduction_sleep_aware, const f64 pair_reduction_incremental,
                                      const f64 manifold_reduction_incremental, const f64 point_reduction_incremental) {
    std::ofstream out{};
    if (!bench::open_json_output(path, out)) {
        return false;
    }

    out << "{\n";
    out << "  \"bench\": \"sleep_aware_collision\",\n";
    out << "  \"units\": \"us/iter\",\n";
    out << "  \"config\": {\n";
    out << "    \"bodies\": " << cfg.bodies << ",\n";
    out << "    \"awake_ratio\": " << cfg.awake_ratio << ",\n";
    out << "    \"moved_awake_ratio\": " << cfg.moved_awake_ratio << ",\n";
    out << "    \"awake_count\": " << awake_count << ",\n";
    out << "    \"sleeping_count\": " << sleeping_count << ",\n";
    out << "    \"spacing\": " << cfg.spacing << ",\n";
    out << "    \"half_extent\": " << cfg.half_extent << ",\n";
    out << "    \"center_y\": " << cfg.center_y << ",\n";
    out << "    \"moved_offset\": " << cfg.moved_offset << ",\n";
    out << "    \"iterations\": " << cfg.iterations << ",\n";
    out << "    \"warmup\": " << cfg.warmup << ",\n";
    out << "    \"samples\": " << cfg.samples << ",\n";
    out << "    \"seed\": " << cfg.seed << "\n";
    out << "  },\n";
    out << "  \"compare_metrics\": {\n";
    out << "    \"baseline_median_us_per_iter\": " << baseline_median_us << ",\n";
    out << "    \"baseline_p95_us_per_iter\": " << baseline_p95_us << ",\n";
    out << "    \"sleep_aware_median_us_per_iter\": " << sleep_aware_median_us << ",\n";
    out << "    \"sleep_aware_p95_us_per_iter\": " << sleep_aware_p95_us << ",\n";
    out << "    \"incremental_median_us_per_iter\": " << incremental_median_us << ",\n";
    out << "    \"incremental_p95_us_per_iter\": " << incremental_p95_us << ",\n";
    out << "    \"incremental_speedup_vs_sleep_aware_x\": " << incremental_speedup_vs_sleep_aware << "\n";
    out << "  },\n";
    out << "  \"summary\": {\n";
    out << "    \"sleep_aware_speedup_vs_baseline_x\": " << speedup_sleep_aware << ",\n";
    out << "    \"incremental_speedup_vs_baseline_x\": " << speedup_incremental << ",\n";
    out << "    \"incremental_speedup_vs_sleep_aware_x\": " << incremental_speedup_vs_sleep_aware << ",\n";
    out << "    \"sleep_aware_pairs_reduction_ratio\": " << pair_reduction_sleep_aware << ",\n";
    out << "    \"sleep_aware_manifolds_reduction_ratio\": " << manifold_reduction_sleep_aware << ",\n";
    out << "    \"sleep_aware_contact_points_reduction_ratio\": " << point_reduction_sleep_aware << ",\n";
    out << "    \"incremental_pairs_reduction_ratio\": " << pair_reduction_incremental << ",\n";
    out << "    \"incremental_manifolds_reduction_ratio\": " << manifold_reduction_incremental << ",\n";
    out << "    \"incremental_contact_points_reduction_ratio\": " << point_reduction_incremental << "\n";
    out << "  },\n";
    out << "  \"modes\": {\n";
    out << "    \"baseline\": {\n";
    out << "      \"samples_us_per_iter\": ";
    bench::write_json_f64_array(out, baseline.us_per_iteration);
    out << ",\n";
    out << "      \"median_us_per_iter\": " << baseline_median_us << ",\n";
    out << "      \"p95_us_per_iter\": " << baseline_p95_us << ",\n";
    out << "      \"avg_pairs_per_iteration\": " << baseline.avg_pairs_per_iteration << ",\n";
    out << "      \"avg_manifolds_per_iteration\": " << baseline.avg_manifolds_per_iteration << ",\n";
    out << "      \"avg_points_per_iteration\": " << baseline.avg_points_per_iteration << ",\n";
    out << "      \"checksum\": " << baseline.checksum << "\n";
    out << "    },\n";
    out << "    \"sleep_aware\": {\n";
    out << "      \"samples_us_per_iter\": ";
    bench::write_json_f64_array(out, sleep_aware.us_per_iteration);
    out << ",\n";
    out << "      \"median_us_per_iter\": " << sleep_aware_median_us << ",\n";
    out << "      \"p95_us_per_iter\": " << sleep_aware_p95_us << ",\n";
    out << "      \"avg_pairs_per_iteration\": " << sleep_aware.avg_pairs_per_iteration << ",\n";
    out << "      \"avg_manifolds_per_iteration\": " << sleep_aware.avg_manifolds_per_iteration << ",\n";
    out << "      \"avg_points_per_iteration\": " << sleep_aware.avg_points_per_iteration << ",\n";
    out << "      \"checksum\": " << sleep_aware.checksum << "\n";
    out << "    },\n";
    out << "    \"incremental_moved_carry\": {\n";
    out << "      \"samples_us_per_iter\": ";
    bench::write_json_f64_array(out, incremental.us_per_iteration);
    out << ",\n";
    out << "      \"median_us_per_iter\": " << incremental_median_us << ",\n";
    out << "      \"p95_us_per_iter\": " << incremental_p95_us << ",\n";
    out << "      \"avg_pairs_per_iteration\": " << incremental.avg_pairs_per_iteration << ",\n";
    out << "      \"avg_manifolds_per_iteration\": " << incremental.avg_manifolds_per_iteration << ",\n";
    out << "      \"avg_points_per_iteration\": " << incremental.avg_points_per_iteration << ",\n";
    out << "      \"checksum\": " << incremental.checksum << "\n";
    out << "    }\n";
    out << "  }\n";
    out << "}\n";
    return true;
}

} // namespace

int main(int argc, char **argv) {
    Config cfg{};
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg{argv[i]};
        if (bench::is_help_arg(arg)) {
            print_usage(argv[0], cfg);
            return 0;
        }
        if (!parse_arg(arg, cfg)) {
            std::cerr << "Unrecognized argument: " << arg << "\n";
            print_usage(argv[0], cfg);
            return 1;
        }
    }

    if (cfg.bodies == 0u || cfg.iterations == 0u || cfg.samples == 0u || cfg.spacing <= 0.0f ||
        cfg.half_extent <= 0.0f || cfg.moved_offset < 0.0f) {
        std::cerr << "bodies/iterations/samples must be > 0 and "
                     "spacing/half-extent must be positive. moved-offset must be >= 0.\n";
        return 1;
    }
    cfg.awake_ratio = std::clamp(cfg.awake_ratio, 0.0f, 1.0f);
    cfg.moved_awake_ratio = std::clamp(cfg.moved_awake_ratio, 0.0f, 1.0f);

    const Dataset dataset = build_dataset(cfg);
    const SampleSummary baseline = run_mode_samples(cfg, dataset, Mode::baseline_all_dynamic);
    const SampleSummary sleep_aware = run_mode_samples(cfg, dataset, Mode::sleep_aware);
    const SampleSummary incremental = run_mode_samples(cfg, dataset, Mode::incremental_moved_carry);

    std::println("Config:");
    std::println("  bodies: {}", cfg.bodies);
    std::println("  awake ratio: {}", cfg.awake_ratio);
    std::println("  moved awake ratio: {}", cfg.moved_awake_ratio);
    std::println("  awake count: {}", dataset.awake_dynamic_ids.size());
    std::println("  moved awake count: {}", dataset.moved_awake_ids.size());
    std::println("  sleeping count: {}", dataset.dynamic_ids.size() - dataset.awake_dynamic_ids.size());
    std::println("  spacing: {}", cfg.spacing);
    std::println("  half extent: {}", cfg.half_extent);
    std::println("  center y: {}", cfg.center_y);
    std::println("  moved offset: {}", cfg.moved_offset);
    std::println("  iterations: {}", cfg.iterations);
    std::println("  warmup: {}", cfg.warmup);
    std::println("  samples: {}", cfg.samples);
    std::println("  seed: {}", cfg.seed);
    std::println("");

    print_summary("baseline (query/update all dynamic, ground for all dynamic)", baseline);
    print_summary("sleep-aware (query/update awake dynamic, ground for awake dynamic)", sleep_aware);
    print_summary("incremental (query moved awake + carry overlapping previous manifolds)", incremental);

    const f64 baseline_median_us = bench::median(baseline.us_per_iteration);
    const f64 baseline_p95_us = bench::p95(baseline.us_per_iteration);
    const f64 sleep_aware_median_us = bench::median(sleep_aware.us_per_iteration);
    const f64 sleep_aware_p95_us = bench::p95(sleep_aware.us_per_iteration);
    const f64 incremental_median_us = bench::median(incremental.us_per_iteration);
    const f64 incremental_p95_us = bench::p95(incremental.us_per_iteration);
    const f64 speedup_sleep_aware = (sleep_aware_median_us > 0.0) ? (baseline_median_us / sleep_aware_median_us)
                                                                  : std::numeric_limits<f64>::quiet_NaN();
    const f64 speedup_incremental = (incremental_median_us > 0.0) ? (baseline_median_us / incremental_median_us)
                                                                  : std::numeric_limits<f64>::quiet_NaN();
    const f64 incremental_speedup_vs_sleep_aware = (incremental_median_us > 0.0)
                                                       ? (sleep_aware_median_us / incremental_median_us)
                                                       : std::numeric_limits<f64>::quiet_NaN();

    const f64 pair_reduction_sleep_aware =
        (baseline.avg_pairs_per_iteration > 0.0)
            ? (1.0 - sleep_aware.avg_pairs_per_iteration / baseline.avg_pairs_per_iteration)
            : 0.0;
    const f64 manifold_reduction_sleep_aware =
        (baseline.avg_manifolds_per_iteration > 0.0)
            ? (1.0 - sleep_aware.avg_manifolds_per_iteration / baseline.avg_manifolds_per_iteration)
            : 0.0;
    const f64 point_reduction_sleep_aware =
        (baseline.avg_points_per_iteration > 0.0)
            ? (1.0 - sleep_aware.avg_points_per_iteration / baseline.avg_points_per_iteration)
            : 0.0;
    const f64 pair_reduction_incremental =
        (baseline.avg_pairs_per_iteration > 0.0)
            ? (1.0 - incremental.avg_pairs_per_iteration / baseline.avg_pairs_per_iteration)
            : 0.0;
    const f64 manifold_reduction_incremental =
        (baseline.avg_manifolds_per_iteration > 0.0)
            ? (1.0 - incremental.avg_manifolds_per_iteration / baseline.avg_manifolds_per_iteration)
            : 0.0;
    const f64 point_reduction_incremental =
        (baseline.avg_points_per_iteration > 0.0)
            ? (1.0 - incremental.avg_points_per_iteration / baseline.avg_points_per_iteration)
            : 0.0;

    std::println("Workload delta (sleep-aware vs baseline):");
    std::println("  pairs reduction: {}%", pair_reduction_sleep_aware * 100.0);
    std::println("  manifolds reduction: {}%", manifold_reduction_sleep_aware * 100.0);
    std::println("  contact points reduction: {}%", point_reduction_sleep_aware * 100.0);
    std::println("  runtime speedup (median): {}x", speedup_sleep_aware);
    std::println("");
    std::println("Workload delta (incremental moved+carry vs baseline):");
    std::println("  pairs reduction: {}%", pair_reduction_incremental * 100.0);
    std::println("  manifolds reduction: {}%", manifold_reduction_incremental * 100.0);
    std::println("  contact points reduction: {}%", point_reduction_incremental * 100.0);
    std::println("  runtime speedup (median): {}x", speedup_incremental);
    std::println("  incremental vs sleep-aware speedup (median): {}x", incremental_speedup_vs_sleep_aware);

    if (!cfg.json_out.empty()) {
        if (!write_json_summary(cfg.json_out, cfg, baseline, sleep_aware, incremental,
                                static_cast<u32>(dataset.awake_dynamic_ids.size()),
                                static_cast<u32>(dataset.dynamic_ids.size() - dataset.awake_dynamic_ids.size()),
                                baseline_median_us, baseline_p95_us, sleep_aware_median_us, sleep_aware_p95_us,
                                incremental_median_us, incremental_p95_us, speedup_sleep_aware, speedup_incremental,
                                incremental_speedup_vs_sleep_aware, pair_reduction_sleep_aware,
                                manifold_reduction_sleep_aware, point_reduction_sleep_aware, pair_reduction_incremental,
                                manifold_reduction_incremental, point_reduction_incremental)) {
            return 1;
        }
    }
    return 0;
}

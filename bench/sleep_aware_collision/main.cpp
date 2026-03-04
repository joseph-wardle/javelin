import std;

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
    f32 spacing = 0.92f;
    f32 half_extent = 0.5f;
    f32 center_y = 0.48f;
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
    std::vector<u8> all_query_mask{};
    std::vector<u8> awake_query_mask{};
    std::vector<ContactManifold> previous_manifolds{};
    StaticBvh static_bvh{};
};

struct SampleSummary final {
    std::vector<double> us_per_iteration{};
    u64 checksum{};
    double avg_pairs_per_iteration{};
    double avg_manifolds_per_iteration{};
    double avg_points_per_iteration{};
};

enum struct Mode : u8 {
    baseline_all_dynamic = 0,
    sleep_aware = 1,
};

[[nodiscard]] bool parse_u32(std::string_view text, u32 &out) {
    if (text.empty()) {
        return false;
    }
    u32 value = 0;
    const char *begin = text.data();
    const char *end = begin + text.size();
    const auto result = std::from_chars(begin, end, value);
    if (result.ec != std::errc{} || result.ptr != end) {
        return false;
    }
    out = value;
    return true;
}

[[nodiscard]] bool parse_f32(std::string_view text, f32 &out) {
    if (text.empty()) {
        return false;
    }
    std::string tmp{text};
    char *end = nullptr;
    const f32 value = std::strtof(tmp.c_str(), &end);
    if (end == tmp.c_str() || *end != '\0') {
        return false;
    }
    out = value;
    return true;
}

void print_usage(const char *exe, const Config &cfg) {
    std::println("Sleep-aware collision pipeline benchmark");
    std::println("Usage: {} [--bodies=N] [--awake-ratio=F] [--spacing=F] "
                 "[--half-extent=F]",
                 exe);
    std::println("             [--center-y=F] [--iterations=N] [--warmup=N] "
                 "[--samples=N] [--seed=N]");
    std::println("             [--json-out=PATH]\n");
    std::println("Defaults:");
    std::println("  --bodies={}", cfg.bodies);
    std::println("  --awake-ratio={}", cfg.awake_ratio);
    std::println("  --spacing={}", cfg.spacing);
    std::println("  --half-extent={}", cfg.half_extent);
    std::println("  --center-y={}", cfg.center_y);
    std::println("  --iterations={}", cfg.iterations);
    std::println("  --warmup={}", cfg.warmup);
    std::println("  --samples={}", cfg.samples);
    std::println("  --seed={}", cfg.seed);
}

[[nodiscard]] bool parse_arg(std::string_view arg, Config &cfg) {
    if (!arg.starts_with("--")) {
        return false;
    }
    const auto eq = arg.find('=');
    if (eq == std::string_view::npos) {
        return false;
    }

    const std::string_view key = arg.substr(2, eq - 2);
    const std::string_view value = arg.substr(eq + 1);

    if (key == "bodies") {
        return parse_u32(value, cfg.bodies);
    }
    if (key == "awake-ratio") {
        return parse_f32(value, cfg.awake_ratio);
    }
    if (key == "spacing") {
        return parse_f32(value, cfg.spacing);
    }
    if (key == "half-extent") {
        return parse_f32(value, cfg.half_extent);
    }
    if (key == "center-y") {
        return parse_f32(value, cfg.center_y);
    }
    if (key == "iterations") {
        return parse_u32(value, cfg.iterations);
    }
    if (key == "warmup") {
        return parse_u32(value, cfg.warmup);
    }
    if (key == "samples") {
        return parse_u32(value, cfg.samples);
    }
    if (key == "seed") {
        return parse_u32(value, cfg.seed);
    }
    if (key == "json-out") {
        cfg.json_out = std::string{value};
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
    return out;
}

[[nodiscard]] DynamicBvh build_dynamic_bvh(const Dataset &dataset) {
    DynamicBvh bvh{};
    bvh.reserve(static_cast<u32>(dataset.dynamic_ids.size()));
    broad_phase_update_dynamic_bvh(dataset.dynamic_ids, bvh, dataset.bounds);
    return bvh;
}

[[nodiscard]] SampleSummary run_mode_samples(const Config &cfg, const Dataset &dataset, const Mode mode) {
    SampleSummary summary{};
    summary.us_per_iteration.reserve(cfg.samples);

    std::vector<BodyPair> pairs{};
    pairs.reserve(static_cast<usize>(cfg.bodies) * 8u);
    std::vector<ContactManifold> manifolds{};
    manifolds.reserve(static_cast<usize>(cfg.bodies) * 4u);
    BroadPhaseScratch scratch{};
    scratch.reserve(cfg.bodies, 2u);

    const auto run_iterations = [&](const u32 iteration_count, u64 &checksum_out, u64 &pair_sum, u64 &manifold_sum,
                                    u64 &point_sum) -> std::chrono::nanoseconds {
        DynamicBvh dynamic_bvh = build_dynamic_bvh(dataset);
        const std::span<const u32> update_ids = (mode == Mode::baseline_all_dynamic)
                                                    ? std::span<const u32>{dataset.dynamic_ids}
                                                    : std::span<const u32>{dataset.awake_dynamic_ids};
        const std::span<const u32> query_ids = (mode == Mode::baseline_all_dynamic)
                                                   ? std::span<const u32>{dataset.dynamic_ids}
                                                   : std::span<const u32>{dataset.awake_dynamic_ids};
        const std::span<const u8> query_mask = (mode == Mode::baseline_all_dynamic)
                                                   ? std::span<const u8>{dataset.all_query_mask}
                                                   : std::span<const u8>{dataset.awake_query_mask};

        using clock = SteadyClock;
        const auto start = clock::now();
        for (u32 iter = 0; iter < iteration_count; ++iter) {
            broad_phase_update_dynamic_bvh(update_ids, dynamic_bvh, dataset.bounds);
            broad_phase_generate_pairs(query_ids, dynamic_bvh, dataset.static_bvh, dataset.bounds, query_mask, pairs,
                                       scratch);
            normalize_pairs(pairs);
            narrow_phase_contacts(dataset.position, dataset.orientation, dataset.shape_kind, dataset.shapes,
                                  dataset.shape_index, dataset.inv_mass, pairs, dataset.previous_manifolds, query_ids,
                                  manifolds);

            const u32 point_count = contact_point_count(manifolds);
            pair_sum += pairs.size();
            manifold_sum += manifolds.size();
            point_sum += point_count;
            checksum_out += static_cast<u64>(pairs.size()) * 3u + static_cast<u64>(manifolds.size()) * 5u +
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
        summary.us_per_iteration.push_back(static_cast<double>(elapsed.count()) /
                                           (static_cast<double>(cfg.iterations) * 1000.0));
    }

    const double total_iterations = static_cast<double>(cfg.samples) * static_cast<double>(cfg.iterations);
    summary.avg_pairs_per_iteration = static_cast<double>(pair_total) / total_iterations;
    summary.avg_manifolds_per_iteration = static_cast<double>(manifold_total) / total_iterations;
    summary.avg_points_per_iteration = static_cast<double>(point_total) / total_iterations;
    return summary;
}

[[nodiscard]] double median(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    return values[values.size() / 2u];
}

[[nodiscard]] double p95(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    const usize p95_index = static_cast<usize>(std::ceil(values.size() * 0.95)) - 1u;
    return values[p95_index];
}

void print_summary(const std::string_view label, const SampleSummary &summary) {
    std::println("{}:", label);
    for (usize i = 0; i < summary.us_per_iteration.size(); ++i) {
        std::println("  sample {}: {} us/iter", i + 1u, summary.us_per_iteration[i]);
    }
    std::println("  median: {} us/iter", median(summary.us_per_iteration));
    std::println("  p95: {} us/iter", p95(summary.us_per_iteration));
    std::println("  avg pairs/iter: {}", summary.avg_pairs_per_iteration);
    std::println("  avg manifolds/iter: {}", summary.avg_manifolds_per_iteration);
    std::println("  avg points/iter: {}", summary.avg_points_per_iteration);
    std::println("  checksum: {}", summary.checksum);
    std::println("");
}

[[nodiscard]] bool write_json_summary(const std::string &path, const Config &cfg, const SampleSummary &baseline,
                                      const SampleSummary &sleep_aware, const u32 awake_count, const u32 sleeping_count,
                                      const double baseline_median_us, const double baseline_p95_us,
                                      const double sleep_aware_median_us, const double sleep_aware_p95_us,
                                      const double speedup, const double pair_reduction,
                                      const double manifold_reduction, const double point_reduction) {
    namespace fs = std::filesystem;
    const fs::path out_path{path};
    std::error_code ec{};
    if (out_path.has_parent_path()) {
        fs::create_directories(out_path.parent_path(), ec);
        if (ec) {
            std::cerr << "Failed to create directory '" << out_path.parent_path().string()
                      << "' for JSON output: " << ec.message() << "\n";
            return false;
        }
    }

    std::ofstream out{out_path};
    if (!out.is_open()) {
        std::cerr << "Failed to open JSON output file: " << out_path.string() << "\n";
        return false;
    }
    out << std::setprecision(17);
    out << "{\n";
    out << "  \"bench\": \"sleep_aware_collision\",\n";
    out << "  \"units\": \"us/iter\",\n";
    out << "  \"config\": {\n";
    out << "    \"bodies\": " << cfg.bodies << ",\n";
    out << "    \"awake_ratio\": " << cfg.awake_ratio << ",\n";
    out << "    \"awake_count\": " << awake_count << ",\n";
    out << "    \"sleeping_count\": " << sleeping_count << ",\n";
    out << "    \"spacing\": " << cfg.spacing << ",\n";
    out << "    \"half_extent\": " << cfg.half_extent << ",\n";
    out << "    \"center_y\": " << cfg.center_y << ",\n";
    out << "    \"iterations\": " << cfg.iterations << ",\n";
    out << "    \"warmup\": " << cfg.warmup << ",\n";
    out << "    \"samples\": " << cfg.samples << ",\n";
    out << "    \"seed\": " << cfg.seed << "\n";
    out << "  },\n";
    out << "  \"compare_metrics\": {\n";
    out << "    \"baseline_median_us_per_iter\": " << baseline_median_us << ",\n";
    out << "    \"baseline_p95_us_per_iter\": " << baseline_p95_us << ",\n";
    out << "    \"sleep_aware_median_us_per_iter\": " << sleep_aware_median_us << ",\n";
    out << "    \"sleep_aware_p95_us_per_iter\": " << sleep_aware_p95_us << "\n";
    out << "  },\n";
    out << "  \"summary\": {\n";
    out << "    \"runtime_speedup_median_x\": " << speedup << ",\n";
    out << "    \"pairs_reduction_ratio\": " << pair_reduction << ",\n";
    out << "    \"manifolds_reduction_ratio\": " << manifold_reduction << ",\n";
    out << "    \"contact_points_reduction_ratio\": " << point_reduction << "\n";
    out << "  },\n";
    out << "  \"modes\": {\n";
    out << "    \"baseline\": {\n";
    out << "      \"samples_us_per_iter\": [";
    for (usize i = 0u; i < baseline.us_per_iteration.size(); ++i) {
        if (i > 0u) {
            out << ", ";
        }
        out << baseline.us_per_iteration[i];
    }
    out << "],\n";
    out << "      \"median_us_per_iter\": " << baseline_median_us << ",\n";
    out << "      \"p95_us_per_iter\": " << baseline_p95_us << ",\n";
    out << "      \"avg_pairs_per_iteration\": " << baseline.avg_pairs_per_iteration << ",\n";
    out << "      \"avg_manifolds_per_iteration\": " << baseline.avg_manifolds_per_iteration << ",\n";
    out << "      \"avg_points_per_iteration\": " << baseline.avg_points_per_iteration << ",\n";
    out << "      \"checksum\": " << baseline.checksum << "\n";
    out << "    },\n";
    out << "    \"sleep_aware\": {\n";
    out << "      \"samples_us_per_iter\": [";
    for (usize i = 0u; i < sleep_aware.us_per_iteration.size(); ++i) {
        if (i > 0u) {
            out << ", ";
        }
        out << sleep_aware.us_per_iteration[i];
    }
    out << "],\n";
    out << "      \"median_us_per_iter\": " << sleep_aware_median_us << ",\n";
    out << "      \"p95_us_per_iter\": " << sleep_aware_p95_us << ",\n";
    out << "      \"avg_pairs_per_iteration\": " << sleep_aware.avg_pairs_per_iteration << ",\n";
    out << "      \"avg_manifolds_per_iteration\": " << sleep_aware.avg_manifolds_per_iteration << ",\n";
    out << "      \"avg_points_per_iteration\": " << sleep_aware.avg_points_per_iteration << ",\n";
    out << "      \"checksum\": " << sleep_aware.checksum << "\n";
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
        if (arg == "--help" || arg == "-h") {
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
        cfg.half_extent <= 0.0f) {
        std::cerr << "bodies/iterations/samples must be > 0 and "
                     "spacing/half-extent must be positive.\n";
        return 1;
    }
    cfg.awake_ratio = std::clamp(cfg.awake_ratio, 0.0f, 1.0f);

    const Dataset dataset = build_dataset(cfg);
    const SampleSummary baseline = run_mode_samples(cfg, dataset, Mode::baseline_all_dynamic);
    const SampleSummary sleep_aware = run_mode_samples(cfg, dataset, Mode::sleep_aware);

    std::println("Config:");
    std::println("  bodies: {}", cfg.bodies);
    std::println("  awake ratio: {}", cfg.awake_ratio);
    std::println("  awake count: {}", dataset.awake_dynamic_ids.size());
    std::println("  sleeping count: {}", dataset.dynamic_ids.size() - dataset.awake_dynamic_ids.size());
    std::println("  spacing: {}", cfg.spacing);
    std::println("  half extent: {}", cfg.half_extent);
    std::println("  center y: {}", cfg.center_y);
    std::println("  iterations: {}", cfg.iterations);
    std::println("  warmup: {}", cfg.warmup);
    std::println("  samples: {}", cfg.samples);
    std::println("  seed: {}", cfg.seed);
    std::println("");

    print_summary("baseline (query/update all dynamic, ground for all dynamic)", baseline);
    print_summary("sleep-aware (query/update awake dynamic, ground for awake dynamic)", sleep_aware);

    const double baseline_median_us = median(baseline.us_per_iteration);
    const double baseline_p95_us = p95(baseline.us_per_iteration);
    const double sleep_aware_median_us = median(sleep_aware.us_per_iteration);
    const double sleep_aware_p95_us = p95(sleep_aware.us_per_iteration);
    const double speedup = (sleep_aware_median_us > 0.0) ? (baseline_median_us / sleep_aware_median_us)
                                                         : std::numeric_limits<double>::quiet_NaN();

    const double pair_reduction = (baseline.avg_pairs_per_iteration > 0.0)
                                      ? (1.0 - sleep_aware.avg_pairs_per_iteration / baseline.avg_pairs_per_iteration)
                                      : 0.0;
    const double manifold_reduction =
        (baseline.avg_manifolds_per_iteration > 0.0)
            ? (1.0 - sleep_aware.avg_manifolds_per_iteration / baseline.avg_manifolds_per_iteration)
            : 0.0;
    const double point_reduction =
        (baseline.avg_points_per_iteration > 0.0)
            ? (1.0 - sleep_aware.avg_points_per_iteration / baseline.avg_points_per_iteration)
            : 0.0;

    std::println("Workload delta (sleep-aware vs baseline):");
    std::println("  pairs reduction: {}%", pair_reduction * 100.0);
    std::println("  manifolds reduction: {}%", manifold_reduction * 100.0);
    std::println("  contact points reduction: {}%", point_reduction * 100.0);
    std::println("  runtime speedup (median): {}x", speedup);

    if (!cfg.json_out.empty()) {
        if (!write_json_summary(cfg.json_out, cfg, baseline, sleep_aware,
                                static_cast<u32>(dataset.awake_dynamic_ids.size()),
                                static_cast<u32>(dataset.dynamic_ids.size() - dataset.awake_dynamic_ids.size()),
                                baseline_median_us, baseline_p95_us, sleep_aware_median_us, sleep_aware_p95_us, speedup,
                                pair_reduction, manifold_reduction, point_reduction)) {
            return 1;
        }
    }
    return 0;
}

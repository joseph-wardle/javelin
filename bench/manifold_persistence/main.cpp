import std;

import javelin.bench.cli;
import javelin.bench.stats;
import javelin.bench.timer;
import javelin.core.time;
import javelin.core.types;
import javelin.math;
import javelin.physics.types;

using namespace javelin;

namespace {

struct Config final {
    u32 bodies = 2048;
    u32 previous_manifolds = 9000;
    u32 next_manifolds = 9500;
    f32 match_ratio = 0.75f;
    f32 anchor_jitter = 0.01f;
    u32 iterations = 400;
    u32 warmup = 30;
    u32 samples = 7;
    u32 seed = 1337;
};

struct PersistenceRefreshStats final {
    u32 previous_point_count{};
    u32 next_point_count{};
    u32 matched_point_count{};
    u32 dropped_point_count{};
};

struct Dataset final {
    std::vector<Vec3> position;
    std::vector<Quat> orientation;
    std::vector<ContactManifold> previous_manifolds;
    std::vector<ContactManifold> next_seed;
    std::unordered_map<u64, u32> previous_lookup;
};

struct SampleSummary final {
    std::vector<f64> us_per_iteration;
    std::vector<f64> ns_per_next_manifold;
    u64 checksum{};
};

inline constexpr f32 kPersistenceAnchorThreshold = 0.03f;
inline constexpr f32 kPersistenceAnchorThresholdSq = kPersistenceAnchorThreshold * kPersistenceAnchorThreshold;
inline constexpr f32 kPersistenceNormalBreakThreshold = 0.015f;
inline constexpr f32 kPersistenceTangentialDriftBreakThreshold = 0.025f;
inline constexpr f32 kPersistenceTangentialDriftBreakThresholdSq =
    kPersistenceTangentialDriftBreakThreshold * kPersistenceTangentialDriftBreakThreshold;
inline constexpr f32 kPersistenceMatchEps = 1e-6f;

void print_usage(const char *exe, const Config &cfg) {
    std::println("Manifold persistence microbench");
    std::println("Usage: {} [--bodies=N] [--previous-manifolds=N] [--next-manifolds=N]", exe);
    std::println("             [--match-ratio=F] [--anchor-jitter=F] "
                 "[--iterations=N] [--warmup=N]");
    std::println("             [--samples=N] [--seed=N]\n");
    std::println("Defaults:");
    std::println("  --bodies={}", cfg.bodies);
    std::println("  --previous-manifolds={}", cfg.previous_manifolds);
    std::println("  --next-manifolds={}", cfg.next_manifolds);
    std::println("  --match-ratio={}", cfg.match_ratio);
    std::println("  --anchor-jitter={}", cfg.anchor_jitter);
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
    if (parsed.key == "previous-manifolds") {
        return bench::parse_u32(parsed.value, cfg.previous_manifolds);
    }
    if (parsed.key == "next-manifolds") {
        return bench::parse_u32(parsed.value, cfg.next_manifolds);
    }
    if (parsed.key == "match-ratio") {
        return bench::parse_f32(parsed.value, cfg.match_ratio);
    }
    if (parsed.key == "anchor-jitter") {
        return bench::parse_f32(parsed.value, cfg.anchor_jitter);
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

    return false;
}

[[nodiscard]] Vec3 random_vec3(std::mt19937 &rng, const f32 min_v, const f32 max_v) {
    std::uniform_real_distribution<f32> dist{min_v, max_v};
    return Vec3{dist(rng), dist(rng), dist(rng)};
}

[[nodiscard]] Vec3 random_unit_vec3(std::mt19937 &rng) {
    for (;;) {
        Vec3 axis = random_vec3(rng, -1.0f, 1.0f);
        if (axis.try_normalize()) {
            return axis;
        }
    }
}

[[nodiscard]] Quat random_orientation(std::mt19937 &rng) {
    constexpr f32 kPi = 3.14159265358979323846f;
    std::uniform_real_distribution<f32> angle_dist{-kPi, kPi};
    return from_axis_angle(random_unit_vec3(rng), angle_dist(rng));
}

[[nodiscard]] std::vector<BodyPair> build_pair_pool(const u32 body_count, const u32 pair_count) {
    std::vector<BodyPair> out;
    out.reserve(pair_count);
    for (u32 a = 0; a < body_count && out.size() < pair_count; ++a) {
        for (u32 b = a + 1u; b < body_count && out.size() < pair_count; ++b) {
            out.push_back(BodyPair{.a = a, .b = b});
        }
    }
    return out;
}

[[nodiscard]] bool manifold_pair_less(const ContactManifold &lhs, const ContactManifold &rhs) noexcept {
    if (lhs.a != rhs.a) {
        return lhs.a < rhs.a;
    }
    return lhs.b < rhs.b;
}

[[nodiscard]] ContactPoint random_point(std::mt19937 &rng, const bool include_impulse) {
    std::uniform_real_distribution<f32> anchor_dist{-0.2f, 0.2f};
    std::uniform_real_distribution<f32> sep_dist{-0.03f, 0.005f};
    std::uniform_real_distribution<f32> impulse_dist{0.0f, 2.0f};
    std::uniform_real_distribution<f32> friction_dist{-0.6f, 0.6f};
    std::uniform_int_distribution<u32> feature_dist{0u, 255u};
    std::uniform_real_distribution<f32> feature_valid_dist{0.0f, 1.0f};

    ContactPoint point{};
    point.local_anchor_a = Vec3{anchor_dist(rng), anchor_dist(rng), anchor_dist(rng)};
    point.local_anchor_b = Vec3{anchor_dist(rng), anchor_dist(rng), anchor_dist(rng)};
    point.separation = sep_dist(rng);
    point.feature_id = (feature_valid_dist(rng) < 0.85f) ? feature_dist(rng) : kInvalidContactFeature;
    point.normal_impulse = include_impulse ? impulse_dist(rng) : 0.0f;
    point.tangent_impulse = include_impulse ? Vec3{friction_dist(rng), friction_dist(rng), friction_dist(rng)} : Vec3{};
    point.persisted = include_impulse;
    return point;
}

[[nodiscard]] ContactManifold random_manifold(std::mt19937 &rng, const BodyPair pair) {
    std::uniform_int_distribution<u32> point_count_dist{1u, kMaxManifoldPoints};
    std::uniform_int_distribution<u32> feature_dist{0u, 2047u};

    ContactManifold manifold{};
    manifold.a = pair.a;
    manifold.b = pair.b;
    manifold.normal = random_unit_vec3(rng);
    manifold.point_count = point_count_dist(rng);
    manifold.manifold_feature_id = feature_dist(rng);
    for (u32 i = 0; i < manifold.point_count; ++i) {
        manifold.points[i] = random_point(rng, true);
    }
    return manifold;
}

[[nodiscard]] ContactManifold matched_manifold(std::mt19937 &rng, const ContactManifold &previous,
                                               const f32 anchor_jitter) {
    std::uniform_real_distribution<f32> jitter_dist{-anchor_jitter, anchor_jitter};
    std::uniform_real_distribution<f32> feature_keep_dist{0.0f, 1.0f};
    std::uniform_int_distribution<u32> feature_dist{0u, 255u};

    ContactManifold next = previous;
    for (u32 i = 0; i < next.point_count; ++i) {
        ContactPoint &point = next.points[i];
        point.local_anchor_a += Vec3{jitter_dist(rng), jitter_dist(rng), jitter_dist(rng)};
        point.local_anchor_b += Vec3{jitter_dist(rng), jitter_dist(rng), jitter_dist(rng)};
        point.separation += jitter_dist(rng) * 0.25f;
        if (feature_keep_dist(rng) > 0.9f) {
            point.feature_id = feature_dist(rng);
        }
        point.normal_impulse = 0.0f;
        point.tangent_impulse = Vec3{};
        point.persisted = false;
    }
    return next;
}

[[nodiscard]] f32 local_anchor_match_distance_sq(const ContactPoint &lhs, const ContactPoint &rhs,
                                                 const bool manifold_has_body_b) noexcept {
    const f32 anchor_delta_a_sq = (lhs.local_anchor_a - rhs.local_anchor_a).length_sq();
    if (!manifold_has_body_b) {
        return anchor_delta_a_sq;
    }
    const f32 anchor_delta_b_sq = (lhs.local_anchor_b - rhs.local_anchor_b).length_sq();
    return std::max(anchor_delta_a_sq, anchor_delta_b_sq);
}

void reset_point_cache(ContactPoint &point) noexcept {
    point.normal_impulse = 0.0f;
    point.tangent_impulse = Vec3{};
    point.persisted = false;
}

void reset_manifold_point_cache(ContactManifold &manifold) noexcept {
    for (u32 i = 0; i < manifold.point_count; ++i) {
        reset_point_cache(manifold.points[i]);
    }
}

void copy_point_cache(ContactPoint &dst, const ContactPoint &src) noexcept {
    dst.normal_impulse = src.normal_impulse;
    dst.tangent_impulse = src.tangent_impulse;
    dst.persisted = true;
}

[[nodiscard]] bool should_drop_persisted_point(std::span<const Vec3> position, std::span<const Quat> orientation,
                                               const ContactManifold &manifold, const ContactPoint &next_point,
                                               const ContactPoint &previous_point) noexcept {
    const u32 a = manifold.a;
    const Vec3 world_a_previous = position[a] + rotate(orientation[a], previous_point.local_anchor_a);
    const Vec3 world_a_next = position[a] + rotate(orientation[a], next_point.local_anchor_a);

    f32 normal_separation = 0.0f;
    f32 normal_drift = 0.0f;
    Vec3 tangential_delta{};
    if (manifold.b != kInvalidBody) {
        const u32 b = manifold.b;
        const Vec3 world_b_previous = position[b] + rotate(orientation[b], previous_point.local_anchor_b);
        const Vec3 world_b_next = position[b] + rotate(orientation[b], next_point.local_anchor_b);
        const Vec3 delta_previous = world_b_previous - world_a_previous;
        const Vec3 delta_next = world_b_next - world_a_next;

        const f32 normal_previous = dot(delta_previous, manifold.normal);
        const f32 normal_next = dot(delta_next, manifold.normal);
        normal_separation = normal_next;
        normal_drift = std::fabs(normal_next - normal_previous);

        const Vec3 tangential_previous = delta_previous - manifold.normal * normal_previous;
        const Vec3 tangential_next = delta_next - manifold.normal * normal_next;
        tangential_delta = tangential_next - tangential_previous;
    } else {
        const Vec3 delta = world_a_previous - world_a_next;
        const f32 normal_component = dot(delta, manifold.normal);
        normal_separation = std::fabs(normal_component);
        normal_drift = std::fabs(normal_component);
        tangential_delta = delta - manifold.normal * normal_component;
    }

    const bool normal_break_exceeded = normal_separation > kPersistenceNormalBreakThreshold;
    const bool normal_drift_exceeded = normal_drift > kPersistenceNormalBreakThreshold;
    const bool tangential_drift_exceeded = tangential_delta.length_sq() > kPersistenceTangentialDriftBreakThresholdSq;
    return normal_break_exceeded || normal_drift_exceeded || tangential_drift_exceeded;
}

void match_and_transfer_point_cache(std::span<const Vec3> position, std::span<const Quat> orientation,
                                    ContactManifold &next_manifold, const ContactManifold &previous_manifold) noexcept {
    const u32 next_point_count = next_manifold.point_count;
    const u32 previous_point_count = previous_manifold.point_count;
    const bool manifold_has_body_b = next_manifold.b != kInvalidBody;
    u8 previous_used_mask = 0u;
    const u8 all_previous_used_mask = static_cast<u8>((1u << previous_point_count) - 1u);

    for (u32 i = 0; i < next_point_count; ++i) {
        reset_point_cache(next_manifold.points[i]);
    }

    auto try_match_point = [&](const u32 next_index, const bool feature_pass) {
        ContactPoint &next_point = next_manifold.points[next_index];
        const u32 next_feature_id = next_point.feature_id;
        if (feature_pass && next_feature_id == kInvalidContactFeature) {
            return false;
        }

        u32 best_previous = kMaxManifoldPoints;
        f32 best_metric = std::numeric_limits<f32>::infinity();
        for (u32 previous_index = 0; previous_index < previous_point_count; ++previous_index) {
            const u8 previous_bit = static_cast<u8>(1u << previous_index);
            if ((previous_used_mask & previous_bit) != 0u) {
                continue;
            }

            const ContactPoint &previous_point = previous_manifold.points[previous_index];
            if (feature_pass && previous_point.feature_id != next_feature_id) {
                continue;
            }

            const f32 metric = local_anchor_match_distance_sq(next_point, previous_point, manifold_has_body_b);
            if (metric > kPersistenceAnchorThresholdSq) {
                continue;
            }

            const bool better =
                metric < best_metric - kPersistenceMatchEps ||
                (std::fabs(metric - best_metric) <= kPersistenceMatchEps && previous_index < best_previous);
            if (better) {
                best_metric = metric;
                best_previous = previous_index;
            }
        }

        if (best_previous == kMaxManifoldPoints) {
            return false;
        }
        const ContactPoint &previous_point = previous_manifold.points[best_previous];
        if (should_drop_persisted_point(position, orientation, next_manifold, next_point, previous_point)) {
            return false;
        }

        copy_point_cache(next_point, previous_point);
        previous_used_mask |= static_cast<u8>(1u << best_previous);
        return true;
    };

    for (u32 i = 0; i < next_point_count; ++i) {
        if (previous_used_mask == all_previous_used_mask) {
            return;
        }
        static_cast<void>(try_match_point(i, true));
    }
    for (u32 i = 0; i < next_point_count; ++i) {
        if (previous_used_mask == all_previous_used_mask) {
            return;
        }
        if (next_manifold.points[i].persisted) {
            continue;
        }
        static_cast<void>(try_match_point(i, false));
    }
}

[[nodiscard]] PersistenceRefreshStats refresh_hash_lookup(std::span<const Vec3> position,
                                                          std::span<const Quat> orientation,
                                                          std::span<const ContactManifold> previous_manifolds,
                                                          const std::unordered_map<u64, u32> &previous_lookup,
                                                          std::span<ContactManifold> next_manifolds) noexcept {
    PersistenceRefreshStats stats{};
    for (const ContactManifold &previous : previous_manifolds) {
        stats.previous_point_count += previous.point_count;
    }

    for (ContactManifold &next_manifold : next_manifolds) {
        stats.next_point_count += next_manifold.point_count;
        const auto it = previous_lookup.find(body_pair_key(next_manifold.a, next_manifold.b));
        if (it == previous_lookup.end()) {
            reset_manifold_point_cache(next_manifold);
            continue;
        }

        const ContactManifold &previous_manifold = previous_manifolds[it->second];
        match_and_transfer_point_cache(position, orientation, next_manifold, previous_manifold);
        for (u32 i = 0; i < next_manifold.point_count; ++i) {
            stats.matched_point_count += next_manifold.points[i].persisted ? 1u : 0u;
        }
    }
    stats.dropped_point_count = (stats.previous_point_count > stats.matched_point_count)
                                    ? (stats.previous_point_count - stats.matched_point_count)
                                    : 0u;
    return stats;
}

[[nodiscard]] PersistenceRefreshStats refresh_linear_merge(std::span<const Vec3> position,
                                                           std::span<const Quat> orientation,
                                                           std::span<const ContactManifold> previous_manifolds,
                                                           std::span<ContactManifold> next_manifolds) noexcept {
    PersistenceRefreshStats stats{};
    for (const ContactManifold &previous : previous_manifolds) {
        stats.previous_point_count += previous.point_count;
    }

    const u32 previous_count = static_cast<u32>(previous_manifolds.size());
    const u32 next_count = static_cast<u32>(next_manifolds.size());
    auto reset_unmatched_next_range = [&](const u32 begin, const u32 end) {
        for (u32 i = begin; i < end; ++i) {
            ContactManifold &next_manifold = next_manifolds[i];
            stats.next_point_count += next_manifold.point_count;
            reset_manifold_point_cache(next_manifold);
        }
    };

    if (next_count == 0u) {
        return stats;
    }
    if (previous_count == 0u) {
        reset_unmatched_next_range(0u, next_count);
        return stats;
    }

    const ContactManifold &previous_first = previous_manifolds.front();
    const ContactManifold &previous_last = previous_manifolds.back();
    const ContactManifold &next_first = next_manifolds.front();
    const ContactManifold &next_last = next_manifolds.back();
    const bool disjoint_pair_ranges =
        manifold_pair_less(previous_last, next_first) || manifold_pair_less(next_last, previous_first);
    if (disjoint_pair_ranges) {
        reset_unmatched_next_range(0u, next_count);
        return stats;
    }

    u32 next_index = static_cast<u32>(
        std::lower_bound(next_manifolds.begin(), next_manifolds.end(), previous_first, manifold_pair_less) -
        next_manifolds.begin());
    reset_unmatched_next_range(0u, next_index);
    if (next_index >= next_count) {
        return stats;
    }

    u32 previous_index = static_cast<u32>(std::lower_bound(previous_manifolds.begin(), previous_manifolds.end(),
                                                           next_manifolds[next_index], manifold_pair_less) -
                                          previous_manifolds.begin());
    while (next_index < next_count && previous_index < previous_count) {
        ContactManifold &next_manifold = next_manifolds[next_index];
        const ContactManifold &previous_manifold = previous_manifolds[previous_index];

        if (manifold_pair_less(previous_manifold, next_manifold)) {
            ++previous_index;
            continue;
        }
        stats.next_point_count += next_manifold.point_count;
        if (manifold_pair_less(next_manifold, previous_manifold)) {
            reset_manifold_point_cache(next_manifold);
            ++next_index;
            continue;
        }

        match_and_transfer_point_cache(position, orientation, next_manifold, previous_manifold);
        for (u32 i = 0; i < next_manifold.point_count; ++i) {
            stats.matched_point_count += next_manifold.points[i].persisted ? 1u : 0u;
        }
        ++previous_index;
        ++next_index;
    }

    reset_unmatched_next_range(next_index, next_count);

    stats.dropped_point_count = (stats.previous_point_count > stats.matched_point_count)
                                    ? (stats.previous_point_count - stats.matched_point_count)
                                    : 0u;
    return stats;
}

[[nodiscard]] Dataset build_dataset(const Config &cfg) {
    Dataset dataset{};
    dataset.position.reserve(cfg.bodies);
    dataset.orientation.reserve(cfg.bodies);

    std::mt19937 rng{cfg.seed};
    for (u32 i = 0; i < cfg.bodies; ++i) {
        dataset.position.push_back(random_vec3(rng, -30.0f, 30.0f));
        dataset.orientation.push_back(random_orientation(rng));
    }

    const u32 required_pairs =
        cfg.previous_manifolds + (cfg.next_manifolds - std::min(cfg.previous_manifolds, cfg.next_manifolds));
    const std::vector<BodyPair> pair_pool = build_pair_pool(cfg.bodies, required_pairs);
    if (pair_pool.size() < required_pairs) {
        throw std::runtime_error("Not enough unique body pairs. Increase --bodies "
                                 "or reduce manifold counts.");
    }

    dataset.previous_manifolds.reserve(cfg.previous_manifolds);
    for (u32 i = 0; i < cfg.previous_manifolds; ++i) {
        dataset.previous_manifolds.push_back(random_manifold(rng, pair_pool[i]));
    }

    const u32 max_overlap = std::min(cfg.previous_manifolds, cfg.next_manifolds);
    const u32 overlap = static_cast<u32>(std::round(static_cast<f64>(max_overlap) * cfg.match_ratio));
    const u32 clamped_overlap = std::min(overlap, max_overlap);

    std::vector<u32> overlap_indices;
    overlap_indices.reserve(cfg.previous_manifolds);
    for (u32 i = 0; i < cfg.previous_manifolds; ++i) {
        overlap_indices.push_back(i);
    }
    std::shuffle(overlap_indices.begin(), overlap_indices.end(), rng);
    overlap_indices.resize(clamped_overlap);
    std::sort(overlap_indices.begin(), overlap_indices.end());

    dataset.next_seed.reserve(cfg.next_manifolds);
    for (const u32 index : overlap_indices) {
        dataset.next_seed.push_back(matched_manifold(rng, dataset.previous_manifolds[index], cfg.anchor_jitter));
    }

    for (u32 i = clamped_overlap; i < cfg.next_manifolds; ++i) {
        const u32 pool_index = cfg.previous_manifolds + (i - clamped_overlap);
        ContactManifold manifold = random_manifold(rng, pair_pool[pool_index]);
        reset_manifold_point_cache(manifold);
        dataset.next_seed.push_back(manifold);
    }

    std::sort(dataset.previous_manifolds.begin(), dataset.previous_manifolds.end(), manifold_pair_less);
    std::sort(dataset.next_seed.begin(), dataset.next_seed.end(), manifold_pair_less);

    dataset.previous_lookup.reserve(dataset.previous_manifolds.size() * 2u);
    for (u32 i = 0; i < dataset.previous_manifolds.size(); ++i) {
        const ContactManifold &manifold = dataset.previous_manifolds[i];
        dataset.previous_lookup.emplace(body_pair_key(manifold.a, manifold.b), i);
    }

    return dataset;
}

template <typename RefreshFn>
[[nodiscard]] SampleSummary run_samples(const Config &cfg, const Dataset &dataset, RefreshFn &&refresh_fn) {
    SampleSummary summary{};
    summary.us_per_iteration.reserve(cfg.samples);
    summary.ns_per_next_manifold.reserve(cfg.samples);

    std::vector<ContactManifold> next_work = dataset.next_seed;
    auto run_iterations = [&](const u32 iterations, u64 &checksum_out) {
        using clock = SteadyClock;
        const auto start = clock::now();
        for (u32 iter = 0; iter < iterations; ++iter) {
            const PersistenceRefreshStats stats = refresh_fn(
                std::span<const Vec3>{dataset.position}, std::span<const Quat>{dataset.orientation},
                std::span<const ContactManifold>{dataset.previous_manifolds}, std::span<ContactManifold>{next_work});

            checksum_out += static_cast<u64>(stats.matched_point_count) * 3u +
                            static_cast<u64>(stats.next_point_count) * 5u +
                            static_cast<u64>(stats.dropped_point_count) * 7u;
        }
        return std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now() - start);
    };

    u64 warmup_checksum = 0;
    run_iterations(cfg.warmup, warmup_checksum);

    for (u32 sample = 0; sample < cfg.samples; ++sample) {
        u64 sample_checksum = 0;
        const auto elapsed = run_iterations(cfg.iterations, sample_checksum);
        summary.checksum += sample_checksum;

        const f64 elapsed_ns = bench::duration_ns(elapsed);
        const f64 us_per_iteration = bench::us_per_iteration(elapsed, static_cast<u64>(cfg.iterations));
        const f64 ns_per_next_manifold =
            elapsed_ns / (static_cast<f64>(cfg.iterations) * static_cast<f64>(dataset.next_seed.size()));

        summary.us_per_iteration.push_back(us_per_iteration);
        summary.ns_per_next_manifold.push_back(ns_per_next_manifold);
    }

    return summary;
}

void print_samples(std::string_view name, const SampleSummary &summary) {
    std::println("{}:", name);
    for (usize i = 0; i < summary.us_per_iteration.size(); ++i) {
        std::println("  sample {}: {} us/iter, {} ns/next-manifold", i + 1u, summary.us_per_iteration[i],
                     summary.ns_per_next_manifold[i]);
    }
    std::println("  median: {} us/iter, {} ns/next-manifold", bench::median(summary.us_per_iteration),
                 bench::median(summary.ns_per_next_manifold));
    std::println("  p95: {} us/iter, {} ns/next-manifold", bench::p95(summary.us_per_iteration),
                 bench::p95(summary.ns_per_next_manifold));
    std::println("  checksum: {}", summary.checksum);
    std::println("");
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

    if (cfg.bodies < 2u || cfg.previous_manifolds == 0u || cfg.next_manifolds == 0u || cfg.iterations == 0u ||
        cfg.samples == 0u) {
        std::cerr << "bodies>=2, manifold counts>0, iterations>0, and samples>0 "
                     "are required.\n";
        return 1;
    }
    cfg.match_ratio = std::clamp(cfg.match_ratio, 0.0f, 1.0f);
    cfg.anchor_jitter = std::max(cfg.anchor_jitter, 0.0f);

    const Dataset dataset = build_dataset(cfg);

    std::println("Config:");
    std::println("  bodies: {}", cfg.bodies);
    std::println("  previous manifolds: {}", cfg.previous_manifolds);
    std::println("  next manifolds: {}", cfg.next_manifolds);
    std::println("  match ratio: {}", cfg.match_ratio);
    std::println("  anchor jitter: {}", cfg.anchor_jitter);
    std::println("  iterations: {}", cfg.iterations);
    std::println("  warmup: {}", cfg.warmup);
    std::println("  samples: {}", cfg.samples);
    std::println("  seed: {}", cfg.seed);
    std::println("");

    const SampleSummary hash_summary = run_samples(
        cfg, dataset,
        [&](std::span<const Vec3> position, std::span<const Quat> orientation,
            std::span<const ContactManifold> previous_manifolds, std::span<ContactManifold> next_manifolds) {
            return refresh_hash_lookup(position, orientation, previous_manifolds, dataset.previous_lookup,
                                       next_manifolds);
        });
    const SampleSummary merge_summary = run_samples(
        cfg, dataset,
        [&](std::span<const Vec3> position, std::span<const Quat> orientation,
            std::span<const ContactManifold> previous_manifolds, std::span<ContactManifold> next_manifolds) {
            return refresh_linear_merge(position, orientation, previous_manifolds, next_manifolds);
        });

    print_samples("hash lookup", hash_summary);
    print_samples("linear merge", merge_summary);

    const f64 hash_median = bench::median(hash_summary.us_per_iteration);
    const f64 merge_median = bench::median(merge_summary.us_per_iteration);
    const f64 speedup = (merge_median > 0.0) ? (hash_median / merge_median) : 0.0;

    std::println("Speedup (median, hash/linear): {}x", speedup);
    return 0;
}

import std;

import javelin.bench.cli;
import javelin.bench.json;
import javelin.bench.stats;
import javelin.bench.timer;
import javelin.core.time;
import javelin.core.types;
import javelin.math;
import javelin.physics.solve;
import javelin.physics.types;

using namespace javelin;

namespace {

struct Config final {
    u32 bodies = 2048;
    u32 manifolds = 9000;
    u32 iterations = 120;
    u32 warmup = 20;
    u32 samples = 7;
    u32 seed = 1337;
    f32 motion_scale = 1.0f;
    f32 warm_start_scale = 1.0f;
    f32 separation_scale = 1.0f;
    f32 adaptive_epsilon = 1e-3f;
    std::string json_out{};
};

struct Dataset final {
    std::vector<Vec3> velocity_seed{};
    std::vector<Vec3> angular_velocity_seed{};
    std::vector<Vec3> velocity_work{};
    std::vector<Vec3> angular_velocity_work{};
    std::vector<f32> inv_mass{};
    std::vector<Vec3> inv_inertia_body{};
    std::vector<Quat> orientation{};
    std::vector<u8> asleep{};
    std::vector<ContactManifold> manifolds_seed{};
    std::vector<ContactManifold> manifolds_work{};
    std::vector<f32> manifold_restitution{};
    std::vector<f32> manifold_friction{};
};

struct SampleSummary final {
    std::vector<f64> us_per_iteration{};
    f64 avg_manifolds_per_iteration{};
    f64 avg_points_per_iteration{};
    u64 checksum{};
};

struct RunMode final {
    std::string_view label{};
    ContactSolveConfig solve_config{};
};

[[nodiscard]] bool parse_arg(std::string_view arg, Config &cfg) {
    bench::ParsedArg parsed{};
    if (!bench::split_key_value_arg(arg, parsed)) {
        return false;
    }

    if (parsed.key == "bodies") {
        return bench::parse_u32(parsed.value, cfg.bodies);
    }
    if (parsed.key == "manifolds") {
        return bench::parse_u32(parsed.value, cfg.manifolds);
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
    if (parsed.key == "motion-scale") {
        return bench::parse_f32(parsed.value, cfg.motion_scale);
    }
    if (parsed.key == "warm-start-scale") {
        return bench::parse_f32(parsed.value, cfg.warm_start_scale);
    }
    if (parsed.key == "separation-scale") {
        return bench::parse_f32(parsed.value, cfg.separation_scale);
    }
    if (parsed.key == "adaptive-epsilon") {
        return bench::parse_f32(parsed.value, cfg.adaptive_epsilon);
    }
    if (parsed.key == "json-out") {
        cfg.json_out = std::string{parsed.value};
        return !cfg.json_out.empty();
    }
    return false;
}

void print_usage(const char *exe, const Config &cfg) {
    std::println("Contact solver kernel benchmark");
    std::println("Usage: {} [--bodies=N] [--manifolds=N] [--iterations=N] "
                 "[--warmup=N] [--samples=N] [--seed=N]",
                 exe);
    std::println("             [--motion-scale=F] [--warm-start-scale=F] "
                 "[--separation-scale=F]");
    std::println("             [--adaptive-epsilon=F] [--json-out=PATH]\n");
    std::println("Defaults:");
    std::println("  --bodies={}", cfg.bodies);
    std::println("  --manifolds={}", cfg.manifolds);
    std::println("  --iterations={}", cfg.iterations);
    std::println("  --warmup={}", cfg.warmup);
    std::println("  --samples={}", cfg.samples);
    std::println("  --seed={}", cfg.seed);
    std::println("  --motion-scale={}", cfg.motion_scale);
    std::println("  --warm-start-scale={}", cfg.warm_start_scale);
    std::println("  --separation-scale={}", cfg.separation_scale);
    std::println("  --adaptive-epsilon={}", cfg.adaptive_epsilon);
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

[[nodiscard]] ContactPoint random_contact_point(std::mt19937 &rng, const f32 warm_start_scale,
                                                const f32 separation_scale) {
    std::uniform_real_distribution<f32> anchor_dist{-0.15f, 0.15f};
    std::uniform_real_distribution<f32> separation_dist{-0.02f * separation_scale, 0.004f * separation_scale};
    std::uniform_real_distribution<f32> impulse_dist{0.0f, 1.0f};
    std::uniform_int_distribution<u32> feature_dist{0u, 2048u};
    ContactPoint point{};
    point.local_anchor_a = Vec3{anchor_dist(rng), anchor_dist(rng), anchor_dist(rng)};
    point.local_anchor_b = Vec3{anchor_dist(rng), anchor_dist(rng), anchor_dist(rng)};
    point.separation = separation_dist(rng);
    point.normal_impulse = impulse_dist(rng) * warm_start_scale;
    point.tangent_impulse = random_vec3(rng, -0.3f * warm_start_scale, 0.3f * warm_start_scale);
    point.persisted = true;
    point.feature_id = feature_dist(rng);
    return point;
}

[[nodiscard]] u32 contact_point_count(std::span<const ContactManifold> manifolds) noexcept {
    u32 point_count = 0u;
    for (const ContactManifold &manifold : manifolds) {
        point_count += manifold.point_count;
    }
    return point_count;
}

[[nodiscard]] Dataset build_dataset(const Config &cfg) {
    Dataset out{};
    std::mt19937 rng{cfg.seed};

    out.velocity_seed.resize(cfg.bodies);
    out.angular_velocity_seed.resize(cfg.bodies);
    out.velocity_work.resize(cfg.bodies);
    out.angular_velocity_work.resize(cfg.bodies);
    out.inv_mass.resize(cfg.bodies, 1.0f);
    out.inv_inertia_body.resize(cfg.bodies);
    out.orientation.resize(cfg.bodies);
    out.asleep.resize(cfg.bodies, 0u);

    for (u32 i = 0u; i < cfg.bodies; ++i) {
        out.velocity_seed[i] = random_vec3(rng, -1.5f * cfg.motion_scale, 1.5f * cfg.motion_scale);
        out.angular_velocity_seed[i] = random_vec3(rng, -3.0f * cfg.motion_scale, 3.0f * cfg.motion_scale);
        out.inv_inertia_body[i] = Vec3{1.0f, 1.0f, 1.0f};
        out.orientation[i] = random_orientation(rng);
    }

    out.manifolds_seed.resize(cfg.manifolds);
    out.manifolds_work.resize(cfg.manifolds);
    out.manifold_restitution.resize(cfg.manifolds);
    out.manifold_friction.resize(cfg.manifolds);

    std::uniform_int_distribution<u32> body_dist{0u, cfg.bodies - 1u};
    std::uniform_int_distribution<u32> point_count_dist{1u, kMaxManifoldPoints};
    std::uniform_real_distribution<f32> rest_dist{0.0f, 0.6f};
    std::uniform_real_distribution<f32> fric_dist{0.2f, 1.0f};
    std::uniform_real_distribution<f32> ground_dist{0.0f, 1.0f};

    for (u32 i = 0u; i < cfg.manifolds; ++i) {
        ContactManifold manifold{};
        manifold.a = body_dist(rng);
        if (ground_dist(rng) < 0.08f) {
            manifold.b = kInvalidBody;
        } else {
            u32 b = body_dist(rng);
            if (b == manifold.a) {
                b = (b + 1u) % cfg.bodies;
            }
            manifold.b = b;
        }
        manifold.normal = random_unit_vec3(rng);
        manifold.point_count = point_count_dist(rng);
        manifold.manifold_feature_id = i;
        for (u32 point_index = 0u; point_index < manifold.point_count; ++point_index) {
            manifold.points[point_index] = random_contact_point(rng, cfg.warm_start_scale, cfg.separation_scale);
        }

        out.manifolds_seed[i] = manifold;
        out.manifold_restitution[i] = rest_dist(rng);
        out.manifold_friction[i] = fric_dist(rng);
    }

    return out;
}

[[nodiscard]] SampleSummary run_samples(const Config &cfg, Dataset &dataset, const ContactSolveConfig &solve_config) {
    SampleSummary summary{};
    summary.us_per_iteration.reserve(cfg.samples);

    const auto run_iterations = [&](const u32 iteration_count, const bool collect_stats) -> std::chrono::nanoseconds {
        u64 checksum = 0u;
        u64 manifold_sum = 0u;
        u64 point_sum = 0u;

        using clock = SteadyClock;
        const auto start = clock::now();
        for (u32 iter = 0u; iter < iteration_count; ++iter) {
            std::copy(dataset.velocity_seed.begin(), dataset.velocity_seed.end(), dataset.velocity_work.begin());
            std::copy(dataset.angular_velocity_seed.begin(), dataset.angular_velocity_seed.end(),
                      dataset.angular_velocity_work.begin());
            std::copy(dataset.manifolds_seed.begin(), dataset.manifolds_seed.end(), dataset.manifolds_work.begin());

            solve_contact_velocities(dataset.velocity_work, dataset.angular_velocity_work, dataset.inv_mass,
                                     dataset.inv_inertia_body, dataset.orientation, dataset.manifolds_work,
                                     1.0f / 60.0f, dataset.manifold_restitution, dataset.manifold_friction,
                                     dataset.asleep, solve_config);

            if (collect_stats) {
                const u32 manifold_count = static_cast<u32>(dataset.manifolds_work.size());
                const u32 point_count = contact_point_count(dataset.manifolds_work);
                manifold_sum += manifold_count;
                point_sum += point_count;
                checksum += static_cast<u64>(manifold_count) * 3u + static_cast<u64>(point_count) * 5u;
            }
        }

        if (collect_stats) {
            summary.checksum += checksum;
            summary.avg_manifolds_per_iteration += static_cast<f64>(manifold_sum);
            summary.avg_points_per_iteration += static_cast<f64>(point_sum);
        }
        return std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now() - start);
    };

    static_cast<void>(run_iterations(cfg.warmup, false));
    for (u32 sample = 0u; sample < cfg.samples; ++sample) {
        const auto elapsed = run_iterations(cfg.iterations, true);
        summary.us_per_iteration.push_back(bench::us_per_iteration(elapsed, static_cast<u64>(cfg.iterations)));
    }

    const f64 total_iterations = static_cast<f64>(cfg.samples) * static_cast<f64>(cfg.iterations);
    summary.avg_manifolds_per_iteration /= total_iterations;
    summary.avg_points_per_iteration /= total_iterations;
    return summary;
}

void print_summary(const SampleSummary &summary) {
    for (usize i = 0u; i < summary.us_per_iteration.size(); ++i) {
        std::println("  sample {}: {} us/iter", i + 1u, summary.us_per_iteration[i]);
    }
    std::println("  median: {} us/iter", bench::median(summary.us_per_iteration));
    std::println("  p95: {} us/iter", bench::p95(summary.us_per_iteration));
    std::println("  avg manifolds/iter: {}", summary.avg_manifolds_per_iteration);
    std::println("  avg contact points/iter: {}", summary.avg_points_per_iteration);
    std::println("  checksum: {}", summary.checksum);
}

[[nodiscard]] bool write_json_summary(const std::string &path, const Config &cfg, const SampleSummary &fixed_summary,
                                      const SampleSummary &adaptive_summary, const f64 fixed_median_us,
                                      const f64 fixed_p95_us, const f64 adaptive_median_us, const f64 adaptive_p95_us,
                                      const f64 adaptive_speedup) {
    std::ofstream out{};
    if (!bench::open_json_output(path, out)) {
        return false;
    }

    out << "{\n";
    out << "  \"bench\": \"contact_solver_kernel\",\n";
    out << "  \"units\": \"us/iter\",\n";
    out << "  \"config\": {\n";
    out << "    \"bodies\": " << cfg.bodies << ",\n";
    out << "    \"manifolds\": " << cfg.manifolds << ",\n";
    out << "    \"iterations\": " << cfg.iterations << ",\n";
    out << "    \"warmup\": " << cfg.warmup << ",\n";
    out << "    \"samples\": " << cfg.samples << ",\n";
    out << "    \"seed\": " << cfg.seed << ",\n";
    out << "    \"motion_scale\": " << cfg.motion_scale << ",\n";
    out << "    \"warm_start_scale\": " << cfg.warm_start_scale << ",\n";
    out << "    \"separation_scale\": " << cfg.separation_scale << ",\n";
    out << "    \"adaptive_epsilon\": " << cfg.adaptive_epsilon << "\n";
    out << "  },\n";
    out << "  \"compare_metrics\": {\n";
    out << "    \"fixed_median_us_per_iter\": " << fixed_median_us << ",\n";
    out << "    \"fixed_p95_us_per_iter\": " << fixed_p95_us << ",\n";
    out << "    \"adaptive_median_us_per_iter\": " << adaptive_median_us << ",\n";
    out << "    \"adaptive_p95_us_per_iter\": " << adaptive_p95_us << "\n";
    out << "  },\n";
    out << "  \"summary\": {\n";
    out << "    \"adaptive_median_speedup_x\": " << adaptive_speedup << "\n";
    out << "  },\n";
    out << "  \"modes\": {\n";
    out << "    \"fixed\": {\n";
    out << "      \"samples_us_per_iter\": ";
    bench::write_json_f64_array(out, fixed_summary.us_per_iteration);
    out << ",\n";
    out << "      \"median_us_per_iter\": " << fixed_median_us << ",\n";
    out << "      \"p95_us_per_iter\": " << fixed_p95_us << ",\n";
    out << "      \"avg_manifolds_per_iteration\": " << fixed_summary.avg_manifolds_per_iteration << ",\n";
    out << "      \"avg_contact_points_per_iteration\": " << fixed_summary.avg_points_per_iteration << ",\n";
    out << "      \"checksum\": " << fixed_summary.checksum << "\n";
    out << "    },\n";
    out << "    \"adaptive\": {\n";
    out << "      \"samples_us_per_iter\": ";
    bench::write_json_f64_array(out, adaptive_summary.us_per_iteration);
    out << ",\n";
    out << "      \"median_us_per_iter\": " << adaptive_median_us << ",\n";
    out << "      \"p95_us_per_iter\": " << adaptive_p95_us << ",\n";
    out << "      \"avg_manifolds_per_iteration\": " << adaptive_summary.avg_manifolds_per_iteration << ",\n";
    out << "      \"avg_contact_points_per_iteration\": " << adaptive_summary.avg_points_per_iteration << ",\n";
    out << "      \"checksum\": " << adaptive_summary.checksum << "\n";
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

    if (cfg.bodies < 2u || cfg.manifolds == 0u || cfg.iterations == 0u || cfg.samples == 0u ||
        cfg.motion_scale < 0.0f || cfg.warm_start_scale < 0.0f || cfg.separation_scale < 0.0f ||
        cfg.adaptive_epsilon <= 0.0f) {
        std::cerr << "Invalid config: counts must be > 0, bodies >= 2, scales must "
                     "be >= 0, adaptive epsilon > 0.\n";
        return 1;
    }

    Dataset dataset = build_dataset(cfg);
    const RunMode fixed_mode{
        .label = "fixed (deterministic 16 iterations)",
        .solve_config =
            ContactSolveConfig{
                .adaptive_iteration_cap = false,
                .max_iterations = 16u,
                .min_iterations_before_adapt = 4u,
                .adaptive_impulse_epsilon = 1e-4f,
            },
    };
    const RunMode adaptive_mode{
        .label = "adaptive cap (min=4 max=16)",
        .solve_config =
            ContactSolveConfig{
                .adaptive_iteration_cap = true,
                .max_iterations = 16u,
                .min_iterations_before_adapt = 4u,
                .adaptive_impulse_epsilon = cfg.adaptive_epsilon,
            },
    };
    const SampleSummary fixed_summary = run_samples(cfg, dataset, fixed_mode.solve_config);
    const SampleSummary adaptive_summary = run_samples(cfg, dataset, adaptive_mode.solve_config);

    std::println("Config:");
    std::println("  bodies: {}", cfg.bodies);
    std::println("  manifolds: {}", cfg.manifolds);
    std::println("  warmup: {}", cfg.warmup);
    std::println("  iterations: {}", cfg.iterations);
    std::println("  samples: {}", cfg.samples);
    std::println("  seed: {}", cfg.seed);
    std::println("  motion scale: {}", cfg.motion_scale);
    std::println("  warm-start scale: {}", cfg.warm_start_scale);
    std::println("  separation scale: {}", cfg.separation_scale);
    std::println("  adaptive epsilon: {}", cfg.adaptive_epsilon);
    std::println("");

    std::println("{}:", fixed_mode.label);
    print_summary(fixed_summary);
    std::println("");
    std::println("{}:", adaptive_mode.label);
    print_summary(adaptive_summary);

    const f64 fixed_median_us = bench::median(fixed_summary.us_per_iteration);
    const f64 fixed_p95_us = bench::p95(fixed_summary.us_per_iteration);
    const f64 adaptive_median_us = bench::median(adaptive_summary.us_per_iteration);
    const f64 adaptive_p95_us = bench::p95(adaptive_summary.us_per_iteration);
    const f64 adaptive_speedup =
        (adaptive_median_us > 0.0) ? (fixed_median_us / adaptive_median_us) : std::numeric_limits<f64>::quiet_NaN();
    std::println("");
    std::println("Adaptive delta:");
    std::println("  median speedup: {}x", adaptive_speedup);

    if (!cfg.json_out.empty()) {
        if (!write_json_summary(cfg.json_out, cfg, fixed_summary, adaptive_summary, fixed_median_us, fixed_p95_us,
                                adaptive_median_us, adaptive_p95_us, adaptive_speedup)) {
            return 1;
        }
    }
    return 0;
}

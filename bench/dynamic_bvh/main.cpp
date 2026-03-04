import std;

import javelin.bench.cli;
import javelin.bench.json;
import javelin.bench.stats;
import javelin.bench.timer;
import javelin.core.time;
import javelin.core.types;
import javelin.math.vec3;
import javelin.physics.aabb;
import javelin.physics.bvh_dynamic;

using namespace javelin;

namespace {

struct Config final {
    u32 bodies = 20000;
    u32 queries = 2000;
    u32 iterations = 50;
    u32 warmup = 5;
    u32 samples = 5;
    u32 seed = 1337;
    f32 world_extent = 1000.0f;
    f32 body_radius = 1.0f;
    f32 query_radius = 5.0f;
    std::string json_out{};
};

void print_usage(const char *exe, const Config &cfg) {
    std::println("Dynamic BVH microbench");
    std::println("Usage: {} [--bodies=N] [--queries=N] [--iterations=N] "
                 "[--warmup=N] [--samples=N]",
                 exe);
    std::println("             [--seed=N] [--world=F] [--body-radius=F] "
                 "[--query-radius=F]");
    std::println("             [--json-out=PATH]\\n");
    std::println("Defaults:");
    std::println("  --bodies={}", cfg.bodies);
    std::println("  --queries={}", cfg.queries);
    std::println("  --iterations={}", cfg.iterations);
    std::println("  --warmup={}", cfg.warmup);
    std::println("  --samples={}", cfg.samples);
    std::println("  --seed={}", cfg.seed);
    std::println("  --world={}", cfg.world_extent);
    std::println("  --body-radius={}", cfg.body_radius);
    std::println("  --query-radius={}", cfg.query_radius);
}

[[nodiscard]] bool parse_arg(const std::string_view arg, Config &cfg) {
    bench::ParsedArg parsed{};
    if (!bench::split_key_value_arg(arg, parsed)) {
        return false;
    }

    if (parsed.key == "bodies") {
        return bench::parse_u32(parsed.value, cfg.bodies);
    }
    if (parsed.key == "queries") {
        return bench::parse_u32(parsed.value, cfg.queries);
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
    if (parsed.key == "world") {
        return bench::parse_f32(parsed.value, cfg.world_extent);
    }
    if (parsed.key == "body-radius") {
        return bench::parse_f32(parsed.value, cfg.body_radius);
    }
    if (parsed.key == "query-radius") {
        return bench::parse_f32(parsed.value, cfg.query_radius);
    }
    if (parsed.key == "json-out") {
        cfg.json_out = std::string{parsed.value};
        return !cfg.json_out.empty();
    }

    return false;
}

[[nodiscard]] bool write_json_summary(const std::string &path, const Config &cfg, const std::span<const f64> samples,
                                      const f64 median_ns_per_query, const f64 p95_ns_per_query,
                                      const f64 avg_hits_per_query, const u64 checksum, const usize node_count) {
    std::ofstream out{};
    if (!bench::open_json_output(path, out)) {
        return false;
    }

    out << "{\n";
    out << "  \"bench\": \"dynamic_bvh\",\n";
    out << "  \"units\": \"ns/query\",\n";
    out << "  \"config\": {\n";
    out << "    \"bodies\": " << cfg.bodies << ",\n";
    out << "    \"queries\": " << cfg.queries << ",\n";
    out << "    \"iterations\": " << cfg.iterations << ",\n";
    out << "    \"warmup\": " << cfg.warmup << ",\n";
    out << "    \"samples\": " << cfg.samples << ",\n";
    out << "    \"seed\": " << cfg.seed << ",\n";
    out << "    \"world\": " << cfg.world_extent << ",\n";
    out << "    \"body_radius\": " << cfg.body_radius << ",\n";
    out << "    \"query_radius\": " << cfg.query_radius << "\n";
    out << "  },\n";
    out << "  \"samples_ns_per_query\": ";
    bench::write_json_f64_array(out, samples);
    out << ",\n";
    out << "  \"compare_metrics\": {\n";
    out << "    \"median_ns_per_query\": " << median_ns_per_query << ",\n";
    out << "    \"p95_ns_per_query\": " << p95_ns_per_query << "\n";
    out << "  },\n";
    out << "  \"summary\": {\n";
    out << "    \"median_ns_per_query\": " << median_ns_per_query << ",\n";
    out << "    \"p95_ns_per_query\": " << p95_ns_per_query << ",\n";
    out << "    \"avg_hits_per_query\": " << avg_hits_per_query << ",\n";
    out << "    \"checksum\": " << checksum << ",\n";
    out << "    \"nodes\": " << node_count << "\n";
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

    if (cfg.bodies == 0 || cfg.queries == 0 || cfg.iterations == 0 || cfg.samples == 0) {
        std::cerr << "bodies, queries, iterations, and samples must all be > 0.\n";
        return 1;
    }

    DynamicBvh bvh{};
    bvh.reserve(cfg.bodies);

    std::mt19937 rng{cfg.seed};
    std::uniform_real_distribution<f32> dist{-cfg.world_extent, cfg.world_extent};

    const Vec3 body_extent{cfg.body_radius};
    for (u32 i = 0; i < cfg.bodies; ++i) {
        const Vec3 center{dist(rng), dist(rng), dist(rng)};
        bvh.insert(i, Aabb{center - body_extent, center + body_extent});
    }

    std::vector<Aabb> queries;
    queries.reserve(cfg.queries);

    const Vec3 query_extent{cfg.query_radius};
    for (u32 i = 0; i < cfg.queries; ++i) {
        const Vec3 center{dist(rng), dist(rng), dist(rng)};
        queries.emplace_back(center - query_extent, center + query_extent);
    }

    std::vector<u32> out;
    std::vector<u32> stack;
    out.reserve(256);
    stack.reserve(256);

    const auto run_iterations = [&](const u32 iterations, u64 &checksum) {
        using clock = SteadyClock;
        const auto start = clock::now();
        for (u32 iter = 0; iter < iterations; ++iter) {
            for (const Aabb &query : queries) {
                bvh.query(query, out, stack);
                checksum += static_cast<u64>(out.size());
                out.clear();
            }
        }
        return std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now() - start);
    };

    u64 warmup_checksum = 0;
    run_iterations(cfg.warmup, warmup_checksum);

    const u64 total_queries = static_cast<u64>(cfg.iterations) * static_cast<u64>(queries.size());

    std::vector<f64> per_query_ns;
    per_query_ns.reserve(cfg.samples);

    u64 total_checksum = 0;
    for (u32 sample = 0; sample < cfg.samples; ++sample) {
        u64 checksum = 0;
        const auto elapsed = run_iterations(cfg.iterations, checksum);
        total_checksum += checksum;

        const f64 ns_per_query = bench::ns_per_item(elapsed, total_queries);
        per_query_ns.push_back(ns_per_query);

        std::println("sample {}: {} ns/query ({} ms)", sample + 1, ns_per_query, bench::duration_ms(elapsed));
    }

    const f64 median_ns = bench::median(per_query_ns);
    const f64 p95_ns = bench::p95(per_query_ns);
    const f64 avg_hits_per_query = static_cast<f64>(total_checksum) / static_cast<f64>(total_queries * cfg.samples);

    std::println("");
    std::println("Config:");
    std::println("  bodies: {}", cfg.bodies);
    std::println("  queries: {}", cfg.queries);
    std::println("  iterations: {}", cfg.iterations);
    std::println("  warmup: {}", cfg.warmup);
    std::println("  samples: {}", cfg.samples);
    std::println("  seed: {}", cfg.seed);
    std::println("  world: {}", cfg.world_extent);
    std::println("  body radius: {}", cfg.body_radius);
    std::println("  query radius: {}", cfg.query_radius);
    std::println("  nodes: {}", bvh.nodes().size());
    std::println("");
    std::println("Summary:");
    std::println("  median: {} ns/query", median_ns);
    std::println("  p95: {} ns/query", p95_ns);
    std::println("  avg hits/query: {}", avg_hits_per_query);
    std::println("  checksum: {}", total_checksum);

    if (!cfg.json_out.empty()) {
        if (!write_json_summary(cfg.json_out, cfg, per_query_ns, median_ns, p95_ns, avg_hits_per_query, total_checksum,
                                bvh.nodes().size())) {
            return 1;
        }
    }

    return 0;
}

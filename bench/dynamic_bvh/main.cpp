import std;

import javelin.core.types;
import javelin.core.time;
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
    std::println("Dynamic BVH microbench");
    std::println("Usage: {} [--bodies=N] [--queries=N] [--iterations=N] "
                 "[--warmup=N] [--samples=N]",
                 exe);
    std::println("             [--seed=N] [--world=F] [--body-radius=F] "
                 "[--query-radius=F]");
    std::println("             [--json-out=PATH]\n");
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
    if (key == "queries") {
        return parse_u32(value, cfg.queries);
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
    if (key == "world") {
        return parse_f32(value, cfg.world_extent);
    }
    if (key == "body-radius") {
        return parse_f32(value, cfg.body_radius);
    }
    if (key == "query-radius") {
        return parse_f32(value, cfg.query_radius);
    }
    if (key == "json-out") {
        cfg.json_out = std::string{value};
        return !cfg.json_out.empty();
    }

    return false;
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

[[nodiscard]] bool write_json_summary(const std::string &path, const Config &cfg, std::span<const double> samples,
                                      const double median_ns_per_query, const double p95_ns_per_query,
                                      const double avg_hits_per_query, const u64 checksum, const usize node_count) {
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
    out << "  \"samples_ns_per_query\": [";
    for (usize i = 0u; i < samples.size(); ++i) {
        if (i > 0u) {
            out << ", ";
        }
        out << samples[i];
    }
    out << "],\n";
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

    std::vector<double> per_query_ns;
    per_query_ns.reserve(cfg.samples);

    u64 total_checksum = 0;
    for (u32 sample = 0; sample < cfg.samples; ++sample) {
        u64 checksum = 0;
        const auto elapsed = run_iterations(cfg.iterations, checksum);
        total_checksum += checksum;

        const double ns = static_cast<double>(elapsed.count());
        const double ns_per_query = ns / static_cast<double>(total_queries);
        per_query_ns.push_back(ns_per_query);

        std::println("sample {}: {} ns/query ({} ms)", sample + 1, ns_per_query, to_ms(elapsed));
    }

    const double median_ns = median(per_query_ns);
    const double p95_ns = p95(per_query_ns);
    const double avg_hits_per_query =
        static_cast<double>(total_checksum) / static_cast<double>(total_queries * cfg.samples);

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

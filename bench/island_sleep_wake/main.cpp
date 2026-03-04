import std;

import javelin.bench.cli;
import javelin.bench.json;
import javelin.bench.stats;
import javelin.bench.timer;
import javelin.core.time;
import javelin.core.types;

using namespace javelin;

namespace {

constexpr u32 kSleepTickThreshold = 60u;
constexpr u32 kInvalidBody = std::numeric_limits<u32>::max();
constexpr u32 kInvalidIsland = std::numeric_limits<u32>::max();

struct Config final {
    u32 bodies = 2048;
    u32 ticks = 1200;
    u32 warmup = 200;
    u32 samples = 7;
    u32 seed = 1337;
    std::string json_out{};
};

struct Edge final {
    u32 a{};
    u32 b{};
};

struct SampleSummary final {
    std::vector<f64> us_per_tick{};
    f64 avg_awake_bodies_per_tick{};
    f64 avg_active_edges_per_tick{};
    u64 checksum{};
};

enum struct Mode : u8 {
    per_body = 0,
    island = 1,
};

struct PolicyState final {
    std::vector<u8> asleep{};
    std::vector<u32> sleep_timer{};
    std::vector<u8> activity_mask{};
    std::vector<Edge> active_edges{};
};

struct IslandState final {
    std::vector<u32> parent{};
    std::vector<u8> rank{};
    std::vector<u32> member_head{};
    std::vector<u32> member_next{};
    std::vector<u32> member_count{};
    std::vector<u32> roots{};

    std::vector<u32> sleep_island_of_body{};
    std::vector<u32> sleep_island_next_body{};
    std::vector<u32> sleep_island_head{};
    std::vector<u32> sleep_island_size{};
    std::vector<u32> sleep_island_free_ids{};
};

[[nodiscard]] bool parse_arg(std::string_view arg, Config &cfg) {
    bench::ParsedArg parsed{};
    if (!bench::split_key_value_arg(arg, parsed)) {
        return false;
    }

    if (parsed.key == "bodies") {
        return bench::parse_u32(parsed.value, cfg.bodies);
    }
    if (parsed.key == "ticks") {
        return bench::parse_u32(parsed.value, cfg.ticks);
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

void print_usage(const char *exe, const Config &cfg) {
    std::println("Island sleep/wake benchmark");
    std::println("Usage: {} [--bodies=N] [--ticks=N] [--warmup=N] [--samples=N] "
                 "[--seed=N] [--json-out=PATH]\n",
                 exe);
    std::println("Defaults:");
    std::println("  --bodies={}", cfg.bodies);
    std::println("  --ticks={}", cfg.ticks);
    std::println("  --warmup={}", cfg.warmup);
    std::println("  --samples={}", cfg.samples);
    std::println("  --seed={}", cfg.seed);
}

[[nodiscard]] std::vector<Edge> build_chain_edges(const u32 body_count) {
    std::vector<Edge> edges{};
    if (body_count <= 1u) {
        return edges;
    }
    edges.reserve(body_count - 1u);
    for (u32 i = 0u; i + 1u < body_count; ++i) {
        edges.push_back(Edge{.a = i, .b = i + 1u});
    }
    return edges;
}

[[nodiscard]] PolicyState build_initial_state(const u32 body_count, const u32 seed) {
    PolicyState out{};
    out.asleep.assign(body_count, 0u);
    out.sleep_timer.resize(body_count);
    out.activity_mask.assign(body_count, 0u);

    std::mt19937 rng{seed};
    std::uniform_int_distribution<u32> timer_dist{0u, kSleepTickThreshold - 1u};
    for (u32 i = 0u; i < body_count; ++i) {
        out.sleep_timer[i] = timer_dist(rng);
    }
    return out;
}

void build_active_edges(const std::vector<Edge> &all_edges, std::span<const u8> asleep, std::vector<Edge> &out_edges,
                        std::span<u8> out_activity_mask) {
    std::fill(out_activity_mask.begin(), out_activity_mask.end(), static_cast<u8>(0u));
    out_edges.clear();
    out_edges.reserve(all_edges.size());

    for (const Edge edge : all_edges) {
        const bool a_awake = asleep[edge.a] == 0u;
        const bool b_awake = asleep[edge.b] == 0u;
        if (!a_awake && !b_awake) {
            continue;
        }
        out_edges.push_back(edge);
        out_activity_mask[edge.a] = 1u;
        out_activity_mask[edge.b] = 1u;
    }
}

[[nodiscard]] u32 count_awake(std::span<const u8> asleep) noexcept {
    u32 awake = 0u;
    for (const u8 value : asleep) {
        awake += (value == 0u) ? 1u : 0u;
    }
    return awake;
}

void update_sleep_timers(std::span<const u8> activity_mask, std::span<u8> asleep, std::span<u32> sleep_timer) noexcept {
    for (u32 i = 0u; i < asleep.size(); ++i) {
        if (asleep[i] != 0u) {
            continue;
        }
        if (activity_mask[i] != 0u) {
            ++sleep_timer[i];
        } else {
            sleep_timer[i] = 0u;
        }
    }
}

void wake_sleeping_bodies_per_body(std::span<const Edge> active_edges, std::span<u8> asleep,
                                   std::span<u32> sleep_timer) noexcept {
    for (const Edge edge : active_edges) {
        const bool a_awake = asleep[edge.a] == 0u;
        const bool b_awake = asleep[edge.b] == 0u;
        if (asleep[edge.a] != 0u && b_awake) {
            asleep[edge.a] = 0u;
            sleep_timer[edge.a] = 0u;
        }
        if (asleep[edge.b] != 0u && a_awake) {
            asleep[edge.b] = 0u;
            sleep_timer[edge.b] = 0u;
        }
    }
}

void mark_bodies_asleep_per_body(std::span<const u32> sleep_timer, std::span<u8> asleep) noexcept {
    for (u32 i = 0u; i < asleep.size(); ++i) {
        if (sleep_timer[i] >= kSleepTickThreshold) {
            asleep[i] = 1u;
        }
    }
}

[[nodiscard]] u32 island_find_root(IslandState &state, const u32 body) {
    u32 root = body;
    while (state.parent[root] != root) {
        root = state.parent[root];
    }

    u32 current = body;
    while (state.parent[current] != current) {
        const u32 parent = state.parent[current];
        state.parent[current] = root;
        current = parent;
    }
    return root;
}

void island_union(IslandState &state, const u32 lhs, const u32 rhs) {
    u32 root_lhs = island_find_root(state, lhs);
    u32 root_rhs = island_find_root(state, rhs);
    if (root_lhs == root_rhs) {
        return;
    }

    const u8 rank_lhs = state.rank[root_lhs];
    const u8 rank_rhs = state.rank[root_rhs];
    if (rank_lhs < rank_rhs) {
        std::swap(root_lhs, root_rhs);
    }
    state.parent[root_rhs] = root_lhs;
    if (rank_lhs == rank_rhs) {
        ++state.rank[root_lhs];
    }
}

void build_islands(IslandState &state, const u32 body_count, std::span<const Edge> active_edges) {
    if (state.parent.size() != body_count) {
        state.parent.resize(body_count);
        state.rank.resize(body_count);
        state.member_head.resize(body_count);
        state.member_next.resize(body_count);
        state.member_count.resize(body_count);
    }

    for (u32 i = 0u; i < body_count; ++i) {
        state.parent[i] = i;
        state.rank[i] = 0u;
        state.member_head[i] = kInvalidBody;
        state.member_next[i] = kInvalidBody;
        state.member_count[i] = 0u;
    }
    state.roots.clear();
    state.roots.reserve(body_count);

    for (const Edge edge : active_edges) {
        island_union(state, edge.a, edge.b);
    }

    for (u32 body = 0u; body < body_count; ++body) {
        const u32 root = island_find_root(state, body);
        if (state.member_head[root] == kInvalidBody) {
            state.roots.push_back(root);
        }
        state.member_next[body] = state.member_head[root];
        state.member_head[root] = body;
        ++state.member_count[root];
    }
}

[[nodiscard]] u32 allocate_sleep_island_id(IslandState &state) {
    if (!state.sleep_island_free_ids.empty()) {
        const u32 id = state.sleep_island_free_ids.back();
        state.sleep_island_free_ids.pop_back();
        return id;
    }
    const u32 id = static_cast<u32>(state.sleep_island_head.size());
    state.sleep_island_head.push_back(kInvalidBody);
    state.sleep_island_size.push_back(0u);
    return id;
}

[[nodiscard]] u32 wake_sleep_island(IslandState &state, const u32 island_id, std::span<u8> asleep,
                                    std::span<u32> sleep_timer) {
    if (island_id == kInvalidIsland || island_id >= state.sleep_island_head.size()) {
        return 0u;
    }
    u32 body = state.sleep_island_head[island_id];
    if (body == kInvalidBody) {
        return 0u;
    }

    u32 woken = 0u;
    while (body != kInvalidBody) {
        const u32 next = state.sleep_island_next_body[body];
        asleep[body] = 0u;
        sleep_timer[body] = 0u;
        state.sleep_island_of_body[body] = kInvalidIsland;
        state.sleep_island_next_body[body] = kInvalidBody;
        ++woken;
        body = next;
    }

    state.sleep_island_head[island_id] = kInvalidBody;
    state.sleep_island_size[island_id] = 0u;
    state.sleep_island_free_ids.push_back(island_id);
    return woken;
}

void wake_sleeping_islands(std::span<const Edge> active_edges, IslandState &state, std::span<u8> asleep,
                           std::span<u32> sleep_timer) {
    auto wake_if_sleeping = [&](const u32 body) {
        if (asleep[body] == 0u) {
            return;
        }
        const u32 island_id = state.sleep_island_of_body[body];
        if (island_id == kInvalidIsland) {
            asleep[body] = 0u;
            sleep_timer[body] = 0u;
            return;
        }
        static_cast<void>(wake_sleep_island(state, island_id, asleep, sleep_timer));
    };

    for (const Edge edge : active_edges) {
        const bool a_awake = asleep[edge.a] == 0u;
        const bool b_awake = asleep[edge.b] == 0u;
        if (a_awake == b_awake) {
            continue;
        }
        wake_if_sleeping(a_awake ? edge.b : edge.a);
    }
}

void register_sleep_island_for_component(IslandState &state, const u32 component_head) {
    if (component_head == kInvalidBody) {
        return;
    }
    const u32 island_id = allocate_sleep_island_id(state);
    u32 member_count = 0u;
    u32 body = component_head;
    while (body != kInvalidBody) {
        const u32 next = state.member_next[body];
        state.sleep_island_of_body[body] = island_id;
        state.sleep_island_next_body[body] = state.sleep_island_head[island_id];
        state.sleep_island_head[island_id] = body;
        ++member_count;
        body = next;
    }
    state.sleep_island_size[island_id] = member_count;
}

void sleep_awake_islands(IslandState &state, std::span<u8> asleep, std::span<u32> sleep_timer) {
    for (const u32 root : state.roots) {
        const u32 component_head = state.member_head[root];
        if (component_head == kInvalidBody) {
            continue;
        }

        bool has_awake_member = false;
        bool all_awake_members_ready = true;
        u32 body = component_head;
        while (body != kInvalidBody) {
            if (asleep[body] == 0u) {
                has_awake_member = true;
                if (sleep_timer[body] < kSleepTickThreshold) {
                    all_awake_members_ready = false;
                }
            }
            body = state.member_next[body];
        }

        if (!has_awake_member || !all_awake_members_ready) {
            continue;
        }

        body = component_head;
        while (body != kInvalidBody) {
            asleep[body] = 1u;
            sleep_timer[body] = std::max(sleep_timer[body], kSleepTickThreshold);
            body = state.member_next[body];
        }
        register_sleep_island_for_component(state, component_head);
    }
}

[[nodiscard]] SampleSummary run_mode_samples(const Config &cfg, const std::vector<Edge> &edges, const Mode mode) {
    SampleSummary summary{};
    summary.us_per_tick.reserve(cfg.samples);
    u64 awake_total = 0u;
    u64 edge_total = 0u;

    for (u32 sample = 0u; sample < cfg.samples; ++sample) {
        PolicyState state = build_initial_state(cfg.bodies, cfg.seed + sample);
        state.active_edges.reserve(edges.size());
        IslandState island{};
        if (mode == Mode::island) {
            island.sleep_island_of_body.assign(cfg.bodies, kInvalidIsland);
            island.sleep_island_next_body.assign(cfg.bodies, kInvalidBody);
            island.sleep_island_head.clear();
            island.sleep_island_size.clear();
            island.sleep_island_free_ids.clear();
        }

        const auto run_ticks = [&](const u32 tick_count, const bool collect_stats) -> std::chrono::nanoseconds {
            using clock = SteadyClock;
            const auto start = clock::now();
            for (u32 tick = 0u; tick < tick_count; ++tick) {
                build_active_edges(edges, state.asleep, state.active_edges, state.activity_mask);

                if (mode == Mode::per_body) {
                    wake_sleeping_bodies_per_body(state.active_edges, state.asleep, state.sleep_timer);
                } else {
                    build_islands(island, cfg.bodies, state.active_edges);
                    wake_sleeping_islands(state.active_edges, island, state.asleep, state.sleep_timer);
                }

                update_sleep_timers(state.activity_mask, state.asleep, state.sleep_timer);
                if (mode == Mode::per_body) {
                    mark_bodies_asleep_per_body(state.sleep_timer, state.asleep);
                } else {
                    sleep_awake_islands(island, state.asleep, state.sleep_timer);
                }

                if (collect_stats) {
                    const u32 awake = count_awake(state.asleep);
                    const u32 active_edge_count = static_cast<u32>(state.active_edges.size());
                    awake_total += awake;
                    edge_total += active_edge_count;
                    summary.checksum += static_cast<u64>(awake) * 3u + static_cast<u64>(active_edge_count) * 5u;
                }
            }
            return std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now() - start);
        };

        static_cast<void>(run_ticks(cfg.warmup, false));
        const auto measured_ns = run_ticks(cfg.ticks, true);
        summary.us_per_tick.push_back(bench::us_per_iteration(measured_ns, static_cast<u64>(cfg.ticks)));
    }

    const f64 total_ticks = static_cast<f64>(cfg.samples) * static_cast<f64>(cfg.ticks);
    summary.avg_awake_bodies_per_tick = static_cast<f64>(awake_total) / total_ticks;
    summary.avg_active_edges_per_tick = static_cast<f64>(edge_total) / total_ticks;
    return summary;
}

void print_summary(const std::string_view label, const SampleSummary &summary) {
    std::println("{}:", label);
    for (usize i = 0u; i < summary.us_per_tick.size(); ++i) {
        std::println("  sample {}: {} us/tick", i + 1u, summary.us_per_tick[i]);
    }
    std::println("  median: {} us/tick", bench::median(summary.us_per_tick));
    std::println("  p95: {} us/tick", bench::p95(summary.us_per_tick));
    std::println("  avg awake bodies/tick: {}", summary.avg_awake_bodies_per_tick);
    std::println("  avg active edges/tick: {}", summary.avg_active_edges_per_tick);
    std::println("  checksum: {}", summary.checksum);
    std::println("");
}

[[nodiscard]] bool write_json_summary(const std::string &path, const Config &cfg, const u32 edge_count,
                                      const SampleSummary &per_body, const SampleSummary &island,
                                      const f64 per_body_median_us, const f64 per_body_p95_us,
                                      const f64 island_median_us, const f64 island_p95_us, const f64 runtime_speedup,
                                      const f64 awake_reduction, const f64 edge_reduction) {
    std::ofstream out{};
    if (!bench::open_json_output(path, out)) {
        return false;
    }

    out << "{\n";
    out << "  \"bench\": \"island_sleep_wake\",\n";
    out << "  \"units\": \"us/tick\",\n";
    out << "  \"config\": {\n";
    out << "    \"bodies\": " << cfg.bodies << ",\n";
    out << "    \"chain_edges\": " << edge_count << ",\n";
    out << "    \"ticks\": " << cfg.ticks << ",\n";
    out << "    \"warmup\": " << cfg.warmup << ",\n";
    out << "    \"samples\": " << cfg.samples << ",\n";
    out << "    \"seed\": " << cfg.seed << ",\n";
    out << "    \"sleep_tick_threshold\": " << kSleepTickThreshold << "\n";
    out << "  },\n";
    out << "  \"compare_metrics\": {\n";
    out << "    \"per_body_median_us_per_tick\": " << per_body_median_us << ",\n";
    out << "    \"per_body_p95_us_per_tick\": " << per_body_p95_us << ",\n";
    out << "    \"island_median_us_per_tick\": " << island_median_us << ",\n";
    out << "    \"island_p95_us_per_tick\": " << island_p95_us << "\n";
    out << "  },\n";
    out << "  \"summary\": {\n";
    out << "    \"runtime_speedup_median_x\": " << runtime_speedup << ",\n";
    out << "    \"awake_bodies_reduction_ratio\": " << awake_reduction << ",\n";
    out << "    \"active_edges_reduction_ratio\": " << edge_reduction << "\n";
    out << "  },\n";
    out << "  \"modes\": {\n";
    out << "    \"per_body\": {\n";
    out << "      \"samples_us_per_tick\": ";
    bench::write_json_f64_array(out, per_body.us_per_tick);
    out << ",\n";
    out << "      \"median_us_per_tick\": " << per_body_median_us << ",\n";
    out << "      \"p95_us_per_tick\": " << per_body_p95_us << ",\n";
    out << "      \"avg_awake_bodies_per_tick\": " << per_body.avg_awake_bodies_per_tick << ",\n";
    out << "      \"avg_active_edges_per_tick\": " << per_body.avg_active_edges_per_tick << ",\n";
    out << "      \"checksum\": " << per_body.checksum << "\n";
    out << "    },\n";
    out << "    \"island\": {\n";
    out << "      \"samples_us_per_tick\": ";
    bench::write_json_f64_array(out, island.us_per_tick);
    out << ",\n";
    out << "      \"median_us_per_tick\": " << island_median_us << ",\n";
    out << "      \"p95_us_per_tick\": " << island_p95_us << ",\n";
    out << "      \"avg_awake_bodies_per_tick\": " << island.avg_awake_bodies_per_tick << ",\n";
    out << "      \"avg_active_edges_per_tick\": " << island.avg_active_edges_per_tick << ",\n";
    out << "      \"checksum\": " << island.checksum << "\n";
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

    if (cfg.bodies < 2u || cfg.ticks == 0u || cfg.samples == 0u) {
        std::cerr << "bodies must be >= 2 and ticks/samples must be > 0.\n";
        return 1;
    }

    const std::vector<Edge> edges = build_chain_edges(cfg.bodies);
    const SampleSummary per_body = run_mode_samples(cfg, edges, Mode::per_body);
    const SampleSummary island = run_mode_samples(cfg, edges, Mode::island);

    std::println("Config:");
    std::println("  bodies: {}", cfg.bodies);
    std::println("  chain edges: {}", edges.size());
    std::println("  sleep threshold: {}", kSleepTickThreshold);
    std::println("  warmup ticks: {}", cfg.warmup);
    std::println("  measured ticks: {}", cfg.ticks);
    std::println("  samples: {}", cfg.samples);
    std::println("  seed: {}", cfg.seed);
    std::println("");

    print_summary("per-body sleep/wake (legacy)", per_body);
    print_summary("island sleep/wake", island);

    const f64 per_body_median_us = bench::median(per_body.us_per_tick);
    const f64 per_body_p95_us = bench::p95(per_body.us_per_tick);
    const f64 island_median_us = bench::median(island.us_per_tick);
    const f64 island_p95_us = bench::p95(island.us_per_tick);
    const f64 runtime_speedup =
        (island_median_us > 0.0) ? (per_body_median_us / island_median_us) : std::numeric_limits<f64>::quiet_NaN();

    const f64 awake_reduction = (per_body.avg_awake_bodies_per_tick > 0.0)
                                    ? (1.0 - island.avg_awake_bodies_per_tick / per_body.avg_awake_bodies_per_tick)
                                    : 0.0;
    const f64 edge_reduction = (per_body.avg_active_edges_per_tick > 0.0)
                                   ? (1.0 - island.avg_active_edges_per_tick / per_body.avg_active_edges_per_tick)
                                   : 0.0;

    std::println("Workload delta (island vs per-body):");
    std::println("  awake bodies reduction: {}%", awake_reduction * 100.0);
    std::println("  active edges reduction: {}%", edge_reduction * 100.0);
    std::println("  policy runtime speedup (median): {}x", runtime_speedup);

    if (!cfg.json_out.empty()) {
        if (!write_json_summary(cfg.json_out, cfg, static_cast<u32>(edges.size()), per_body, island, per_body_median_us,
                                per_body_p95_us, island_median_us, island_p95_us, runtime_speedup, awake_reduction,
                                edge_reduction)) {
            return 1;
        }
    }
    return 0;
}

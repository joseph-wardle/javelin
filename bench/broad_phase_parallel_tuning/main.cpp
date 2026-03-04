import std;

import javelin.bench.cli;
import javelin.bench.stats;
import javelin.bench.timer;
import javelin.core.time;
import javelin.core.types;
import javelin.math.vec3;
import javelin.physics.aabb;
import javelin.physics.broad_phase;
import javelin.physics.bvh_dynamic;
import javelin.physics.bvh_static;
import javelin.physics.types;

using namespace javelin;

namespace {

struct Config final {
    u32 bodies = 4096;
    std::vector<u32> active_counts = {256u, 512u, 1024u, 2048u, 4096u};
    f32 spacing = 0.92f;
    f32 half_extent = 0.5f;
    u32 iterations = 100;
    u32 warmup = 20;
    u32 samples = 7;
    u32 seed = 1337;
    u32 workers = 0u;
};

struct Dataset final {
    std::vector<Aabb> bounds{};
    DynamicBvh dynamic_bvh{};
    StaticBvh static_bvh{};
    std::vector<u32> shuffled_dynamic_ids{};
};

struct Workload final {
    std::vector<u32> query_dynamic_ids{};
    std::vector<u8> query_dynamic_mask{};
};

struct Dispatch final {
    u32 worker_count{};
    u32 chunk_size{};
};

struct SampleSummary final {
    std::vector<f64> us_per_iteration{};
    f64 avg_pairs_per_iteration{};
    f64 avg_workers_per_iteration{};
    f64 avg_chunk_size_per_iteration{};
    u64 checksum{};
};

enum struct Mode : u8 {
    baseline = 0,
    tuned = 1,
};

// Keep these constants in sync with PhysicsSystem broad-phase dispatch policy.
inline constexpr u32 kParallelMinQueries = 64u;
inline constexpr u32 kTargetQueriesPerWorker = 16u;
inline constexpr u32 kMinQueriesPerWorker = 8u;

[[nodiscard]] bool parse_u32_list(std::string_view text, std::vector<u32> &out) {
    return bench::parse_u32_csv(text, out);
}

void print_usage(const char *exe, const Config &cfg) {
    std::println("Broad-phase parallel dispatch benchmark");
    std::println("Usage: {} [--bodies=N] [--active-counts=N,N,...] [--spacing=F] "
                 "[--half-extent=F]",
                 exe);
    std::println("             [--iterations=N] [--warmup=N] [--samples=N] "
                 "[--seed=N] [--workers=N]\n");
    std::println("Defaults:");
    std::println("  --bodies={}", cfg.bodies);
    std::println("  --active-counts={}", [&cfg] {
        std::string text{};
        for (usize i = 0; i < cfg.active_counts.size(); ++i) {
            if (i > 0u) {
                text += ",";
            }
            text += std::to_string(cfg.active_counts[i]);
        }
        return text;
    }());
    std::println("  --spacing={}", cfg.spacing);
    std::println("  --half-extent={}", cfg.half_extent);
    std::println("  --iterations={}", cfg.iterations);
    std::println("  --warmup={}", cfg.warmup);
    std::println("  --samples={}", cfg.samples);
    std::println("  --seed={}", cfg.seed);
    std::println("  --workers={} (0=hardware_concurrency)", cfg.workers);
}

[[nodiscard]] bool parse_arg(std::string_view arg, Config &cfg) {
    bench::ParsedArg parsed{};
    if (!bench::split_key_value_arg(arg, parsed)) {
        return false;
    }

    if (parsed.key == "bodies") {
        return bench::parse_u32(parsed.value, cfg.bodies);
    }
    if (parsed.key == "active-counts") {
        return parse_u32_list(parsed.value, cfg.active_counts);
    }
    if (parsed.key == "spacing") {
        return bench::parse_f32(parsed.value, cfg.spacing);
    }
    if (parsed.key == "half-extent") {
        return bench::parse_f32(parsed.value, cfg.half_extent);
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
    if (parsed.key == "workers") {
        return bench::parse_u32(parsed.value, cfg.workers);
    }
    return false;
}

[[nodiscard]] static u32 ceil_div_u32(const u32 numerator, const u32 denominator) noexcept {
    return (numerator + denominator - 1u) / denominator;
}

[[nodiscard]] Dispatch baseline_dispatch(const u32 query_count, const u32 max_workers) noexcept {
    if (query_count == 0u || max_workers == 0u) {
        return Dispatch{};
    }
    const u32 worker_count = std::min(max_workers, query_count);
    return Dispatch{.worker_count = worker_count, .chunk_size = ceil_div_u32(query_count, worker_count)};
}

[[nodiscard]] Dispatch tuned_dispatch(const u32 query_count, const u32 max_workers) noexcept {
    if (query_count == 0u || max_workers == 0u) {
        return Dispatch{};
    }
    if (max_workers == 1u || query_count < kParallelMinQueries) {
        return Dispatch{.worker_count = 1u, .chunk_size = query_count};
    }
    const u32 worker_cap_by_chunk = std::max<u32>(1u, query_count / kMinQueriesPerWorker);
    const u32 worker_cap = std::min(max_workers, worker_cap_by_chunk);
    const u32 worker_target = std::max<u32>(1u, ceil_div_u32(query_count, kTargetQueriesPerWorker));
    const u32 worker_count = std::clamp(worker_target, 1u, worker_cap);
    return Dispatch{.worker_count = worker_count, .chunk_size = ceil_div_u32(query_count, worker_count)};
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
    if (pairs.empty()) {
        return;
    }
    for (BodyPair &pair : pairs) {
        pair = canonical_body_pair(pair.a, pair.b);
    }
    std::sort(pairs.begin(), pairs.end(), body_pair_less);
    const auto unique_end = std::unique(pairs.begin(), pairs.end(), body_pair_equal);
    pairs.erase(unique_end, pairs.end());
}

[[nodiscard]] bool pairs_equal(std::span<const BodyPair> lhs, std::span<const BodyPair> rhs) noexcept {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (usize i = 0; i < lhs.size(); ++i) {
        if (lhs[i].a != rhs[i].a || lhs[i].b != rhs[i].b) {
            return false;
        }
    }
    return true;
}

[[nodiscard]] Dataset build_dataset(const Config &cfg) {
    Dataset out{};
    out.bounds.resize(cfg.bodies);
    out.shuffled_dynamic_ids.resize(cfg.bodies);
    std::iota(out.shuffled_dynamic_ids.begin(), out.shuffled_dynamic_ids.end(), 0u);

    const u32 side = static_cast<u32>(std::ceil(std::cbrt(static_cast<f64>(cfg.bodies))));
    for (u32 i = 0u; i < cfg.bodies; ++i) {
        const u32 x = i % side;
        const u32 y = (i / side) % side;
        const u32 z = i / (side * side);
        const Vec3 center{
            static_cast<f32>(x) * cfg.spacing,
            static_cast<f32>(y) * cfg.spacing,
            static_cast<f32>(z) * cfg.spacing,
        };
        const Vec3 extents{cfg.half_extent};
        out.bounds[i] = Aabb{center - extents, center + extents};
    }

    std::mt19937 rng{cfg.seed};
    std::shuffle(out.shuffled_dynamic_ids.begin(), out.shuffled_dynamic_ids.end(), rng);

    out.dynamic_bvh.reserve(cfg.bodies);
    std::vector<u32> dynamic_ids{};
    dynamic_ids.resize(cfg.bodies);
    std::iota(dynamic_ids.begin(), dynamic_ids.end(), 0u);
    broad_phase_update_dynamic_bvh(dynamic_ids, out.dynamic_bvh, out.bounds);
    return out;
}

[[nodiscard]] Workload build_workload(const Dataset &dataset, const u32 body_count, const u32 active_count) {
    Workload workload{};
    const u32 clamped_active_count = std::min(active_count, body_count);
    workload.query_dynamic_ids.assign(dataset.shuffled_dynamic_ids.begin(),
                                      dataset.shuffled_dynamic_ids.begin() + clamped_active_count);
    std::sort(workload.query_dynamic_ids.begin(), workload.query_dynamic_ids.end());
    workload.query_dynamic_mask.assign(body_count, static_cast<u8>(0u));
    for (const u32 id : workload.query_dynamic_ids) {
        workload.query_dynamic_mask[id] = 1u;
    }
    return workload;
}

class BroadPhaseWorkerPool final {
  public:
    explicit BroadPhaseWorkerPool(const u32 worker_count, const u32 reserve_count)
        : worker_count_{std::max<u32>(worker_count, 1u)}, workers_(worker_count_) {
        for (auto &worker : workers_) {
            worker.scratch.reserve(reserve_count, 2u);
            worker.pairs.reserve(static_cast<usize>(reserve_count) * 8u);
        }
        pair_offsets_.reserve(static_cast<usize>(worker_count_) + 1u);
        threads_.reserve(worker_count_ - 1u);
        for (u32 worker_index = 1u; worker_index < worker_count_; ++worker_index) {
            threads_.emplace_back([this, worker_index] { worker_loop_(worker_index); });
        }
    }

    ~BroadPhaseWorkerPool() { stop_(); }

    BroadPhaseWorkerPool(const BroadPhaseWorkerPool &) = delete;
    BroadPhaseWorkerPool &operator=(const BroadPhaseWorkerPool &) = delete;

    [[nodiscard]] u32 worker_count() const noexcept { return worker_count_; }

    void run(const DynamicBvh &dynamic_bvh, const StaticBvh &static_bvh, std::span<const Aabb> bounds_cache,
             std::span<const u32> query_dynamic_ids, std::span<const u8> query_dynamic_mask, const Dispatch dispatch,
             std::vector<BodyPair> &out_pairs) {
        out_pairs.clear();
        if (dispatch.worker_count == 0u || query_dynamic_ids.empty()) {
            return;
        }

        const Job job{
            .dynamic_bvh = &dynamic_bvh,
            .static_bvh = &static_bvh,
            .bounds_cache = bounds_cache,
            .query_dynamic_ids = query_dynamic_ids,
            .query_dynamic_mask = query_dynamic_mask,
            .query_dynamic_count = static_cast<u32>(query_dynamic_ids.size()),
            .worker_count = dispatch.worker_count,
            .chunk_size = dispatch.chunk_size,
        };

        if (dispatch.worker_count > 1u) {
            {
                std::lock_guard lock(mutex_);
                job_ = job;
                jobs_remaining_.store(dispatch.worker_count - 1u, std::memory_order_release);
                ++job_id_;
            }
            cv_.notify_all();
        }

        run_chunk_(job, 0u);

        if (dispatch.worker_count > 1u) {
            std::unique_lock lock(mutex_);
            done_cv_.wait(lock, [&] { return jobs_remaining_.load(std::memory_order_acquire) == 0u; });
        }

        if (dispatch.worker_count == 1u) {
            const auto &pairs = workers_[0].pairs;
            out_pairs.assign(pairs.begin(), pairs.end());
            return;
        }

        pair_offsets_.resize(static_cast<usize>(dispatch.worker_count) + 1u);
        pair_offsets_[0] = 0u;
        for (u32 worker_index = 0u; worker_index < dispatch.worker_count; ++worker_index) {
            const usize count = workers_[worker_index].pairs.size();
            pair_offsets_[static_cast<usize>(worker_index) + 1u] = pair_offsets_[worker_index] + count;
        }

        const usize total_pairs = pair_offsets_[dispatch.worker_count];
        out_pairs.resize(total_pairs);
        for (u32 worker_index = 0u; worker_index < dispatch.worker_count; ++worker_index) {
            const auto &src = workers_[worker_index].pairs;
            const usize offset = pair_offsets_[worker_index];
            std::copy(src.begin(), src.end(), out_pairs.data() + offset);
        }
    }

  private:
    struct Worker final {
        BroadPhaseScratch scratch{};
        std::vector<BodyPair> pairs{};
    };

    struct Job final {
        const DynamicBvh *dynamic_bvh{};
        const StaticBvh *static_bvh{};
        std::span<const Aabb> bounds_cache{};
        std::span<const u32> query_dynamic_ids{};
        std::span<const u8> query_dynamic_mask{};
        u32 query_dynamic_count{};
        u32 worker_count{};
        u32 chunk_size{};
    };

    u32 worker_count_{1u};
    std::vector<Worker> workers_{};
    std::vector<std::thread> threads_{};
    std::vector<usize> pair_offsets_{};
    std::mutex mutex_{};
    std::condition_variable cv_{};
    std::condition_variable done_cv_{};
    Job job_{};
    u64 job_id_{0u};
    std::atomic<u32> jobs_remaining_{0u};
    bool stop_requested_{false};

    void run_chunk_(const Job &job, const u32 worker_index) {
        if (worker_index >= job.worker_count) {
            return;
        }
        const u32 begin = worker_index * job.chunk_size;
        if (begin >= job.query_dynamic_count) {
            workers_[worker_index].pairs.clear();
            return;
        }
        const u32 end = std::min(begin + job.chunk_size, job.query_dynamic_count);
        const std::span<const u32> chunk{job.query_dynamic_ids.data() + begin, end - begin};
        Worker &worker = workers_[worker_index];
        broad_phase_generate_pairs(chunk, *job.dynamic_bvh, *job.static_bvh, job.bounds_cache, job.query_dynamic_mask,
                                   worker.pairs, worker.scratch);
    }

    void worker_loop_(const u32 worker_index) {
        u64 last_job = 0u;
        for (;;) {
            Job job{};
            {
                std::unique_lock lock(mutex_);
                cv_.wait(lock, [&] { return stop_requested_ || job_id_ != last_job; });
                if (stop_requested_) {
                    return;
                }
                last_job = job_id_;
                job = job_;
            }
            if (worker_index >= job.worker_count) {
                continue;
            }
            run_chunk_(job, worker_index);
            if (jobs_remaining_.fetch_sub(1u, std::memory_order_acq_rel) == 1u) {
                std::lock_guard lock(mutex_);
                done_cv_.notify_one();
            }
        }
    }

    void stop_() noexcept {
        if (threads_.empty()) {
            return;
        }
        {
            std::lock_guard lock(mutex_);
            stop_requested_ = true;
            ++job_id_;
        }
        cv_.notify_all();
        for (auto &thread : threads_) {
            thread.join();
        }
        threads_.clear();
        stop_requested_ = false;
    }
};

[[nodiscard]] bool validate_dispatch_equivalence(BroadPhaseWorkerPool &pool, const Dataset &dataset,
                                                 const Workload &workload) {
    std::vector<BodyPair> baseline_pairs{};
    std::vector<BodyPair> tuned_pairs{};
    const Dispatch baseline =
        baseline_dispatch(static_cast<u32>(workload.query_dynamic_ids.size()), pool.worker_count());
    const Dispatch tuned = tuned_dispatch(static_cast<u32>(workload.query_dynamic_ids.size()), pool.worker_count());
    pool.run(dataset.dynamic_bvh, dataset.static_bvh, dataset.bounds, workload.query_dynamic_ids,
             workload.query_dynamic_mask, baseline, baseline_pairs);
    normalize_pairs(baseline_pairs);
    pool.run(dataset.dynamic_bvh, dataset.static_bvh, dataset.bounds, workload.query_dynamic_ids,
             workload.query_dynamic_mask, tuned, tuned_pairs);
    normalize_pairs(tuned_pairs);
    return pairs_equal(baseline_pairs, tuned_pairs);
}

[[nodiscard]] SampleSummary run_mode_samples(const Config &cfg, const Dataset &dataset, const Workload &workload,
                                             BroadPhaseWorkerPool &pool, const Mode mode) {
    SampleSummary summary{};
    summary.us_per_iteration.reserve(cfg.samples);
    std::vector<BodyPair> pairs{};
    pairs.reserve(static_cast<usize>(workload.query_dynamic_ids.size()) * 8u);

    const auto run_iterations = [&](const u32 iteration_count, u64 &checksum_out, u64 &pairs_out, u64 &workers_out,
                                    u64 &chunk_size_out) -> std::chrono::nanoseconds {
        using clock = SteadyClock;
        const auto start = clock::now();
        const u32 query_count = static_cast<u32>(workload.query_dynamic_ids.size());
        for (u32 iter = 0u; iter < iteration_count; ++iter) {
            const Dispatch dispatch = (mode == Mode::baseline) ? baseline_dispatch(query_count, pool.worker_count())
                                                               : tuned_dispatch(query_count, pool.worker_count());
            pool.run(dataset.dynamic_bvh, dataset.static_bvh, dataset.bounds, workload.query_dynamic_ids,
                     workload.query_dynamic_mask, dispatch, pairs);
            normalize_pairs(pairs);
            pairs_out += pairs.size();
            workers_out += dispatch.worker_count;
            chunk_size_out += dispatch.chunk_size;
            if (!pairs.empty()) {
                const BodyPair first = pairs.front();
                const BodyPair last = pairs.back();
                checksum_out += (static_cast<u64>(first.a) << 48u) ^ (static_cast<u64>(first.b) << 32u) ^
                                (static_cast<u64>(last.a) << 16u) ^ static_cast<u64>(last.b);
            }
            checksum_out += static_cast<u64>(pairs.size());
        }
        return std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now() - start);
    };

    u64 warmup_checksum = 0u;
    u64 warmup_pairs = 0u;
    u64 warmup_workers = 0u;
    u64 warmup_chunk = 0u;
    run_iterations(cfg.warmup, warmup_checksum, warmup_pairs, warmup_workers, warmup_chunk);

    u64 checksum = 0u;
    u64 pair_total = 0u;
    u64 worker_total = 0u;
    u64 chunk_total = 0u;
    for (u32 sample = 0u; sample < cfg.samples; ++sample) {
        u64 sample_checksum = 0u;
        u64 sample_pairs = 0u;
        u64 sample_workers = 0u;
        u64 sample_chunk = 0u;
        const auto elapsed =
            run_iterations(cfg.iterations, sample_checksum, sample_pairs, sample_workers, sample_chunk);
        checksum += sample_checksum;
        pair_total += sample_pairs;
        worker_total += sample_workers;
        chunk_total += sample_chunk;
        const f64 us_per_iteration = bench::us_per_iteration(elapsed, static_cast<u64>(cfg.iterations));
        summary.us_per_iteration.push_back(us_per_iteration);
        std::println("  sample {}: {} us/iter", sample + 1u, us_per_iteration);
    }

    const f64 total_iterations = static_cast<f64>(cfg.iterations) * static_cast<f64>(cfg.samples);
    summary.avg_pairs_per_iteration = static_cast<f64>(pair_total) / total_iterations;
    summary.avg_workers_per_iteration = static_cast<f64>(worker_total) / total_iterations;
    summary.avg_chunk_size_per_iteration = static_cast<f64>(chunk_total) / total_iterations;
    summary.checksum = checksum;
    return summary;
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

    if (cfg.bodies == 0u || cfg.iterations == 0u || cfg.samples == 0u || cfg.active_counts.empty()) {
        std::cerr << "bodies, iterations, samples, and active-counts must all be > 0.\n";
        return 1;
    }
    for (u32 &active_count : cfg.active_counts) {
        active_count = std::clamp(active_count, 1u, cfg.bodies);
    }
    std::sort(cfg.active_counts.begin(), cfg.active_counts.end());
    cfg.active_counts.erase(std::unique(cfg.active_counts.begin(), cfg.active_counts.end()), cfg.active_counts.end());

    const u32 worker_count = (cfg.workers > 0u) ? cfg.workers : std::max<u32>(std::thread::hardware_concurrency(), 1u);
    const Dataset dataset = build_dataset(cfg);
    BroadPhaseWorkerPool pool{worker_count, cfg.bodies};

    std::println("Broad-phase parallel dispatch benchmark");
    std::println("  bodies: {}", cfg.bodies);
    std::println("  workers: {}", pool.worker_count());
    std::println("  iterations: {}", cfg.iterations);
    std::println("  warmup: {}", cfg.warmup);
    std::println("  samples: {}", cfg.samples);
    std::println("  spacing: {}", cfg.spacing);
    std::println("  half extent: {}", cfg.half_extent);
    std::println("");

    for (const u32 active_count : cfg.active_counts) {
        const Workload workload = build_workload(dataset, cfg.bodies, active_count);
        if (!validate_dispatch_equivalence(pool, dataset, workload)) {
            std::cerr << "Dispatch mismatch: tuned and baseline produced different "
                         "normalized pair sets "
                      << "(active-count=" << active_count << ").\n";
            return 2;
        }

        std::println("active-count={}", active_count);
        std::println(" baseline:");
        const SampleSummary baseline = run_mode_samples(cfg, dataset, workload, pool, Mode::baseline);
        std::println(" tuned:");
        const SampleSummary tuned = run_mode_samples(cfg, dataset, workload, pool, Mode::tuned);

        const f64 baseline_median = bench::median(baseline.us_per_iteration);
        const f64 tuned_median = bench::median(tuned.us_per_iteration);
        const f64 baseline_p95 = bench::p95(baseline.us_per_iteration);
        const f64 tuned_p95 = bench::p95(tuned.us_per_iteration);
        const f64 speedup = (tuned_median > 0.0) ? (baseline_median / tuned_median) : 0.0;

        std::println(" summary:");
        std::println("  baseline median: {} us/iter", baseline_median);
        std::println("  tuned median: {} us/iter", tuned_median);
        std::println("  baseline p95: {} us/iter", baseline_p95);
        std::println("  tuned p95: {} us/iter", tuned_p95);
        std::println("  speedup (median): {}x", speedup);
        std::println("  baseline avg workers: {}", baseline.avg_workers_per_iteration);
        std::println("  tuned avg workers: {}", tuned.avg_workers_per_iteration);
        std::println("  baseline avg chunk size: {}", baseline.avg_chunk_size_per_iteration);
        std::println("  tuned avg chunk size: {}", tuned.avg_chunk_size_per_iteration);
        std::println("  avg normalized pairs/iter: {}", tuned.avg_pairs_per_iteration);
        std::println("  checksum baseline={} tuned={}", baseline.checksum, tuned.checksum);
        std::println("");
    }

    return 0;
}

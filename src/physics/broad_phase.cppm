module;

#include <tracy/Tracy.hpp>

export module javelin.physics.broad_phase;

import std;
import javelin.core.logging;
import javelin.core.types;
import javelin.physics.aabb;
import javelin.physics.bvh_dynamic;
import javelin.physics.bvh_static;
import javelin.physics.types;

export namespace javelin {

// Broad phase contract:
// - input bounds are world-space AABBs for the current frame.
// - output pairs are potential overlaps only; narrow phase confirms contacts.
// - ordering inside each chunk follows dynamic id order; global canonicalization
//   and de-duplication are performed by BroadPhaseStage.
struct BroadPhaseScratch final {
    std::vector<u32> query_stack_overflow{};

    void reserve(const u32 count, const u32 query_stack_factor) {
        query_stack_overflow.reserve(static_cast<usize>(count) * query_stack_factor);
    }
};

void broad_phase_update_dynamic_bvh(std::span<const u32> dynamic_ids, DynamicBvh &dynamic_bvh,
                                    std::span<const Aabb> bounds_cache) {
    ZoneScopedN("Physics broad phase update");
    if (dynamic_ids.empty()) {
        return;
    }
    for (const u32 id : dynamic_ids) {
        dynamic_bvh.update(id, bounds_cache[id]);
    }
}

// Chunked query helper: operates on a contiguous span with per-thread scratch/output.
void broad_phase_chunk(std::span<const u32> query_dynamic_ids, const DynamicBvh &dynamic_bvh,
                       const StaticBvh &static_bvh, std::span<const Aabb> bounds_cache,
                       std::span<const u8> query_dynamic_mask, std::vector<BodyPair> &pairs,
                       BroadPhaseScratch &scratch) {
    ZoneScopedN("Physics broad phase query");
    pairs.clear();
    if (query_dynamic_ids.empty()) {
        return;
    }

    const bool has_static = !static_bvh.empty();
    // query_dynamic_ids are sorted ascending.
    // Duplicate rule:
    // - query-vs-query pairs are emitted once by id order.
    // - query-vs-nonquery pairs are always emitted (nonquery body never queries back).
    for (const u32 id : query_dynamic_ids) {
        const Aabb query_bounds = bounds_cache[id];

        dynamic_bvh.query_each_overlap(
            query_bounds,
            [&](const u32 j) {
                if (j == id) {
                    return;
                }
                const bool j_is_query_body = (j < query_dynamic_mask.size()) && query_dynamic_mask[j] != 0u;
                if (j_is_query_body && j < id) {
                    return;
                }
                pairs.push_back(BodyPair{.a = id, .b = j});
            },
            scratch.query_stack_overflow);

        if (!has_static) {
            continue;
        }

        static_bvh.query_each_overlap(
            query_bounds,
            [&](const u32 j) {
                if (j == id) {
                    return;
                }
                pairs.push_back(BodyPair{.a = id, .b = j});
            },
            scratch.query_stack_overflow);
    }
}

// Main entrypoint for broad phase pair generation.
void broad_phase_generate_pairs(std::span<const u32> query_dynamic_ids, const DynamicBvh &dynamic_bvh,
                                const StaticBvh &static_bvh, std::span<const Aabb> bounds_cache,
                                std::span<const u8> query_dynamic_mask, std::vector<BodyPair> &pairs,
                                BroadPhaseScratch &scratch) {
    broad_phase_chunk(query_dynamic_ids, dynamic_bvh, static_bvh, bounds_cache, query_dynamic_mask, pairs, scratch);
}

// Aggregated broad-phase stage: owns the worker pool, candidate pair storage,
// dispatch policy, carry-forward of previous manifolds, deterministic
// sort + dedup, and (optionally) the equivalence validator.
//
// Threading model:
// - the broad phase runs synchronously from the physics thread's perspective:
//   find_pairs() blocks until all workers finish their chunks.
// - worker threads live for the lifetime of the stage (started lazily on
//   first find_pairs call, joined on stop()).
//
// Worker pool sizing: uses `hardware_concurrency - 2` to leave room for the
// physics control thread and the renderer.  One worker minimum.
struct BroadPhaseStage final {
    struct CarryForwardStats final {
        u32 carried_pair_count{};
        u32 validated_pair_count{};
    };

    struct FindPairsInputs final {
        const DynamicBvh *dynamic_bvh{nullptr};
        const StaticBvh *static_bvh{nullptr};
        std::span<const Aabb> bounds{};
        // Bodies to query this tick (either the full awake set or the moved-awake subset).
        std::span<const u32> query_ids{};
        std::span<const u8> query_mask{};
        u32 body_count{0u};
        std::span<const u8> awake_dynamic_mask{};
        // Previous-tick manifolds, used for carry-forward when include_carry_forward is true.
        std::span<const ContactManifold> previous_manifolds{};
        // Set when query_ids is the moved-awake subset; emits carry-forward pairs for
        // unmoved overlaps that still need narrow-phase confirmation.
        bool include_carry_forward{false};
    };

    struct FindPairsResult final {
        std::span<const BodyPair> pairs{};
        CarryForwardStats carry_forward{};
    };

    // Capacity heuristics.  Same constants the orchestrator used previously.
    static constexpr u32 kQueryStackReserveFactor = 2;
    static constexpr u32 kPairReserveFactor = 8;
    // Below kParallelMinQueries, run single-threaded to avoid scheduler overhead.
    static constexpr u32 kParallelMinQueries = 64u;
    static constexpr u32 kTargetQueriesPerWorker = 16u;
    static constexpr u32 kMinQueriesPerWorker = 8u;
#if defined(JAVELIN_BROAD_PHASE_VALIDATE)
    // Debug validator cadence: full-query equivalence check every N ticks.
    static constexpr u32 kValidationIntervalTicks = 120u;
#endif

    void reserve(const u32 body_count) {
        ensure_workers_started_();
        if (body_count <= worker_reserve_count_) {
            reserve_worker_pair_buffers_(worker_pair_reserve_hint_);
            return;
        }
        for (auto &worker : workers_) {
            worker.reserve(body_count, kQueryStackReserveFactor, kPairReserveFactor);
        }
        worker_reserve_count_ = body_count;
        const usize pair_capacity_floor = static_cast<usize>(body_count) * kPairReserveFactor;
        update_reserve_hint_(pair_capacity_floor, worker_pair_reserve_hint_);
        reserve_worker_pair_buffers_(worker_pair_reserve_hint_);
        if (candidate_pair_reserve_hint_ > candidate_pairs_.capacity()) {
            candidate_pairs_.reserve(candidate_pair_reserve_hint_);
        }
#if defined(JAVELIN_BROAD_PHASE_VALIDATE)
        if (expected_pairs_.capacity() < pair_capacity_floor) {
            expected_pairs_.reserve(pair_capacity_floor);
        }
#endif
    }

    void stop() noexcept {
        if (threads_.empty()) {
            return;
        }
        {
            std::lock_guard lock(mutex_);
            stop_ = true;
            ++job_id_;
        }
        cv_.notify_all();
        for (auto &thread : threads_) {
            thread.join();
        }
        threads_.clear();
        stop_ = false;
        log::info(physics, "Broad phase workers stopped");
    }

    [[nodiscard]] FindPairsResult find_pairs(const FindPairsInputs &in) {
        ensure_workers_started_();
        if (candidate_pair_reserve_hint_ > candidate_pairs_.capacity()) {
            candidate_pairs_.reserve(candidate_pair_reserve_hint_);
        }

        run_queries_(in);
        CarryForwardStats carry_stats{};
        if (in.include_carry_forward) {
            carry_stats = append_overlapping_previous_pairs_(in.body_count, in.awake_dynamic_mask, in.bounds,
                                                             in.previous_manifolds);
        }
        normalize_and_sort_pairs_();
        update_reserve_hint_(candidate_pairs_.size(), candidate_pair_reserve_hint_);
        return FindPairsResult{
            .pairs = std::span<const BodyPair>{candidate_pairs_},
            .carry_forward = carry_stats,
        };
    }

#if defined(JAVELIN_BROAD_PHASE_VALIDATE)
    void maybe_validate(const DynamicBvh &dynamic_bvh, const StaticBvh &static_bvh, std::span<const Aabb> bounds,
                        std::span<const u32> awake_dynamic_ids, std::span<const u8> awake_dynamic_mask,
                        const bool used_full_awake_query, const u64 step_id) {
        if (used_full_awake_query) {
            TracyPlot("physics_broad_phase_validator_ran", static_cast<i64>(0));
            return;
        }
        if (kValidationIntervalTicks == 0u ||
            (step_id % static_cast<u64>(kValidationIntervalTicks)) != 0u) {
            TracyPlot("physics_broad_phase_validator_ran", static_cast<i64>(0));
            return;
        }

        ZoneScopedN("Physics validate broad phase incremental");
        BroadPhaseWorker &worker = workers_[0];
        expected_pairs_.clear();
        if (candidate_pairs_.size() > expected_pairs_.capacity()) {
            expected_pairs_.reserve(candidate_pairs_.size());
        }
        broad_phase_generate_pairs(awake_dynamic_ids, dynamic_bvh, static_bvh, bounds, awake_dynamic_mask,
                                   expected_pairs_, worker.scratch);
        normalize_and_sort_body_pairs_(expected_pairs_);
        TracyPlot("physics_broad_phase_validator_ran", static_cast<i64>(1));
        TracyPlot("physics_broad_phase_validator_expected_pairs", static_cast<i64>(expected_pairs_.size()));

        if (expected_pairs_.size() != candidate_pairs_.size()) {
            log::error(physics,
                       "Incremental broad phase validation failed (step={} expected_pairs={} actual_pairs={})",
                       step_id, expected_pairs_.size(), candidate_pairs_.size());
            std::terminate();
        }
        for (usize i = 0; i < candidate_pairs_.size(); ++i) {
            if (!body_pair_equal_(expected_pairs_[i], candidate_pairs_[i])) {
                const BodyPair expected = expected_pairs_[i];
                const BodyPair actual = candidate_pairs_[i];
                log::error(physics,
                           "Incremental broad phase validation failed (step={} index={} "
                           "expected=({}, {}) actual=({}, {}))",
                           step_id, i, expected.a, expected.b, actual.a, actual.b);
                std::terminate();
            }
        }
    }
#endif

    [[nodiscard]] std::span<const BodyPair> pairs() const noexcept { return std::span<const BodyPair>{candidate_pairs_}; }
    [[nodiscard]] usize candidate_pairs_capacity() const noexcept { return candidate_pairs_.capacity(); }
    [[nodiscard]] usize worker_pair_capacity_sum() const noexcept {
        usize total = 0u;
        for (const auto &worker : workers_) {
            total += worker.pairs.capacity();
        }
        return total;
    }

  private:
    struct BroadPhaseWorker final {
        BroadPhaseScratch scratch{};
        std::vector<BodyPair> pairs{};

        void reserve(const u32 count, const u32 query_stack_factor, const u32 pair_factor) {
            scratch.reserve(count, query_stack_factor);
            pairs.reserve(static_cast<usize>(count) * pair_factor);
        }
    };

    struct BroadPhaseJob final {
        std::span<const u32> query_dynamic_ids{};
        std::span<const u8> query_dynamic_mask{};
        std::span<const Aabb> bounds{};
        const DynamicBvh *dynamic_bvh{nullptr};
        const StaticBvh *static_bvh{nullptr};
        u32 query_dynamic_count{};
        u32 worker_count{};
        u32 chunk_size{};
    };

    struct BroadPhaseDispatch final {
        u32 worker_count{};
        u32 chunk_size{};
    };

    [[nodiscard]] static u32 ceil_div_u32_(const u32 numerator, const u32 denominator) noexcept {
        return (numerator + denominator - 1u) / denominator;
    }

    [[nodiscard]] static BroadPhaseDispatch choose_dispatch_(const u32 query_dynamic_count,
                                                             const u32 max_worker_count) noexcept {
        if (query_dynamic_count == 0u || max_worker_count == 0u) {
            return BroadPhaseDispatch{};
        }
        if (max_worker_count == 1u || query_dynamic_count < kParallelMinQueries) {
            return BroadPhaseDispatch{.worker_count = 1u, .chunk_size = query_dynamic_count};
        }
        const u32 worker_cap_by_chunk = std::max<u32>(1u, query_dynamic_count / kMinQueriesPerWorker);
        const u32 worker_cap = std::min(max_worker_count, worker_cap_by_chunk);
        const u32 worker_target = std::max<u32>(1u, ceil_div_u32_(query_dynamic_count, kTargetQueriesPerWorker));
        const u32 worker_count = std::clamp(worker_target, 1u, worker_cap);
        return BroadPhaseDispatch{
            .worker_count = worker_count,
            .chunk_size = ceil_div_u32_(query_dynamic_count, worker_count),
        };
    }

    [[nodiscard]] static usize grown_capacity_(const usize current_capacity, const usize required_capacity) noexcept {
        if (required_capacity <= current_capacity) {
            return current_capacity;
        }
        const usize base = std::max<usize>(current_capacity, 64u);
        const usize grown = base + base / 2u;
        return std::max(grown, required_capacity);
    }

    static void update_reserve_hint_(const usize observed_size, usize &reserve_hint) noexcept {
        if (observed_size <= reserve_hint) {
            return;
        }
        reserve_hint = grown_capacity_(reserve_hint, observed_size);
    }

    [[nodiscard]] static bool body_pair_less_(const BodyPair lhs, const BodyPair rhs) noexcept {
        if (lhs.a != rhs.a) {
            return lhs.a < rhs.a;
        }
        return lhs.b < rhs.b;
    }

    [[nodiscard]] static bool body_pair_equal_(const BodyPair lhs, const BodyPair rhs) noexcept {
        return lhs.a == rhs.a && lhs.b == rhs.b;
    }

    static void normalize_and_sort_body_pairs_(std::vector<BodyPair> &pairs) {
        if (pairs.empty()) {
            return;
        }
        for (BodyPair &pair : pairs) {
            pair = canonical_body_pair(pair.a, pair.b);
        }
        std::sort(pairs.begin(), pairs.end(), body_pair_less_);
        const auto unique_end = std::unique(pairs.begin(), pairs.end(), body_pair_equal_);
        pairs.erase(unique_end, pairs.end());
    }

    void normalize_and_sort_pairs_() {
        ZoneScopedN("Physics normalize candidate pairs");
        normalize_and_sort_body_pairs_(candidate_pairs_);
    }

    void ensure_workers_started_() {
        if (worker_count_ > 0u) {
            return;
        }
        const u32 hw_threads = std::thread::hardware_concurrency();
        const u32 usable_threads = (hw_threads > 2u) ? (hw_threads - 2u) : 1u;
        worker_count_ = (hw_threads > 0u) ? usable_threads : 1u;
        workers_.resize(worker_count_);
        threads_.reserve(worker_count_ - 1u);
        pair_offsets_.reserve(static_cast<usize>(worker_count_) + 1u);
        if (worker_count_ > 1u) {
            stop_ = false;
            for (u32 worker_index = 1; worker_index < worker_count_; ++worker_index) {
                threads_.emplace_back([this, worker_index] { worker_loop_(worker_index); });
            }
            log::info(physics, "Broad phase workers={}", worker_count_);
        }
    }

    void reserve_worker_pair_buffers_(const usize pair_capacity_hint) {
        if (pair_capacity_hint <= worker_pair_reserve_hint_) {
            return;
        }
        for (auto &worker : workers_) {
            worker.pairs.reserve(pair_capacity_hint);
        }
        worker_pair_reserve_hint_ = pair_capacity_hint;
    }

    void run_queries_(const FindPairsInputs &in) {
        ZoneScopedN("Physics broad phase parallel");
        candidate_pairs_.clear();
        const u32 query_dynamic_count = static_cast<u32>(in.query_ids.size());
        if (query_dynamic_count == 0u) {
            TracyPlot("physics_broad_phase_workers_used", static_cast<i64>(0));
            TracyPlot("physics_broad_phase_chunk_size", static_cast<i64>(0));
            return;
        }
#ifndef NDEBUG
        if (worker_count_ == 0u || workers_.empty()) {
            log::error(physics, "Broad phase workers not initialized before query dispatch");
            std::terminate();
        }
        if (!std::is_sorted(in.query_ids.begin(), in.query_ids.end())) {
            log::error(physics, "Broad phase query dynamic ids are not sorted");
            std::terminate();
        }
#endif

        const BroadPhaseDispatch dispatch = choose_dispatch_(query_dynamic_count, worker_count_);
        const u32 worker_count = dispatch.worker_count;
        const u32 chunk_size = dispatch.chunk_size;
        TracyPlot("physics_broad_phase_workers_used", static_cast<i64>(worker_count));
        TracyPlot("physics_broad_phase_chunk_size", static_cast<i64>(chunk_size));

        // Worker pool: fixed thread count, contiguous chunks, deterministic merge by chunk order.
        const BroadPhaseJob job{
            .query_dynamic_ids = in.query_ids,
            .query_dynamic_mask = in.query_mask,
            .bounds = in.bounds,
            .dynamic_bvh = in.dynamic_bvh,
            .static_bvh = in.static_bvh,
            .query_dynamic_count = query_dynamic_count,
            .worker_count = worker_count,
            .chunk_size = chunk_size,
        };

        if (worker_count > 1) {
            {
                std::lock_guard lock(mutex_);
                current_job_ = job;
                jobs_remaining_.store(worker_count - 1u, std::memory_order_release);
                ++job_id_;
            }
            cv_.notify_all();
        }

        run_chunk_(job, 0);

        if (worker_count > 1) {
            std::unique_lock lock(mutex_);
            done_cv_.wait(lock, [&] { return jobs_remaining_.load(std::memory_order_acquire) == 0u; });
        }

        {
            ZoneScopedN("Physics broad phase merge");
            usize max_worker_pair_count = 0u;
            if (worker_count == 1u) {
                const auto &pairs = workers_[0].pairs;
                candidate_pairs_.assign(pairs.begin(), pairs.end());
                max_worker_pair_count = pairs.size();
            } else {
                pair_offsets_.resize(static_cast<usize>(worker_count) + 1u);
                pair_offsets_[0] = 0;
                for (u32 worker_index = 0; worker_index < worker_count; ++worker_index) {
                    const usize count = workers_[worker_index].pairs.size();
                    max_worker_pair_count = std::max(max_worker_pair_count, count);
                    pair_offsets_[static_cast<usize>(worker_index) + 1u] = pair_offsets_[worker_index] + count;
                }
                const usize total_pairs = pair_offsets_[worker_count];
                candidate_pairs_.resize(total_pairs);
                for (u32 worker_index = 0; worker_index < worker_count; ++worker_index) {
                    auto &src = workers_[worker_index].pairs;
                    const usize offset = pair_offsets_[worker_index];
                    std::copy(src.begin(), src.end(), candidate_pairs_.data() + offset);
                }
            }
            update_reserve_hint_(max_worker_pair_count, worker_pair_reserve_hint_);
            reserve_worker_pair_buffers_(worker_pair_reserve_hint_);
        }
    }

    void run_chunk_(const BroadPhaseJob &job, const u32 worker_index) {
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
        BroadPhaseWorker &worker = workers_[worker_index];
        broad_phase_generate_pairs(chunk, *job.dynamic_bvh, *job.static_bvh, job.bounds, job.query_dynamic_mask,
                                   worker.pairs, worker.scratch);
    }

    void worker_loop_(const u32 worker_index) {
        thread_local std::string name;
        name = "Physics BroadPhase " + std::to_string(worker_index);
        tracy::SetThreadName(name.c_str());
        u64 last_job = 0;
        for (;;) {
            BroadPhaseJob job{};
            {
                std::unique_lock lock(mutex_);
                cv_.wait(lock, [&] { return stop_ || job_id_ != last_job; });
                if (stop_) {
                    return;
                }
                last_job = job_id_;
                job = current_job_;
            }

            if (worker_index >= job.worker_count) {
                continue;
            }
            {
                ZoneScopedN("Physics broad phase worker");
                run_chunk_(job, worker_index);
            }
            if (jobs_remaining_.fetch_sub(1u, std::memory_order_acq_rel) == 1u) {
                std::lock_guard lock(mutex_);
                done_cv_.notify_one();
            }
        }
    }

    [[nodiscard]] CarryForwardStats append_overlapping_previous_pairs_(const u32 body_count,
                                                                       std::span<const u8> awake_dynamic_mask,
                                                                       std::span<const Aabb> bounds,
                                                                       std::span<const ContactManifold> manifolds) {
        ZoneScopedN("Physics carry manifold pairs");
        CarryForwardStats stats{};
        if (manifolds.empty()) {
            return stats;
        }
        for (const ContactManifold &manifold : manifolds) {
            if (manifold.b == kInvalidBody) {
                continue;
            }
            const BodyPair pair = canonical_body_pair(manifold.a, manifold.b);
#ifndef NDEBUG
            if (pair.a >= body_count || pair.b >= body_count) {
                log::error(physics,
                           "Previous manifold pair out of range during carry-forward (a={} b={} count={})",
                           pair.a, pair.b, body_count);
                std::terminate();
            }
#endif
            if (awake_dynamic_mask[pair.a] == 0u && awake_dynamic_mask[pair.b] == 0u) {
                continue;
            }
            ++stats.carried_pair_count;
            if (!overlaps(bounds[pair.a], bounds[pair.b])) {
                continue;
            }
            candidate_pairs_.push_back(pair);
            ++stats.validated_pair_count;
        }
        return stats;
    }

    std::vector<BodyPair> candidate_pairs_{};
    usize candidate_pair_reserve_hint_{0};

    std::vector<BroadPhaseWorker> workers_{};
    std::vector<std::thread> threads_{};
    std::vector<usize> pair_offsets_{};
    std::mutex mutex_{};
    std::condition_variable cv_{};
    std::condition_variable done_cv_{};
    BroadPhaseJob current_job_{};
    u64 job_id_{0};
    std::atomic<u32> jobs_remaining_{0};
    bool stop_{false};
    u32 worker_count_{0};
    u32 worker_reserve_count_{0};
    usize worker_pair_reserve_hint_{0};
#if defined(JAVELIN_BROAD_PHASE_VALIDATE)
    std::vector<BodyPair> expected_pairs_{};
#endif
};

} // namespace javelin

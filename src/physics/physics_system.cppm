module;

#include <tracy/Tracy.hpp>

export module javelin.physics.physics_system;

import std;
import javelin.core.logging;
import javelin.core.time;
import javelin.core.types;
import javelin.physics.aabb_debug;
import javelin.physics.contact_debug;
import javelin.physics.tick_pipeline;
import javelin.scene;

export namespace javelin {

// Fixed-timestep physics driver.
//
// PhysicsSystem owns:
//   - the dedicated physics thread + its fixed-rate ticker,
//   - the simulation control state machine (pause / manual step budget / reset),
//   - the atomic configuration knobs callers tweak from other threads,
//   - and a TickPipeline that does the actual per-step work.
//
// It owns no per-stage state itself.  All buffers, BVHs, manifolds, and
// debug channels live inside TickPipeline; PhysicsSystem only forwards
// the small public surface area (debug toggles, snapshots) through to it.
struct PhysicsSystem final {
    void init(Scene &scene) noexcept {
        scene_ = &scene;
        pipeline_.init(scene);
    }

    void set_gravity(const f32 gravity) noexcept { gravity_.store(gravity, std::memory_order_relaxed); }
    void set_linear_damping(const f32 damping) noexcept {
        linear_damping_.store(std::max(damping, 0.0f), std::memory_order_relaxed);
    }
    void set_angular_damping(const f32 damping) noexcept {
        angular_damping_.store(std::max(damping, 0.0f), std::memory_order_relaxed);
    }
    void set_contact_debug_enabled(const bool enabled) noexcept { pipeline_.set_contact_debug_enabled(enabled); }
    void set_aabb_debug_enabled(const bool enabled) noexcept { pipeline_.set_aabb_debug_enabled(enabled); }

    // Thread-safe control request consumed by the physics thread.
    // Reset is serviced even while simulation is paused.
    void request_reset() noexcept {
        reset_requested_.store(true, std::memory_order_release);
        simulation_control_cv_.notify_one();
    }

    // Pauses/resumes continuous fixed-rate ticking.
    // Resuming clears queued manual-step budget.
    void set_simulation_paused(const bool paused) noexcept {
        const bool previous = simulation_paused_.exchange(paused, std::memory_order_acq_rel);
        if (!paused) {
            pending_step_budget_.store(0u, std::memory_order_release);
        }
        if (previous != paused || paused) {
            simulation_control_cv_.notify_one();
        }
    }

    // Queues manual fixed ticks while paused.  No-op when simulation is running.
    void request_simulation_steps(const u32 count = 1u) noexcept {
        if (count == 0u || !simulation_paused_.load(std::memory_order_acquire)) {
            return;
        }
        u32 pending = pending_step_budget_.load(std::memory_order_relaxed);
        for (;;) {
            const u32 remaining_capacity = std::numeric_limits<u32>::max() - pending;
            const u32 next = pending + std::min(count, remaining_capacity);
            if (pending_step_budget_.compare_exchange_weak(pending, next, std::memory_order_acq_rel,
                                                          std::memory_order_relaxed)) {
                break;
            }
        }
        simulation_control_cv_.notify_one();
    }

    [[nodiscard]] f32 gravity() const noexcept { return gravity_.load(std::memory_order_relaxed); }
    [[nodiscard]] f32 linear_damping() const noexcept { return linear_damping_.load(std::memory_order_relaxed); }
    [[nodiscard]] f32 angular_damping() const noexcept { return angular_damping_.load(std::memory_order_relaxed); }
    [[nodiscard]] bool contact_debug_enabled() const noexcept { return pipeline_.contact_debug_enabled(); }
    [[nodiscard]] ContactDebugSnapshot contact_debug_snapshot() const noexcept {
        return pipeline_.contact_debug_snapshot();
    }
    [[nodiscard]] bool aabb_debug_enabled() const noexcept { return pipeline_.aabb_debug_enabled(); }
    [[nodiscard]] AabbDebugSnapshot aabb_debug_snapshot() const noexcept { return pipeline_.aabb_debug_snapshot(); }
    [[nodiscard]] f32 last_tick_dt_ms() const noexcept { return last_tick_dt_ms_.load(std::memory_order_relaxed); }
    [[nodiscard]] bool simulation_paused() const noexcept {
        return simulation_paused_.load(std::memory_order_acquire);
    }
    [[nodiscard]] u32 pending_simulation_steps() const noexcept {
        return pending_step_budget_.load(std::memory_order_acquire);
    }
    [[nodiscard]] u64 completed_simulation_steps() const noexcept {
        return completed_sim_step_count_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] static constexpr u32 fixed_step_hz() noexcept { return 60u; }
    [[nodiscard]] static constexpr f32 fixed_step_dt_seconds() noexcept { return kFixedStepDtSeconds; }

    [[nodiscard]] bool step_fixed() noexcept {
        if (thread_.joinable()) {
            log::warn(physics, "step_fixed ignored while threaded simulation is running");
            return false;
        }
        last_tick_dt_ms_.store(static_cast<f32>(kFixedStepDtMilliseconds), std::memory_order_relaxed);
        if (!run_one_tick_()) {
            return false;
        }
        completed_sim_step_count_.fetch_add(1u, std::memory_order_relaxed);
        return true;
    }

    void start() {
        if (thread_.joinable()) {
            log::warn(physics, "Start ignored (already running)");
            return;
        }
        if (scene_ == nullptr) {
            log::warn(physics, "Starting without scene bound");
        }

        log::info(physics, "Starting physics system");
        log::info(physics, "Params gravity={} linear_damping={} angular_damping={}", gravity(), linear_damping(),
                  angular_damping());
        thread_ = std::jthread([this](const std::stop_token &stop_token) {
            tracy::SetThreadName("Physics");

            constexpr auto fixed_step_interval =
                std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<f64>(kFixedStepDtSeconds));
            FixedRateTicker ticker{fixed_step_interval};
            bool ticker_needs_resync = false;

            while (!stop_token.stop_requested()) {
                double tick_dt_ms = 0.0;
                if (simulation_paused_.load(std::memory_order_acquire)) {
                    // Paused mode: service resets and consume explicit step budget only.
                    ticker_needs_resync = true;
                    static_cast<void>(apply_pending_reset_());

                    if (consume_pending_step_()) {
                        tick_dt_ms = kFixedStepDtMilliseconds;
                    } else {
                        if (!wait_for_simulation_control_event_(stop_token)) {
                            break;
                        }
                        continue;
                    }
                } else {
                    // Running mode: resume phase-locked ticker after any pause interval.
                    if (ticker_needs_resync) {
                        ticker = FixedRateTicker{fixed_step_interval};
                        ticker_needs_resync = false;
                    }
                    const auto timing = ticker.wait_next(stop_token);
                    if (stop_token.stop_requested()) {
                        break;
                    }
                    tick_dt_ms = timing.interval_ms;
                }

                last_tick_dt_ms_.store(static_cast<f32>(tick_dt_ms), std::memory_order_relaxed);
                TracyPlot("physics_dt_ms", tick_dt_ms);

                {
                    ZoneScopedN("Physics tick");
                    if (run_one_tick_()) {
                        completed_sim_step_count_.fetch_add(1u, std::memory_order_relaxed);
                    }
                }
                const bool paused_for_plot = simulation_paused_.load(std::memory_order_relaxed);
                const u32 pending_steps_for_plot = pending_step_budget_.load(std::memory_order_relaxed);
                const u64 completed_steps_for_plot = completed_sim_step_count_.load(std::memory_order_relaxed);
                TracyPlot("physics_paused", paused_for_plot ? static_cast<i64>(1) : static_cast<i64>(0));
                TracyPlot("physics_pending_steps", static_cast<i64>(pending_steps_for_plot));
                TracyPlot("physics_completed_steps", tracy_counter_i64_(completed_steps_for_plot));

                FrameMarkNamed("Physics");
            }
        });
    }

    void stop() noexcept {
        if (thread_.joinable()) {
            log::info(physics, "Stopping physics system");
            thread_.request_stop();
            simulation_control_cv_.notify_all();
            thread_.join();
        }
        pipeline_.shutdown();
    }

  private:
    static constexpr f32 kFixedStepDtSeconds = 1.0f / 60.0f;
    static constexpr double kFixedStepDtMilliseconds = 1000.0 / 60.0;

    [[nodiscard]] static i64 tracy_counter_i64_(const u64 value) noexcept {
        return static_cast<i64>(std::min<u64>(value, static_cast<u64>(std::numeric_limits<i64>::max())));
    }

    // Sleeps only while paused and idle (no pending steps / no reset request).
    // Running mode never takes this path.
    [[nodiscard]] bool wait_for_simulation_control_event_(const std::stop_token &stop_token) {
        std::unique_lock lock(simulation_control_mutex_);
        simulation_control_cv_.wait(lock, [&] {
            return stop_token.stop_requested() || !simulation_paused_.load(std::memory_order_acquire) ||
                   pending_step_budget_.load(std::memory_order_acquire) > 0u ||
                   reset_requested_.load(std::memory_order_acquire);
        });
        return !stop_token.stop_requested();
    }

    [[nodiscard]] bool consume_pending_step_() noexcept {
        u32 pending = pending_step_budget_.load(std::memory_order_relaxed);
        while (pending > 0u) {
            if (pending_step_budget_.compare_exchange_weak(pending, pending - 1u, std::memory_order_acq_rel,
                                                           std::memory_order_relaxed)) {
                return true;
            }
        }
        return false;
    }

    // Services a pending reset request at a thread-safe tick boundary.
    [[nodiscard]] bool apply_pending_reset_() noexcept {
        if (!reset_requested_.exchange(false, std::memory_order_acq_rel)) {
            return false;
        }
        if (scene_ != nullptr) {
            scene_->reset_simulation();
            pipeline_.handle_reset(completed_sim_step_count_.load(std::memory_order_relaxed));
        }
        return true;
    }

    [[nodiscard]] bool run_one_tick_() {
        if (scene_ == nullptr) {
            return false;
        }
        static_cast<void>(apply_pending_reset_());

        const TickParams params{
            .dt = kFixedStepDtSeconds,
            .gravity = gravity_.load(std::memory_order_relaxed),
            .linear_damping = linear_damping_.load(std::memory_order_relaxed),
            .angular_damping = angular_damping_.load(std::memory_order_relaxed),
        };
        const u64 next_step_id = completed_sim_step_count_.load(std::memory_order_relaxed) + 1u;
        return pipeline_.step(*scene_, params, next_step_id);
    }

    Scene *scene_{nullptr};
    TickPipeline pipeline_{};
    std::jthread thread_{};
    std::atomic<f32> gravity_{-9.8f};
    std::atomic<f32> linear_damping_{0.1f};
    std::atomic<f32> angular_damping_{0.4f};
    std::atomic<f32> last_tick_dt_ms_{0.0f};
    std::atomic<bool> reset_requested_{false};
    // Simulation control invariants:
    // - paused gate controls continuous ticking.
    // - pending_step_budget is decremented only by the physics thread.
    // - request_simulation_steps() only increments budget when paused.
    // - completed_sim_step_count is a monotonic diagnostic counter.
    std::atomic<bool> simulation_paused_{false};
    std::atomic<u32> pending_step_budget_{0u};
    std::atomic<u64> completed_sim_step_count_{0u};
    std::mutex simulation_control_mutex_{};
    std::condition_variable simulation_control_cv_{};
};

} // namespace javelin

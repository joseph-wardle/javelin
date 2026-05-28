module;

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/gl.h>

#include <tracy/Tracy.hpp>
#include <tracy/TracyOpenGL.hpp>

export module javelin.render.render_system;

import std;

import javelin.core.types;
import javelin.core.logging;
import javelin.core.time;
import javelin.render.camera_controller;
import javelin.render.camera_manager;
import javelin.render.pipeline;
import javelin.render.render_context;
import javelin.render.render_device;
import javelin.render.render_targets;
import javelin.render.passes.aabb_debug_pass;
import javelin.render.passes.contact_debug_pass;
import javelin.render.passes.constraint_debug_pass;
import javelin.render.passes.velocity_debug_pass;
import javelin.render.passes.display_pass;
import javelin.render.passes.geometry_pass;
import javelin.render.passes.sleep_debug_pass;
import javelin.render.passes.world_grid_pass;
import javelin.render.types;
import javelin.render.ui_panels;
import javelin.physics.aabb_debug;
import javelin.physics.contact_debug;
import javelin.physics.physics_system;
import javelin.scene;
import javelin.scene.pose_channel;
import javelin.platform;
import javelin.platform.input;

export namespace javelin {

struct RenderGpuConfig final {
    bool ui_enabled{true};
    bool fixed_world_camera{false};
    f32 recording_orbit_start_seconds{0.0f};
    bool draw_sleep_state{false};
};

// Owns the GPU pipeline and orchestrates the per-frame sequence:
//   resize → UI build → camera update → cursor mode → pipeline draw → present.
// Delegates camera mechanics to CameraManager and ImGui mechanics to UiPanels;
// everything else here is GL/GLFW plumbing.
struct RenderSystem final {
    void init_cpu(const Scene &scene, PhysicsSystem &physics) noexcept {
        scene_ = &scene;
        physics_ = &physics;

        log::info(render, "Initializing CPU resources for render system");
        camera_manager_.init_default_pose();
    }

    void init_gpu(const WindowHandle window, const RenderGpuConfig &config = {}) {
        ZoneScopedN("Render init GPU");
        log::info(render, "Initializing GPU resources for render system");
        window_ = window;
        ui_enabled_ = config.ui_enabled;
        glfwMakeContextCurrent(window_.native);

        if (gladLoadGL(glfwGetProcAddress) == 0) {
            log::critical(render, "OpenGL loader initialization failed");
        }

        TracyGpuContext;

        const char *vendor = reinterpret_cast<const char *>(glGetString(GL_VENDOR));
        const char *renderer = reinterpret_cast<const char *>(glGetString(GL_RENDERER));
        const char *version = reinterpret_cast<const char *>(glGetString(GL_VERSION));
        if (vendor && renderer && version) {
            log::info(render, "GL vendor={}", vendor);
            log::info(render, "GL device={}", renderer);
            log::info(render, "GL version={}", version);
        } else {
            log::warn(render, "GL info unavailable");
        }

        // 0 uncapped, 1 vsync
        glfwSwapInterval(0);

        targets_.init();
        pipeline_.init(device_);

        int w = 0, h = 0;
        glfwGetFramebufferSize(window_.native, &w, &h);
        extent_ = Extent2D{w, h};
        targets_.resize(extent_);
        pipeline_.resize(device_, extent_);

        if (scene_ != nullptr) {
            camera_manager_.configure_for_scene(*scene_, extent_.width, extent_.height,
                                                config.fixed_world_camera,
                                                config.recording_orbit_start_seconds);
        }

        if (ui_enabled_) {
            ui_panels_.init(window_);
        }
        if (config.draw_sleep_state) {
            ui_panels_.set_draw_sleep_state(true);
        }
        gpu_ready_ = true;

        log::info(render, "GPU initialized");
    }

    void render_frame(const f64 dt, InputState &input) noexcept {
        ZoneScopedN("Render frame");

        if (dt > 0.0) {
            TracyPlot("render_dt_ms", dt * 1000.0);
        }

        if (!gpu_ready_ || scene_ == nullptr)
            return;

        resize_if_needed_();
        if (ui_enabled_) {
            ui_panels_.build_frame(scene_, physics_, static_cast<f32>(dt * 1000.0), input);
        } else {
            input.end_frame(false, false);
        }

        const CursorMode cursor_mode =
            camera_manager_.update_interactive(input.frame(), static_cast<f32>(dt));
        glfwSetInputMode(window_.native, GLFW_CURSOR,
                         cursor_mode == CursorMode::captured ? GLFW_CURSOR_DISABLED
                                                             : GLFW_CURSOR_NORMAL);

        const PoseSnapshot poses = scene_->pose_snapshot();
        draw_frame_(poses, compute_pose_alpha_(poses), true, ui_enabled_);
    }

    void render_offline_frame(const f64 dt, const f32 pose_alpha = 1.0f) noexcept {
        ZoneScopedN("Render offline frame");

        if (dt > 0.0) {
            TracyPlot("render_dt_ms", dt * 1000.0);
        }

        if (!gpu_ready_ || scene_ == nullptr)
            return;

        resize_if_needed_();
        static_cast<void>(camera_manager_.update_offline(static_cast<f32>(dt)));
        glfwSetInputMode(window_.native, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

        const PoseSnapshot poses = scene_->pose_snapshot();
        draw_frame_(poses, pose_alpha, false, false);
    }

    [[nodiscard]] Extent2D extent() const noexcept { return extent_; }

    [[nodiscard]] bool readback_rgba8(std::span<u8> out_pixels) const noexcept {
        if (!gpu_ready_ || !extent_.is_valid()) {
            return false;
        }

        const usize expected_size = static_cast<usize>(extent_.width) *
                                    static_cast<usize>(extent_.height) *
                                    static_cast<usize>(4u);
        if (out_pixels.size() < expected_size) {
            return false;
        }

        glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
        glReadBuffer(GL_BACK);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, extent_.width, extent_.height, GL_RGBA, GL_UNSIGNED_BYTE,
                     out_pixels.data());
        return true;
    }

    void shutdown() noexcept {
        ZoneScopedN("Render shutdown");
        log::info(render, "Shutting down render system");

        if (gpu_ready_) {
            pipeline_.shutdown(device_);
            targets_.shutdown();
            if (ui_enabled_) {
                ui_panels_.shutdown();
            }

            glfwMakeContextCurrent(nullptr);
            gpu_ready_ = false;
        }

        scene_ = nullptr;
        window_ = {};
    }

private:
    void resize_if_needed_() noexcept {
        int w = 0;
        int h = 0;
        glfwGetFramebufferSize(window_.native, &w, &h);
        const Extent2D new_extent{w, h};
        if (new_extent.width == extent_.width && new_extent.height == extent_.height) {
            return;
        }

        ZoneScopedN("Render resize");
        extent_ = new_extent;
        targets_.resize(extent_);
        pipeline_.resize(device_, extent_);
    }

    void draw_frame_(const PoseSnapshot &poses, const f32 pose_alpha,
                     const bool present_enabled, const bool draw_ui) noexcept {
        const FrameCamera frame_camera =
            camera_manager_.frame_camera(extent_.width, extent_.height);

        const DebugToggles &debug = ui_panels_.debug();
        if (physics_ != nullptr) {
            physics_->set_contact_debug_enabled(debug.draw_contacts);
            physics_->set_aabb_debug_enabled(debug.draw_aabbs);
        }
        const ContactDebugSnapshot contacts =
            (physics_ != nullptr) ? physics_->contact_debug_snapshot() : ContactDebugSnapshot{};
        const AabbDebugSnapshot aabbs =
            (physics_ != nullptr) ? physics_->aabb_debug_snapshot() : AabbDebugSnapshot{};

        RenderContext ctx{
            .extent = extent_,
            .camera = frame_camera,
            .view = scene_->bodies(),
            .poses = poses,
            .contacts = contacts,
            .aabbs = aabbs,
            .pose_alpha = pose_alpha,
            .targets = targets_,
            .debug = debug,
        };

        {
            ZoneScopedN("Render passes");
            pipeline_.execute(ctx);
        }

        if (present_enabled) {
            ZoneScopedN("Present");
            TracyGpuZone("Present");
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            glViewport(0, 0, extent_.width, extent_.height);
            if (draw_ui) {
                ui_panels_.submit_draw_data();
            }
            glfwSwapBuffers(window_.native);
        }

        TracyGpuCollect;
    }

    // Pipeline order: opaque geometry → surface overlays → x-ray overlays →
    // display. SleepDebugPass uses GL_LEQUAL depth (surface overlay) and must
    // run before the x-ray passes (ContactDebugPass, AabbDebugPass,
    // ConstraintDebugPass) which disable depth testing.
    using Pipeline =
        RenderPipeline<GeometryPass, WorldGridPass, SleepDebugPass, ContactDebugPass,
                       AabbDebugPass, ConstraintDebugPass, VelocityDebugPass, DisplayPass>;

    const Scene *scene_ = nullptr;
    PhysicsSystem *physics_ = nullptr;
    WindowHandle window_{};

    RenderDevice device_{};
    RenderTargets targets_{};
    Pipeline pipeline_{};
    CameraManager camera_manager_{};
    UiPanels ui_panels_{};
    Extent2D extent_{};

    bool ui_enabled_ = true;
    bool gpu_ready_ = false;

    // Cap interpolation span so a single step after a long pause does not blend
    // slowly. Two fixed physics ticks keeps motion smooth without long wall-clock
    // lag.
    static constexpr u64 kMaxPoseInterpolationSpanNs = 2u * (1'000'000'000ull / 60u);

    [[nodiscard]] static u64 now_ns_() noexcept {
        const auto now = SteadyClock::now().time_since_epoch();
        return static_cast<u64>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
    }

    [[nodiscard]] static f32 compute_pose_alpha_(const PoseSnapshot &poses) noexcept {
        if (poses.count == 0 || poses.prev_time_ns == 0 || poses.curr_time_ns == 0 ||
            poses.curr_time_ns <= poses.prev_time_ns) {
            return 1.0f;
        }

        const u64 raw_span = poses.curr_time_ns - poses.prev_time_ns;
        const u64 interpolation_span = std::min(raw_span, kMaxPoseInterpolationSpanNs);
        if (interpolation_span == 0u) {
            return 1.0f;
        }

        const u64 interpolation_start_time = poses.curr_time_ns - interpolation_span;
        const u64 now = now_ns_();
        const u64 render_time = (now > interpolation_span) ? (now - interpolation_span) : 0u;
        const u64 sample_time =
            std::clamp(render_time, interpolation_start_time, poses.curr_time_ns);
        const f64 alpha = static_cast<f64>(sample_time - interpolation_start_time) /
                          static_cast<f64>(interpolation_span);
        return static_cast<f32>(alpha);
    }
};

} // namespace javelin

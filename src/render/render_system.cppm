module;

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/gl.h>

#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <imgui.h>

#include <tracy/Tracy.hpp>
#include <tracy/TracyOpenGL.hpp>

export module javelin.render.render_system;

import std;

import javelin.core.types;
import javelin.core.logging;
import javelin.core.time;
import javelin.render.pipeline;
import javelin.render.render_context;
import javelin.render.render_device;
import javelin.render.render_targets;
import javelin.render.passes.display_pass;
import javelin.render.passes.geometry_pass;
import javelin.render.passes.world_grid_pass;
import javelin.render.fly_camera;
import javelin.render.types;
import javelin.physics.physics_system;
import javelin.scene;
import javelin.scene.camera;
import javelin.scene.pose_channel;
import javelin.platform.input;
import javelin.platform.window;

export namespace javelin {

struct RenderSystem final {
    void init_cpu(const Scene &scene, PhysicsSystem &physics) noexcept {
        scene_ = &scene;
        physics_ = &physics;
        // TODO: build CPU-side render runtime:
        // - mesh/material registries
        // - draw lists (static)

        log::info(render, "Initializing CPU resources for render system");
        camera_.position = {0.0f, 5.0f, 5.0f};
        camera_.yaw_radians = 0.0f;
        camera_.pitch_radians = -0.35f;
    }

    void init_gpu(const WindowHandle window) {
        ZoneScopedN("Render init GPU");
        log::info(render, "Initializing GPU resources for render system");
        window_ = window;

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

        init_imgui_();
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

        int w = 0, h = 0;
        glfwGetFramebufferSize(window_.native, &w, &h);
        const Extent2D new_extent{w, h};
        if (new_extent.width != extent_.width || new_extent.height != extent_.height) {
            ZoneScopedN("Render resize");
            extent_ = new_extent;
            targets_.resize(extent_);
            pipeline_.resize(device_, extent_);
        }

        // --- ImGui frame ---
        {
            ZoneScopedN("ImGui build");
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            ImGui::Begin("Javelin");
            const f32 render_dt_ms = static_cast<f32>(dt * 1000.0);
            if (render_dt_ms > 0.0f) {
                push_render_dt_sample_(render_dt_ms);
            }
            ImGui::Checkbox("Grid", &debug_.draw_grid);
            ImGui::Checkbox("Color Transform", &debug_.apply_color_transform);
            if (physics_ != nullptr) {
                // TEMP: test-scene physics controls.
                ImGui::Separator();
                ImGui::TextUnformatted("Physics");

                f32 gravity = physics_->gravity();
                if (ImGui::DragFloat("Gravity", &gravity, 0.1f, -50.0f, 0.0f)) {
                    physics_->set_gravity(gravity);
                }
                f32 restitution = physics_->restitution();
                if (ImGui::DragFloat("Restitution", &restitution, 0.01f, 0.0f, 1.0f)) {
                    physics_->set_restitution(restitution);
                }

                f32 friction = physics_->friction();
                if (ImGui::DragFloat("Friction", &friction, 0.01f, 0.0f, 1.0f)) {
                    physics_->set_friction(friction);
                }

                f32 angular_damping = physics_->angular_damping();
                if (ImGui::DragFloat("Angular Damping", &angular_damping, 0.01f, 0.0f, 5.0f)) {
                    physics_->set_angular_damping(angular_damping);
                }

                if (ImGui::Button("Reset Scene")) {
                    physics_->request_reset();
                }

                ImGui::Separator();
                ImGui::Text("Frame dt: %.3f ms", render_dt_ms);
                const int render_history_count =
                    render_dt_history_full_ ? static_cast<int>(kRenderDtHistory) : static_cast<int>(render_dt_cursor_);
                if (render_history_count > 0) {
                    const int offset = render_dt_history_full_ ? static_cast<int>(render_dt_cursor_) : 0;
                    f32 min_v = 0.0f;
                    f32 max_v = 0.0f;
                    double sum = 0.0;
                    for (int i = 0; i < render_history_count; ++i) {
                        const usize idx = (static_cast<usize>(offset) + static_cast<usize>(i)) % kRenderDtHistory;
                        const f32 v = render_dt_history_[idx];
                        if (i == 0) {
                            min_v = v;
                            max_v = v;
                        } else {
                            min_v = std::min(min_v, v);
                            max_v = std::max(max_v, v);
                        }
                        sum += static_cast<double>(v);
                    }
                    const f32 avg = static_cast<f32>(sum / static_cast<double>(render_history_count));
                    f32 max_dev = std::max(max_v - avg, avg - min_v);
                    if (max_dev < 0.25f) {
                        max_dev = 0.25f;
                    }
                    max_dev *= 1.1f;
                    const f32 plot_min = avg - max_dev;
                    const f32 plot_max = avg + max_dev;
                    ImGui::PlotLines("Frame dt (ms)", render_dt_history_.data(), render_history_count, offset, nullptr,
                                     plot_min, plot_max, ImVec2(0, 80));
                }

                const f32 physics_dt_ms = physics_->last_tick_dt_ms();
                if (physics_dt_ms > 0.0f) {
                    push_physics_dt_sample_(physics_dt_ms);
                }
                ImGui::Text("Physics dt: %.3f ms", physics_dt_ms);
                const int history_count = physics_dt_history_full_
                                              ? static_cast<int>(kPhysicsDtHistory)
                                              : static_cast<int>(physics_dt_cursor_);
                if (history_count > 0) {
                    const int offset = physics_dt_history_full_ ? static_cast<int>(physics_dt_cursor_) : 0;
                    f32 min_v = 0.0f;
                    f32 max_v = 0.0f;
                    double sum = 0.0;
                    for (int i = 0; i < history_count; ++i) {
                        const usize idx = (static_cast<usize>(offset) + static_cast<usize>(i)) % kPhysicsDtHistory;
                        const f32 v = physics_dt_history_[idx];
                        if (i == 0) {
                            min_v = v;
                            max_v = v;
                        } else {
                            min_v = std::min(min_v, v);
                            max_v = std::max(max_v, v);
                        }
                        sum += static_cast<double>(v);
                    }
                    const f32 avg = static_cast<f32>(sum / static_cast<double>(history_count));
                    f32 max_dev = std::max(max_v - avg, avg - min_v);
                    if (max_dev < 0.25f) {
                        max_dev = 0.25f;
                    }
                    max_dev *= 1.1f;
                    const f32 plot_min = avg - max_dev;
                    const f32 plot_max = avg + max_dev;
                    ImGui::PlotLines("Physics dt (ms)", physics_dt_history_.data(), history_count, offset, nullptr,
                                     plot_min, plot_max, ImVec2(0, 80));
                }
            }
            ImGui::End();

            const ImGuiIO &io = ImGui::GetIO();
            input.end_frame(io.WantCaptureMouse, io.WantCaptureKeyboard);
            ImGui::Render();
        }

        const CursorMode cursor_mode = fly_camera_.update(camera_, input.frame(), static_cast<f32>(dt));
        glfwSetInputMode(window_.native, GLFW_CURSOR,
                         cursor_mode == CursorMode::captured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);

        const f32 aspect =
            (extent_.height > 0) ? static_cast<f32>(extent_.width) / static_cast<f32>(extent_.height) : 1.0f;
        const auto view = camera_view(camera_);
        const auto proj = camera_proj(camera_.lens, aspect);
        const FrameCamera frame_camera{.view = view, .proj = proj, .view_proj = proj * view};

        const PoseSnapshot poses = scene_->pose_snapshot();
        const f32 pose_alpha = compute_pose_alpha_(poses);

        RenderContext ctx{
            .extent = extent_,
            .camera = frame_camera,
            .view = scene_->render_view(),
            .poses = poses,
            .pose_alpha = pose_alpha,
            .targets = targets_,
            .debug = debug_,
        };

        // --- Draw ---
        {
            ZoneScopedN("Render passes");
            pipeline_.execute(ctx);
        }
        {
            ZoneScopedN("Present");
            TracyGpuZone("Present");
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            glViewport(0, 0, extent_.width, extent_.height);
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            glfwSwapBuffers(window_.native);
        }

        TracyGpuCollect;
    }

    void shutdown() noexcept {
        ZoneScopedN("Render shutdown");
        log::info(render, "Shutting down render system");

        if (gpu_ready_) {
            pipeline_.shutdown(device_);
            targets_.shutdown();
            shutdown_imgui_();

            // Optional: detach context
            glfwMakeContextCurrent(nullptr);
            gpu_ready_ = false;
        }

        scene_ = nullptr;
        window_ = {};
    }

  private:
    void init_imgui_() const {
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGui::StyleColorsDark();

        ImGui_ImplGlfw_InitForOpenGL(window_.native, /*install_callbacks=*/true);
        ImGui_ImplOpenGL3_Init("#version 460");
    }

    static void shutdown_imgui_() noexcept {
        if (ImGui::GetCurrentContext() == nullptr)
            return;
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }

    void push_render_dt_sample_(const f32 dt_ms) noexcept {
        render_dt_sample_tick_ = (render_dt_sample_tick_ + 1u) % kDtSampleStride;
        if (render_dt_sample_tick_ != 0u) {
            return;
        }
        render_dt_history_[render_dt_cursor_] = dt_ms;
        render_dt_cursor_ = (render_dt_cursor_ + 1u) % kRenderDtHistory;
        if (render_dt_cursor_ == 0) {
            render_dt_history_full_ = true;
        }
    }

    void push_physics_dt_sample_(const f32 dt_ms) noexcept {
        physics_dt_sample_tick_ = (physics_dt_sample_tick_ + 1u) % kDtSampleStride;
        if (physics_dt_sample_tick_ != 0u) {
            return;
        }
        physics_dt_history_[physics_dt_cursor_] = dt_ms;
        physics_dt_cursor_ = (physics_dt_cursor_ + 1u) % kPhysicsDtHistory;
        if (physics_dt_cursor_ == 0) {
            physics_dt_history_full_ = true;
        }
    }

  private:
    using Pipeline = RenderPipeline<GeometryPass, WorldGridPass, DisplayPass>;

    const Scene *scene_ = nullptr;
    PhysicsSystem *physics_ = nullptr;
    WindowHandle window_{};

    RenderDevice device_{};
    RenderTargets targets_{};
    Pipeline pipeline_{};
    DebugToggles debug_{};
    CameraState camera_{};
    FlyCameraController fly_camera_{};
    Extent2D extent_{};

    static constexpr usize kDtHistory = 240;
    static constexpr u32 kDtSampleStride = 4;
    static constexpr usize kRenderDtHistory = kDtHistory;
    std::array<f32, kRenderDtHistory> render_dt_history_{};
    usize render_dt_cursor_ = 0;
    bool render_dt_history_full_ = false;
    u32 render_dt_sample_tick_ = 0;

    static constexpr usize kPhysicsDtHistory = kDtHistory;
    std::array<f32, kPhysicsDtHistory> physics_dt_history_{};
    usize physics_dt_cursor_ = 0;
    bool physics_dt_history_full_ = false;
    u32 physics_dt_sample_tick_ = 0;

    bool gpu_ready_ = false;

    [[nodiscard]] static u64 now_ns_() noexcept {
        const auto now = SteadyClock::now().time_since_epoch();
        return static_cast<u64>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
    }

    [[nodiscard]] static f32 compute_pose_alpha_(const PoseSnapshot &poses) noexcept {
        if (poses.count == 0 || poses.prev_time_ns == 0 || poses.curr_time_ns == 0 ||
            poses.curr_time_ns <= poses.prev_time_ns) {
            return 1.0f;
        }

        const u64 span = poses.curr_time_ns - poses.prev_time_ns;
        const u64 now = now_ns_();
        const u64 render_time = (now > span) ? (now - span) : 0;

        u64 sample_time = render_time;
        if (sample_time < poses.prev_time_ns) {
            sample_time = poses.prev_time_ns;
        } else if (sample_time > poses.curr_time_ns) {
            sample_time = poses.curr_time_ns;
        }

        const f64 alpha = static_cast<f64>(sample_time - poses.prev_time_ns) / static_cast<f64>(span);
        return static_cast<f32>(alpha);
    }
};

} // namespace javelin

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

import javelin.core.types;
import javelin.core.logging;
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
            ImGui::Text("dt: %.3f ms", dt * 1000.0);
            ImGui::Checkbox("Grid", &debug_.draw_grid);
            ImGui::Checkbox("Color Transform", &debug_.apply_color_transform);
            if (physics_ != nullptr) {
                ImGui::Separator();
                ImGui::TextUnformatted("Physics");

                f32 gravity = physics_->gravity();
                if (ImGui::DragFloat("Gravity", &gravity, 0.1f, -50.0f, 0.0f)) {
                    physics_->set_gravity(gravity);
                }

                f32 reset_y = physics_->reset_y();
                if (ImGui::DragFloat("Reset Y", &reset_y, 0.1f, -50.0f, 50.0f)) {
                    physics_->set_reset_y(reset_y);
                }

                f32 spawn_y = physics_->spawn_y();
                if (ImGui::DragFloat("Spawn Y", &spawn_y, 0.1f, -50.0f, 50.0f)) {
                    physics_->set_spawn_y(spawn_y);
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

        RenderContext ctx{
            .extent = extent_,
            .camera = frame_camera,
            .view = scene_->render_view(),
            .poses = scene_->pose_snapshot(),
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

  private:
    using Pipeline = RenderPipeline<WorldGridPass, GeometryPass, DisplayPass>;

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

    bool gpu_ready_ = false;
};

} // namespace javelin

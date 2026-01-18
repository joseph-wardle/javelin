module;

#include <tracy/Tracy.hpp>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/gl.h>

#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <imgui.h>

export module javelin.render.render_system;

import javelin.core.types;
import javelin.core.logging;
import javelin.render.pipeline;
import javelin.render.render_context;
import javelin.render.render_device;
import javelin.render.render_targets;
import javelin.render.passes.post_pass;
import javelin.render.passes.world_grid_pass;
import javelin.render.fly_camera;
import javelin.render.types;
import javelin.scene;
import javelin.scene.camera;
import javelin.platform.input;
import javelin.platform.window;

export namespace javelin {

struct RenderSystem final {
    void init_cpu(const Scene &scene) noexcept {
        scene_ = &scene;
        // TODO: build CPU-side render runtime:
        // - mesh/material registries
        // - draw lists (static)

        camera_.position = {0.0f, 2.0f, 5.0f};
        camera_.yaw_radians = 0.0f;
        camera_.pitch_radians = -0.35f;
    }

    void init_gpu(const WindowHandle window) {
        window_ = window;

        glfwMakeContextCurrent(window_.native);

        if (gladLoadGL(glfwGetProcAddress) == 0) {
            log::critical("Failed to initialize OpenGL loader!");
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

        log::info("RenderSystem GPU initialized");
    }

    void render_frame(const double dt, InputState &input) noexcept {
        ZoneScopedN("Render frame");

        if (dt > 0.0) {
            TracyPlot("render_fps", 1.0 / dt);
            TracyPlot("render_dt_ms", dt * 1000.0);
        }

        if (!gpu_ready_ || scene_ == nullptr)
            return;

        int w = 0, h = 0;
        glfwGetFramebufferSize(window_.native, &w, &h);
        const Extent2D new_extent{w, h};
        if (new_extent.width != extent_.width || new_extent.height != extent_.height) {
            extent_ = new_extent;
            targets_.resize(extent_);
            pipeline_.resize(device_, extent_);
        }

        // --- ImGui frame ---
        {
            ZoneScopedN("UI");
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            ImGui::Begin("Javelin");
            ImGui::Text("dt: %.3f ms", dt * 1000.0);
            ImGui::Checkbox("Grid", &debug_.draw_grid);
            ImGui::Checkbox("Debug", &debug_.draw_debug);
            ImGui::Checkbox("Wireframe", &debug_.draw_wireframe);
            ImGui::End();

            // ImGui::ShowDemoWindow();

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
            ZoneScopedN("GL present");
            pipeline_.execute(ctx);
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            glViewport(0, 0, extent_.width, extent_.height);
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            glfwSwapBuffers(window_.native);
        }

        // You can keep this as FrameMarkNamed("Frame") in App for a single canonical marker.
    }

    void shutdown() noexcept {
        ZoneScopedN("Render shutdown");

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
    using Pipeline = RenderPipeline<WorldGridPass, PostPass>;

    const Scene *scene_ = nullptr;
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

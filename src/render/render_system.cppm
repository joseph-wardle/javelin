module;

#include <tracy/Tracy.hpp>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/gl.h>

#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <imgui.h>

export module javelin.render.render_system;

import std;
import javelin.core.logging;
import javelin.scene;
import javelin.platform.window;

export namespace javelin {

struct RenderSystem final {
    void init_cpu(const Scene &scene) noexcept {
        scene_ = &scene;
        // TODO: build CPU-side render runtime:
        // - mesh/material registries
        // - draw lists (static)
    }

    void init_gpu(const WindowHandle window) {
        window_ = window;

        glfwMakeContextCurrent(window_.native);

        if (gladLoadGL(glfwGetProcAddress) == 0) {
            log::critical("Failed to initialize OpenGL loader!");
        }

        // 0 uncapped, 1 vsync
        glfwSwapInterval(0);

        init_imgui_();
        gpu_ready_ = true;

        log::info("RenderSystem GPU initialized");
    }

    void render_frame(const double dt) noexcept {
        ZoneScopedN("Render frame");

        if (dt > 0.0) {
            TracyPlot("render_fps", 1.0 / dt);
            TracyPlot("render_dt_ms", dt * 1000.0);
        }

        if (!gpu_ready_)
            return;

        // --- ImGui frame ---
        {
            ZoneScopedN("UI");
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            ImGui::Begin("Javelin");
            ImGui::Text("dt: %.3f ms", dt * 1000.0);
            ImGui::Text("Render on main thread.");
            ImGui::End();

            // ImGui::ShowDemoWindow();

            ImGui::Render();
        }

        // --- Draw ---
        {
            ZoneScopedN("GL present");

            int w = 0, h = 0;
            glfwGetFramebufferSize(window_.native, &w, &h);
            glViewport(0, 0, w, h);

            glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);

            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            glfwSwapBuffers(window_.native);
        }

        // You can keep this as FrameMarkNamed("Frame") in App for a single canonical marker.
    }

    void shutdown() noexcept {
        ZoneScopedN("Render shutdown");

        if (gpu_ready_) {
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

        ImGui_ImplGlfw_InitForOpenGL(window_.native, /*install_callbacks=*/false);
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
    const Scene *scene_ = nullptr;
    WindowHandle window_{};

    bool gpu_ready_ = false;
};

} // namespace javelin

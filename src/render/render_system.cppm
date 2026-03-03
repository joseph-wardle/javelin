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
import javelin.render.passes.aabb_debug_pass;
import javelin.render.passes.contact_debug_pass;
import javelin.render.passes.constraint_debug_pass;
import javelin.render.passes.velocity_debug_pass;
import javelin.render.passes.display_pass;
import javelin.render.passes.geometry_pass;
import javelin.render.passes.sleep_debug_pass;
import javelin.render.passes.world_grid_pass;
import javelin.render.fly_camera;
import javelin.render.types;
import javelin.physics.aabb_debug;
import javelin.physics.contact_debug;
import javelin.physics.physics_system;
import javelin.scene;
import javelin.scene.camera;
import javelin.scene.pose_channel;
import javelin.platform.input;
import javelin.platform.window;

export namespace javelin {

struct RenderSystem final {
  private:
    struct UiSnapshot final {
        f32 render_dt_ms{};
        DebugToggles debug_toggles{};
        u32 scene_body_count{};
        u32 scene_shape_count{};
        u32 scene_constraint_count{};
        bool has_physics{};
        bool simulation_paused{};
        u32 pending_simulation_steps{};
        u64 completed_simulation_steps{};
        f32 gravity{};
        f32 linear_damping{};
        f32 angular_damping{};
        f32 physics_dt_ms{};
    };

    struct UiCommands final {
        bool has_debug_toggles{};
        DebugToggles debug_toggles{};
        bool has_simulation_paused{};
        bool simulation_paused{};
        u32 requested_simulation_steps{};
        bool has_gravity{};
        f32 gravity{};
        bool has_linear_damping{};
        f32 linear_damping{};
        bool has_angular_damping{};
        f32 angular_damping{};
        bool request_reset{};
    };

    enum struct UiTab : u8 {
        simulation,
        debug,
        performance,
        scene,
    };

  public:
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
        load_ui_state_();
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

            const f32 render_dt_ms = static_cast<f32>(dt * 1000.0);
            if (render_dt_ms > 0.0f) {
                push_render_dt_sample_(render_dt_ms);
            }
            const UiSnapshot ui_snapshot = build_ui_snapshot_(render_dt_ms);
            UiCommands ui_commands{};
            build_ui_(ui_snapshot, ui_commands);
            apply_ui_commands_(ui_commands);

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
        if (physics_ != nullptr) {
            physics_->set_contact_debug_enabled(debug_.draw_contacts);
            physics_->set_aabb_debug_enabled(debug_.draw_aabbs);
        }
        const ContactDebugSnapshot contacts =
            (physics_ != nullptr) ? physics_->contact_debug_snapshot() : ContactDebugSnapshot{};
        const AabbDebugSnapshot aabbs = (physics_ != nullptr) ? physics_->aabb_debug_snapshot() : AabbDebugSnapshot{};
        const f32 pose_alpha = compute_pose_alpha_(poses);

        RenderContext ctx{
            .extent = extent_,
            .camera = frame_camera,
            .view = scene_->render_view(),
            .poses = poses,
            .contacts = contacts,
            .aabbs = aabbs,
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
            save_ui_state_();
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
    static constexpr std::string_view kUiStatePath = "javelin_ui_state.ini";

    [[nodiscard]] static std::string_view tab_to_string_(const UiTab tab) noexcept {
        switch (tab) {
        case UiTab::simulation:
            return "simulation";
        case UiTab::debug:
            return "debug";
        case UiTab::performance:
            return "performance";
        case UiTab::scene:
            return "scene";
        }
        return "simulation";
    }

    [[nodiscard]] static UiTab parse_tab_(const std::string_view value) noexcept {
        if (value == "debug") {
            return UiTab::debug;
        }
        if (value == "performance") {
            return UiTab::performance;
        }
        if (value == "scene") {
            return UiTab::scene;
        }
        return UiTab::simulation;
    }

    [[nodiscard]] static bool parse_f32_(const std::string_view text, f32 &out) noexcept {
        if (text.empty()) {
            return false;
        }

        const std::string owned{text};
        char *end = nullptr;
        const float parsed = std::strtof(owned.c_str(), &end);
        if (end == owned.c_str() || *end != '\0' || !std::isfinite(parsed)) {
            return false;
        }
        out = parsed;
        return true;
    }

    void load_ui_state_() noexcept {
        std::ifstream in{std::string{kUiStatePath}};
        if (!in.is_open()) {
            return;
        }

        bool has_x = false;
        bool has_y = false;
        bool has_w = false;
        bool has_h = false;
        f32 loaded_x = 0.0f;
        f32 loaded_y = 0.0f;
        f32 loaded_w = 0.0f;
        f32 loaded_h = 0.0f;

        std::string line{};
        while (std::getline(in, line)) {
            const usize eq = line.find('=');
            if (eq == std::string::npos) {
                continue;
            }
            const std::string_view key{line.data(), eq};
            const std::string_view value{line.data() + eq + 1u, line.size() - eq - 1u};

            if (key == "selected_tab") {
                selected_tab_ = parse_tab_(value);
                ui_tab_restore_pending_ = true;
                continue;
            }
            if (key == "window_pos_x") {
                has_x = parse_f32_(value, loaded_x);
                continue;
            }
            if (key == "window_pos_y") {
                has_y = parse_f32_(value, loaded_y);
                continue;
            }
            if (key == "window_size_x") {
                has_w = parse_f32_(value, loaded_w);
                continue;
            }
            if (key == "window_size_y") {
                has_h = parse_f32_(value, loaded_h);
                continue;
            }
        }

        if (has_x && has_y && has_w && has_h && loaded_w > 64.0f && loaded_h > 64.0f) {
            ui_window_pos_ = ImVec2{loaded_x, loaded_y};
            ui_window_size_ = ImVec2{loaded_w, loaded_h};
            ui_window_rect_valid_ = true;
            ui_window_restore_pending_ = true;
        }
    }

    void save_ui_state_() const noexcept {
        std::ofstream out{std::string{kUiStatePath}, std::ios::trunc};
        if (!out.is_open()) {
            return;
        }

        out << "selected_tab=" << tab_to_string_(selected_tab_) << '\n';
        if (ui_window_rect_valid_) {
            out << "window_pos_x=" << ui_window_pos_.x << '\n';
            out << "window_pos_y=" << ui_window_pos_.y << '\n';
            out << "window_size_x=" << ui_window_size_.x << '\n';
            out << "window_size_y=" << ui_window_size_.y << '\n';
        }
    }

    [[nodiscard]] ImGuiTabItemFlags tab_item_flags_(const UiTab tab, const bool restore_pending) const noexcept {
        if (restore_pending && tab == selected_tab_) {
            return ImGuiTabItemFlags_SetSelected;
        }
        return ImGuiTabItemFlags_None;
    }

    void handle_keyboard_shortcuts_(const UiSnapshot &snapshot, UiCommands &commands) const {
        if (!snapshot.has_physics) {
            return;
        }

        const ImGuiIO &io = ImGui::GetIO();
        if (io.WantTextInput) {
            return;
        }

        if (ImGui::IsKeyPressed(ImGuiKey_F5, false)) {
            commands.has_simulation_paused = true;
            commands.simulation_paused = !snapshot.simulation_paused;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_F6, false)) {
            add_requested_simulation_steps_(commands, 1u);
        }
        if (ImGui::IsKeyPressed(ImGuiKey_F7, false)) {
            add_requested_simulation_steps_(commands, 10u);
        }
        if (ImGui::IsKeyPressed(ImGuiKey_F9, false)) {
            commands.request_reset = true;
        }
    }

    [[nodiscard]] UiSnapshot build_ui_snapshot_(const f32 render_dt_ms) const noexcept {
        UiSnapshot snapshot{
            .render_dt_ms = render_dt_ms,
            .debug_toggles = debug_,
        };

        if (scene_ != nullptr) {
            const auto scene_view = scene_->render_view();
            snapshot.scene_body_count = static_cast<u32>(scene_view.generation.size());
            snapshot.scene_shape_count = static_cast<u32>(scene_view.shapes.size());
            snapshot.scene_constraint_count = static_cast<u32>(scene_view.constraints.size());
        }

        if (physics_ == nullptr) {
            return snapshot;
        }

        snapshot.has_physics = true;
        snapshot.simulation_paused = physics_->simulation_paused();
        snapshot.pending_simulation_steps = physics_->pending_simulation_steps();
        snapshot.completed_simulation_steps = physics_->completed_simulation_steps();
        snapshot.gravity = physics_->gravity();
        snapshot.linear_damping = physics_->linear_damping();
        snapshot.angular_damping = physics_->angular_damping();
        snapshot.physics_dt_ms = physics_->last_tick_dt_ms();
        return snapshot;
    }

    void build_ui_(const UiSnapshot &snapshot, UiCommands &commands) {
        if (ui_window_restore_pending_ && ui_window_rect_valid_) {
            ImGui::SetNextWindowPos(ui_window_pos_, ImGuiCond_Always);
            ImGui::SetNextWindowSize(ui_window_size_, ImGuiCond_Always);
            ui_window_restore_pending_ = false;
        } else {
            ImGui::SetNextWindowSize(ImVec2(460.0f, 620.0f), ImGuiCond_FirstUseEver);
        }

        ImGui::Begin("Javelin");
        ui_window_pos_ = ImGui::GetWindowPos();
        ui_window_size_ = ImGui::GetWindowSize();
        ui_window_rect_valid_ = true;

        handle_keyboard_shortcuts_(snapshot, commands);

        draw_status_row_(snapshot);
        ImGui::Separator();

        if (ImGui::BeginTabBar("ControlTabs")) {
            bool tab_restore_pending = ui_tab_restore_pending_;

            if (ImGui::BeginTabItem("Simulation", nullptr, tab_item_flags_(UiTab::simulation, tab_restore_pending))) {
                selected_tab_ = UiTab::simulation;
                tab_restore_pending = false;
                draw_simulation_section_(snapshot, commands);
                draw_parameters_section_(snapshot, commands);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Debug", nullptr, tab_item_flags_(UiTab::debug, tab_restore_pending))) {
                selected_tab_ = UiTab::debug;
                tab_restore_pending = false;
                DebugToggles edited_debug = snapshot.debug_toggles;
                draw_debug_section_(edited_debug);
                if (!debug_toggles_equal_(edited_debug, snapshot.debug_toggles)) {
                    commands.has_debug_toggles = true;
                    commands.debug_toggles = edited_debug;
                }
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Performance", nullptr, tab_item_flags_(UiTab::performance, tab_restore_pending))) {
                selected_tab_ = UiTab::performance;
                tab_restore_pending = false;
                draw_performance_section_(snapshot);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Scene", nullptr, tab_item_flags_(UiTab::scene, tab_restore_pending))) {
                selected_tab_ = UiTab::scene;
                tab_restore_pending = false;
                draw_scene_section_(snapshot, commands);
                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
            ui_tab_restore_pending_ = tab_restore_pending;
        }

        ImGui::End();
    }

    void draw_status_row_(const UiSnapshot &snapshot) const {
        if (!snapshot.has_physics) {
            ImGui::TextDisabled("Physics unavailable");
            return;
        }

        const ImVec4 status_color =
            snapshot.simulation_paused ? ImVec4{0.95f, 0.80f, 0.25f, 1.0f} : ImVec4{0.30f, 0.90f, 0.40f, 1.0f};
        ImGui::TextUnformatted("Status:");
        ImGui::SameLine();
        ImGui::TextColored(status_color, "%s", snapshot.simulation_paused ? "Paused" : "Running");
        ImGui::SameLine();
        ImGui::Text("Pending: %u", snapshot.pending_simulation_steps);
        ImGui::SameLine();
        ImGui::Text("Completed: %llu", static_cast<unsigned long long>(snapshot.completed_simulation_steps));
    }

    void draw_debug_section_(DebugToggles &debug) const {
        if (ImGui::Button("All On")) {
            set_all_debug_toggles_(debug, true);
        }
        ImGui::SameLine();
        if (ImGui::Button("All Off")) {
            set_all_debug_toggles_(debug, false);
        }
        ImGui::SameLine();
        help_marker_("Enable or disable every debug and display toggle in this tab.");

        section_header_("Display");

        ImGui::Checkbox("Grid", &debug.draw_grid);
        ImGui::SameLine();
        help_marker_("Render the world-space reference grid.");

        ImGui::Checkbox("Color Transform", &debug.apply_color_transform);
        ImGui::SameLine();
        help_marker_("Apply display color transform to the final image.");

        section_header_("Overlays");

        ImGui::Checkbox("Contacts", &debug.draw_contacts);
        ImGui::SameLine();
        help_marker_("Render collision contact points and normals.");

        ImGui::Checkbox("AABBs", &debug.draw_aabbs);
        ImGui::SameLine();
        help_marker_("Render per-body axis-aligned bounding box wireframes.");

        ImGui::Checkbox("Sleep State", &debug.draw_sleep_state);
        ImGui::SameLine();
        help_marker_("Overlay sleeping bodies gray and recently-woken bodies yellow.");

        ImGui::Checkbox("Constraints", &debug.draw_constraints);
        ImGui::SameLine();
        help_marker_("Draw lines between distance-constraint anchor points.");

        ImGui::Checkbox("Velocities", &debug.draw_velocities);
        ImGui::SameLine();
        help_marker_("Draw linear (green) and angular (magenta) velocity vectors.");
    }

    void draw_simulation_section_(const UiSnapshot &snapshot, UiCommands &commands) const {
        if (!snapshot.has_physics) {
            ImGui::TextDisabled("Physics controls unavailable.");
            return;
        }

        section_header_("Simulation");
        if (ImGui::Button(snapshot.simulation_paused ? "Resume" : "Pause")) {
            commands.has_simulation_paused = true;
            commands.simulation_paused = !snapshot.simulation_paused;
        }
        ImGui::SameLine();
        help_marker_("Pause or resume continuous fixed-rate simulation. Shortcut: F5.");

        ImGui::SameLine();
        ImGui::BeginDisabled(!snapshot.simulation_paused);
        if (ImGui::Button("Step")) {
            add_requested_simulation_steps_(commands, 1u);
        }
        ImGui::SameLine();
        help_marker_("Advance exactly one fixed simulation tick (1/60 s). Shortcut: F6.");
        ImGui::SameLine();
        if (ImGui::Button("Step x10")) {
            add_requested_simulation_steps_(commands, 10u);
        }
        ImGui::SameLine();
        help_marker_("Queue ten fixed simulation ticks while paused. Shortcut: F7.");
        ImGui::EndDisabled();

        ImGui::Text("State: %s", snapshot.simulation_paused ? "Paused" : "Running");
        ImGui::Text("Pending steps: %u", snapshot.pending_simulation_steps);
        ImGui::Text("Completed steps: %llu", static_cast<unsigned long long>(snapshot.completed_simulation_steps));
    }

    void draw_parameters_section_(const UiSnapshot &snapshot, UiCommands &commands) const {
        if (!snapshot.has_physics) {
            return;
        }

        section_header_("Parameters");

        f32 gravity = snapshot.gravity;
        if (ImGui::DragFloat("Gravity (m/s²)", &gravity, 0.1f, -50.0f, 0.0f)) {
            commands.has_gravity = true;
            commands.gravity = gravity;
        }
        ImGui::SameLine();
        help_marker_("Constant vertical acceleration applied to dynamic bodies.");

        f32 linear_damping = snapshot.linear_damping;
        if (ImGui::DragFloat("Linear Damping (1/s)", &linear_damping, 0.01f, 0.0f, 5.0f)) {
            commands.has_linear_damping = true;
            commands.linear_damping = linear_damping;
        }
        ImGui::SameLine();
        help_marker_("Exponential damping coefficient applied to linear velocity.");

        f32 angular_damping = snapshot.angular_damping;
        if (ImGui::DragFloat("Angular Damping (1/s)", &angular_damping, 0.01f, 0.0f, 5.0f)) {
            commands.has_angular_damping = true;
            commands.angular_damping = angular_damping;
        }
        ImGui::SameLine();
        help_marker_("Exponential damping coefficient applied to angular velocity.");
    }

    void draw_performance_section_(const UiSnapshot &snapshot) {
        ImGui::Text("Frame dt: %.3f ms", snapshot.render_dt_ms);
        const int render_history_count =
            render_dt_history_full_ ? static_cast<int>(kRenderDtHistory) : static_cast<int>(render_dt_cursor_);
        if (render_history_count > 0) {
            const int offset = render_dt_history_full_ ? static_cast<int>(render_dt_cursor_) : 0;
            plot_dt_history_("Frame dt (ms)", render_dt_history_.data(), render_history_count, offset,
                             kRenderDtHistory);
        }

        if (!snapshot.has_physics) {
            ImGui::TextDisabled("Physics timing unavailable.");
            return;
        }

        if (snapshot.physics_dt_ms > 0.0f &&
            snapshot.completed_simulation_steps != last_physics_dt_sample_step_count_) {
            push_physics_dt_sample_(snapshot.physics_dt_ms);
            last_physics_dt_sample_step_count_ = snapshot.completed_simulation_steps;
        }

        ImGui::Text("Physics dt: %.3f ms", snapshot.physics_dt_ms);
        const int physics_history_count =
            physics_dt_history_full_ ? static_cast<int>(kPhysicsDtHistory) : static_cast<int>(physics_dt_cursor_);
        if (physics_history_count > 0) {
            const int offset = physics_dt_history_full_ ? static_cast<int>(physics_dt_cursor_) : 0;
            plot_dt_history_("Physics dt (ms)", physics_dt_history_.data(), physics_history_count, offset,
                             kPhysicsDtHistory);
        }
    }

    void draw_scene_section_(const UiSnapshot &snapshot, UiCommands &commands) const {
        section_header_("Scene Stats");
        ImGui::Text("Bodies: %u", snapshot.scene_body_count);
        ImGui::Text("Shapes: %u", snapshot.scene_shape_count);
        ImGui::Text("Constraints: %u", snapshot.scene_constraint_count);

        section_header_("Actions");
        ImGui::BeginDisabled(!snapshot.has_physics);
        if (ImGui::Button("Reset Scene")) {
            commands.request_reset = true;
        }
        ImGui::SameLine();
        help_marker_("Restore authored initial transforms and velocities. Shortcut: F9.");
        ImGui::EndDisabled();
    }

    void apply_ui_commands_(const UiCommands &commands) noexcept {
        if (commands.has_debug_toggles) {
            debug_ = commands.debug_toggles;
        }

        if (physics_ == nullptr) {
            return;
        }

        if (commands.has_simulation_paused) {
            physics_->set_simulation_paused(commands.simulation_paused);
        }
        if (commands.requested_simulation_steps > 0u) {
            physics_->request_simulation_steps(commands.requested_simulation_steps);
        }
        if (commands.has_gravity) {
            physics_->set_gravity(commands.gravity);
        }
        if (commands.has_linear_damping) {
            physics_->set_linear_damping(commands.linear_damping);
        }
        if (commands.has_angular_damping) {
            physics_->set_angular_damping(commands.angular_damping);
        }
        if (commands.request_reset) {
            physics_->request_reset();
        }
    }

    [[nodiscard]] static bool debug_toggles_equal_(const DebugToggles &a, const DebugToggles &b) noexcept {
        return a.draw_grid == b.draw_grid && a.draw_contacts == b.draw_contacts && a.draw_aabbs == b.draw_aabbs &&
               a.draw_sleep_state == b.draw_sleep_state && a.draw_constraints == b.draw_constraints &&
               a.draw_velocities == b.draw_velocities && a.apply_color_transform == b.apply_color_transform;
    }

    static void set_all_debug_toggles_(DebugToggles &debug, const bool enabled) noexcept {
        debug.draw_grid = enabled;
        debug.draw_contacts = enabled;
        debug.draw_aabbs = enabled;
        debug.draw_sleep_state = enabled;
        debug.draw_constraints = enabled;
        debug.draw_velocities = enabled;
        debug.apply_color_transform = enabled;
    }

    static void add_requested_simulation_steps_(UiCommands &commands, const u32 step_count) noexcept {
        const u32 remaining = std::numeric_limits<u32>::max() - commands.requested_simulation_steps;
        commands.requested_simulation_steps += std::min(step_count, remaining);
    }

    static void help_marker_(const char *text) noexcept {
        ImGui::TextDisabled("(?)");
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
            ImGui::BeginTooltip();
            ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
            ImGui::TextUnformatted(text);
            ImGui::PopTextWrapPos();
            ImGui::EndTooltip();
        }
    }

    static void section_header_(const char *label) noexcept {
        ImGui::Spacing();
        ImGui::SeparatorText(label);
    }

    static void plot_dt_history_(const char *label, const f32 *history, const int history_count, const int offset,
                                 const usize history_capacity) {
        f32 min_v = 0.0f;
        f32 max_v = 0.0f;
        double sum = 0.0;
        for (int i = 0; i < history_count; ++i) {
            const usize idx = (static_cast<usize>(offset) + static_cast<usize>(i)) % history_capacity;
            const f32 v = history[idx];
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
        ImGui::PlotLines(label, history, history_count, offset, nullptr, plot_min, plot_max, ImVec2(0, 80));
    }

    void init_imgui_() const {
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        apply_imgui_theme_();

        ImGui_ImplGlfw_InitForOpenGL(window_.native, /*install_callbacks=*/true);
        ImGui_ImplOpenGL3_Init("#version 460");
    }

    static void apply_imgui_theme_() noexcept {
        ImGui::StyleColorsDark();
        ImGuiStyle &style = ImGui::GetStyle();

        // Lightweight, readable defaults for control-heavy tooling UI.
        style.WindowPadding = ImVec2(12.0f, 10.0f);
        style.FramePadding = ImVec2(8.0f, 5.0f);
        style.CellPadding = ImVec2(8.0f, 4.0f);
        style.ItemSpacing = ImVec2(10.0f, 8.0f);
        style.ItemInnerSpacing = ImVec2(8.0f, 6.0f);
        style.TouchExtraPadding = ImVec2(0.0f, 0.0f);
        style.IndentSpacing = 18.0f;
        style.ScrollbarSize = 14.0f;
        style.GrabMinSize = 11.0f;

        style.WindowRounding = 8.0f;
        style.ChildRounding = 6.0f;
        style.FrameRounding = 6.0f;
        style.PopupRounding = 6.0f;
        style.ScrollbarRounding = 8.0f;
        style.GrabRounding = 6.0f;
        style.TabRounding = 6.0f;

        style.WindowBorderSize = 1.0f;
        style.ChildBorderSize = 1.0f;
        style.PopupBorderSize = 1.0f;
        style.FrameBorderSize = 0.0f;
        style.TabBorderSize = 0.0f;

        style.WindowTitleAlign = ImVec2(0.04f, 0.5f);
        style.SeparatorTextBorderSize = 1.0f;
        style.SeparatorTextAlign = ImVec2(0.0f, 0.5f);
        style.SeparatorTextPadding = ImVec2(6.0f, 3.0f);

        ImVec4 *colors = style.Colors;
        colors[ImGuiCol_WindowBg] = ImVec4(0.11f, 0.12f, 0.14f, 1.0f);
        colors[ImGuiCol_TitleBg] = ImVec4(0.10f, 0.11f, 0.13f, 1.0f);
        colors[ImGuiCol_TitleBgActive] = ImVec4(0.16f, 0.18f, 0.22f, 1.0f);
        colors[ImGuiCol_FrameBg] = ImVec4(0.19f, 0.21f, 0.25f, 1.0f);
        colors[ImGuiCol_FrameBgHovered] = ImVec4(0.24f, 0.28f, 0.34f, 1.0f);
        colors[ImGuiCol_FrameBgActive] = ImVec4(0.27f, 0.33f, 0.40f, 1.0f);
        colors[ImGuiCol_Button] = ImVec4(0.23f, 0.30f, 0.41f, 1.0f);
        colors[ImGuiCol_ButtonHovered] = ImVec4(0.31f, 0.40f, 0.54f, 1.0f);
        colors[ImGuiCol_ButtonActive] = ImVec4(0.19f, 0.26f, 0.36f, 1.0f);
        colors[ImGuiCol_Tab] = ImVec4(0.17f, 0.19f, 0.24f, 1.0f);
        colors[ImGuiCol_TabHovered] = ImVec4(0.30f, 0.37f, 0.49f, 1.0f);
        colors[ImGuiCol_TabActive] = ImVec4(0.24f, 0.31f, 0.43f, 1.0f);
        colors[ImGuiCol_CheckMark] = ImVec4(0.56f, 0.78f, 0.98f, 1.0f);
        colors[ImGuiCol_SliderGrab] = ImVec4(0.51f, 0.72f, 0.93f, 1.0f);
        colors[ImGuiCol_SliderGrabActive] = ImVec4(0.67f, 0.84f, 0.99f, 1.0f);
        colors[ImGuiCol_Header] = ImVec4(0.23f, 0.30f, 0.41f, 1.0f);
        colors[ImGuiCol_HeaderHovered] = ImVec4(0.30f, 0.39f, 0.52f, 1.0f);
        colors[ImGuiCol_HeaderActive] = ImVec4(0.20f, 0.28f, 0.38f, 1.0f);
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
    // Pipeline order: opaque geometry → surface overlays → x-ray overlays → display.
    // SleepDebugPass uses GL_LEQUAL depth (surface overlay) and must run before the
    // x-ray passes (ContactDebugPass, AabbDebugPass, ConstraintDebugPass) which disable
    // depth testing.
    using Pipeline = RenderPipeline<GeometryPass, WorldGridPass, SleepDebugPass, ContactDebugPass, AabbDebugPass,
                                    ConstraintDebugPass, VelocityDebugPass, DisplayPass>;

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
    // Cap interpolation span so a single step after a long pause does not blend slowly.
    // Two fixed physics ticks keeps motion smooth without long wall-clock lag.
    static constexpr u64 kMaxPoseInterpolationSpanNs = 2u * (1'000'000'000ull / 60u);
    // Last completed simulation step id that contributed a physics-dt history sample.
    // This avoids duplicating a single physics tick across many render frames.
    u64 last_physics_dt_sample_step_count_{0};

    UiTab selected_tab_{UiTab::simulation};
    bool ui_tab_restore_pending_{false};
    bool ui_window_rect_valid_{false};
    bool ui_window_restore_pending_{false};
    ImVec2 ui_window_pos_{0.0f, 0.0f};
    ImVec2 ui_window_size_{0.0f, 0.0f};

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

        const u64 raw_span = poses.curr_time_ns - poses.prev_time_ns;
        const u64 interpolation_span = std::min(raw_span, kMaxPoseInterpolationSpanNs);
        if (interpolation_span == 0u) {
            return 1.0f;
        }

        // Interpolate over a bounded window ending at the latest published pose.
        // This avoids long blends when stepping after an extended pause.
        const u64 interpolation_start_time = poses.curr_time_ns - interpolation_span;
        const u64 now = now_ns_();
        const u64 render_time = (now > interpolation_span) ? (now - interpolation_span) : 0u;
        const u64 sample_time = std::clamp(render_time, interpolation_start_time, poses.curr_time_ns);
        const f64 alpha =
            static_cast<f64>(sample_time - interpolation_start_time) / static_cast<f64>(interpolation_span);
        return static_cast<f32>(alpha);
    }
};

} // namespace javelin

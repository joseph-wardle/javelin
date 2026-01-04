export module javelin.render.render_system;

import javelin.scene;

export namespace javelin {
    struct RenderSystem {
        void init() {}
        void build_runtime(Scene scene) {}
        void shutdown() {}
    };
}
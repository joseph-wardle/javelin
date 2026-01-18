export module javelin.render.passes.geometry_pass;

import javelin.render.render_context;
import javelin.render.types;

export namespace javelin {

struct GeometryPass final {
    template <class Device> void init(Device &) {
        // compile shaders, create VAOs/VBOs, etc.
    }

    template <class Device> void resize(Device &, Extent2D) {}

    template <class Device> void shutdown(Device &) {}

    void execute(RenderContext &ctx) {
        // bind ctx.targets.scene_fbo
        // set viewport ctx.extent
        // draw spheres using ctx.poses + ctx.view static info
    }
};

} // namespace javelin

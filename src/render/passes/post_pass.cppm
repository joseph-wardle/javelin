module;

#include <glad/gl.h>
#include <tracy/TracyOpenGL.hpp>

export module javelin.render.passes.post_pass;

import javelin.core.logging;
import javelin.render.render_context;
import javelin.render.render_targets;
import javelin.render.types;

export namespace javelin {

struct PostPass final {
    template <class Device> void init(Device &) { log::info(render, "Initializing post process pass"); }

    template <class Device> void resize(Device &, Extent2D) {}

    template <class Device> void shutdown(Device &) { log::info(render, "Shutting down post process pass"); }

    void execute(RenderContext &ctx) {
        if (!ctx.extent.is_valid() || ctx.targets.scene_fbo == 0) {
            return;
        }

        ZoneScopedN("PostPass");
        TracyGpuZone("PostPass");
        glBindFramebuffer(GL_READ_FRAMEBUFFER, ctx.targets.scene_fbo);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
        glViewport(0, 0, ctx.extent.width, ctx.extent.height);
        glBlitFramebuffer(0, 0, ctx.extent.width, ctx.extent.height, 0, 0, ctx.extent.width, ctx.extent.height,
                          GL_COLOR_BUFFER_BIT, GL_NEAREST);
    }
};

} // namespace javelin

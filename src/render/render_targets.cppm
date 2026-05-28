module;

#include <glad/gl.h>
#include <tracy/Tracy.hpp>

export module javelin.render.render_targets;

import javelin.core.logging;
import javelin.core.types;
import javelin.render.types;

namespace javelin::detail {
void setup_color_target(TextureHandle texture, const Extent2D extent) noexcept {
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, extent.width, extent.height, 0, GL_RGBA, GL_FLOAT, nullptr);
}

void setup_depth_target(TextureHandle texture, const Extent2D extent) noexcept {
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, extent.width, extent.height, 0, GL_DEPTH_COMPONENT,
                 GL_UNSIGNED_INT, nullptr);
}

void validate_fbo(const char *label) noexcept {
    const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        log::error(render, "Framebuffer {} incomplete: 0x{:X}", label, static_cast<u32>(status));
    }
}
} // namespace javelin::detail

export namespace javelin {

class RenderTargets final {
  public:
    void init();
    void resize(Extent2D new_extent);
    void shutdown();

    [[nodiscard]] Extent2D extent() const noexcept { return extent_; }
    [[nodiscard]] bool ready() const noexcept { return scene_fbo_ != 0 && extent_.is_valid(); }

    void bind_for_drawing() const noexcept { glBindFramebuffer(GL_FRAMEBUFFER, scene_fbo_); }

    void bind_color_for_sampling(const u32 texture_unit) const noexcept {
        glActiveTexture(GL_TEXTURE0 + texture_unit);
        glBindTexture(GL_TEXTURE_2D, scene_color_);
    }

    void blit_color_to_default() const noexcept {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, scene_fbo_);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
        glViewport(0, 0, extent_.width, extent_.height);
        glBlitFramebuffer(0, 0, extent_.width, extent_.height, 0, 0, extent_.width, extent_.height,
                          GL_COLOR_BUFFER_BIT, GL_NEAREST);
    }

  private:
    Extent2D extent_{};
    FboHandle scene_fbo_{};
    TextureHandle scene_color_{};
    TextureHandle scene_depth_{};
};

void RenderTargets::init() {
    ZoneScopedN("RenderTargets init");
    log::info(render, "Initializing render targets");
    glGenFramebuffers(1, &scene_fbo_);
    glGenTextures(1, &scene_color_);
    glGenTextures(1, &scene_depth_);
}

void RenderTargets::resize(const Extent2D new_extent) {
    if (!new_extent.is_valid()) {
        extent_ = new_extent;
        return;
    }
    if (extent_.width == new_extent.width && extent_.height == new_extent.height) {
        return;
    }

    ZoneScopedN("RenderTargets resize");
    extent_ = new_extent;
    log::info(render, "Render targets resized to {}x{}", extent_.width, extent_.height);

    detail::setup_color_target(scene_color_, extent_);
    detail::setup_depth_target(scene_depth_, extent_);

    glBindFramebuffer(GL_FRAMEBUFFER, scene_fbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, scene_color_, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, scene_depth_, 0);
    const GLenum scene_attachments[1] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, scene_attachments);
    detail::validate_fbo("scene");

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void RenderTargets::shutdown() {
    ZoneScopedN("RenderTargets shutdown");
    log::info(render, "Render targets shutdown");
    if (scene_fbo_ != 0) {
        glDeleteFramebuffers(1, &scene_fbo_);
        scene_fbo_ = 0;
    }
    if (scene_color_ != 0) {
        glDeleteTextures(1, &scene_color_);
        scene_color_ = 0;
    }
    if (scene_depth_ != 0) {
        glDeleteTextures(1, &scene_depth_);
        scene_depth_ = 0;
    }

    extent_ = {};
}

} // namespace javelin

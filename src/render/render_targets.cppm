module;

#include <tracy/Tracy.hpp>

#include <glad/gl.h>

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
        log::error("{} framebuffer incomplete: 0x{:X}", label, static_cast<u32>(status));
    }
}
} // namespace javelin::detail

export namespace javelin {

struct RenderTargets final {
    Extent2D extent{};

    // Main scene target (linear color + depth)
    FboHandle scene_fbo{};
    TextureHandle scene_color{};
    TextureHandle scene_depth{};

    // Post ping-pong (for bloom/SSAO later)
    FboHandle ping_fbo{};
    TextureHandle ping_color{};
    FboHandle pong_fbo{};
    TextureHandle pong_color{};

    void init();
    void resize(Extent2D extent);
    void shutdown();
};

void RenderTargets::init() {
    ZoneScopedN("RenderTargets init");
    glGenFramebuffers(1, &scene_fbo);
    glGenTextures(1, &scene_color);
    glGenTextures(1, &scene_depth);

    glGenFramebuffers(1, &ping_fbo);
    glGenTextures(1, &ping_color);
    glGenFramebuffers(1, &pong_fbo);
    glGenTextures(1, &pong_color);
}

void RenderTargets::resize(const Extent2D new_extent) {
    if (!new_extent.is_valid()) {
        extent = new_extent;
        return;
    }
    if (extent.width == new_extent.width && extent.height == new_extent.height) {
        return;
    }

    ZoneScopedN("RenderTargets resize");
    extent = new_extent;

    detail::setup_color_target(scene_color, extent);
    detail::setup_depth_target(scene_depth, extent);

    glBindFramebuffer(GL_FRAMEBUFFER, scene_fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, scene_color, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, scene_depth, 0);
    const GLenum scene_attachments[1] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, scene_attachments);
    detail::validate_fbo("scene");

    detail::setup_color_target(ping_color, extent);
    glBindFramebuffer(GL_FRAMEBUFFER, ping_fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, ping_color, 0);
    glDrawBuffers(1, scene_attachments);
    detail::validate_fbo("ping");

    detail::setup_color_target(pong_color, extent);
    glBindFramebuffer(GL_FRAMEBUFFER, pong_fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, pong_color, 0);
    glDrawBuffers(1, scene_attachments);
    detail::validate_fbo("pong");

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void RenderTargets::shutdown() {
    ZoneScopedN("RenderTargets shutdown");
    if (scene_fbo != 0) {
        glDeleteFramebuffers(1, &scene_fbo);
        scene_fbo = 0;
    }
    if (ping_fbo != 0) {
        glDeleteFramebuffers(1, &ping_fbo);
        ping_fbo = 0;
    }
    if (pong_fbo != 0) {
        glDeleteFramebuffers(1, &pong_fbo);
        pong_fbo = 0;
    }

    if (scene_color != 0) {
        glDeleteTextures(1, &scene_color);
        scene_color = 0;
    }
    if (scene_depth != 0) {
        glDeleteTextures(1, &scene_depth);
        scene_depth = 0;
    }
    if (ping_color != 0) {
        glDeleteTextures(1, &ping_color);
        ping_color = 0;
    }
    if (pong_color != 0) {
        glDeleteTextures(1, &pong_color);
        pong_color = 0;
    }

    extent = {};
}

} // namespace javelin

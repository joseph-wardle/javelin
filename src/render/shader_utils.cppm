module;

#include <glad/gl.h>

export module javelin.render.shader_utils;

import std;

import javelin.core.logging;
import javelin.core.types;

// Shared GL shader/program build helpers.
//
// Every render pass needs the same boilerplate: create a shader, compile,
// check the log, link a program, detach + delete the intermediate shaders
// on success.  These helpers fold that pattern into two functions so each
// pass only has to provide its shader sources and a log label.
//
// The `label` is included verbatim in error messages so a failing compile
// or link still tells you which pass is broken.
export namespace javelin::shader_utils {

// Compiles a single shader stage.  Returns 0 and logs on failure.
// On failure the partially-created shader is deleted; callers do not need to clean up.
[[nodiscard]] inline u32 compile_shader(const GLenum type, const std::string_view source,
                                        const std::string_view label) noexcept {
    const GLuint shader = glCreateShader(type);
    const char *src = source.data();
    const GLint len = static_cast<GLint>(source.size());
    glShaderSource(shader, 1, &src, &len);
    glCompileShader(shader);

    GLint ok = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (ok == GL_TRUE) {
        return shader;
    }

    std::array<char, 1024> info{};
    glGetShaderInfoLog(shader, static_cast<GLsizei>(info.size()), nullptr, info.data());
    log::error(render, "{} shader compile failed: {}", label, info.data());
    glDeleteShader(shader);
    return 0;
}

// Links a vertex + fragment shader pair into a program.  On success, detaches and deletes
// both inputs (so the caller can drop them after this call).  On failure, deletes everything
// and returns 0.
[[nodiscard]] inline u32 link_program(const u32 vs, const u32 fs, const std::string_view label) noexcept {
    const GLuint program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);

    GLint ok = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (ok == GL_TRUE) {
        glDetachShader(program, vs);
        glDetachShader(program, fs);
        glDeleteShader(vs);
        glDeleteShader(fs);
        return program;
    }

    std::array<char, 1024> info{};
    glGetProgramInfoLog(program, static_cast<GLsizei>(info.size()), nullptr, info.data());
    log::error(render, "{} shader link failed: {}", label, info.data());
    glDeleteProgram(program);
    glDeleteShader(vs);
    glDeleteShader(fs);
    return 0;
}

// Compiles VS + FS and links them in one shot.  If either compile fails, the other
// is cleaned up and 0 is returned.  Convenience wrapper for the common case where
// the caller doesn't need the intermediate shader handles.
[[nodiscard]] inline u32 build_program(const std::string_view vertex_source, const std::string_view fragment_source,
                                       const std::string_view label) noexcept {
    const u32 vs = compile_shader(GL_VERTEX_SHADER, vertex_source, label);
    const u32 fs = compile_shader(GL_FRAGMENT_SHADER, fragment_source, label);
    if (vs == 0 || fs == 0) {
        if (vs != 0) {
            glDeleteShader(vs);
        }
        if (fs != 0) {
            glDeleteShader(fs);
        }
        return 0;
    }
    return link_program(vs, fs, label);
}

} // namespace javelin::shader_utils

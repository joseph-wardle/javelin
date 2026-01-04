import std;

import javelin.core.logging;
import javelin.core.types;
import javelin.math.vec2;
import javelin.math.vec3;
import javelin.math.vec4;
import javelin.math.mat3;
import javelin.math.mat4;
import javelin.math.quat;
import javelin.math.constants;

using namespace javelin;

int main() {
    log::trace("This is a trace log");
    log::debug("This is a debug log");
    log::info("This is a info log");
    log::warn("This is a warn log");
    log::error("This is a error log");
    log::critical("This is a critical log");

    for (usize i = 0; i < 1000000; ++i) {
        auto a = Vec2(1.0f, 2.0f);
        log::info("Vec2 a: ({}, {})", a.x, a.y);
        auto b = Vec2(3.0f, 4.0f);
        log::info("Vec2 b: ({}, {})", b.x, b.y);
        Vec2 c = a + b;
        log::info("Vec2 c = a + b: ({}, {})", c.x, c.y);

        auto u = Vec3(1.0f, 0.0f, 0.0f);
        log::info("Vec3 u: ({}, {}, {})", u.x, u.y, u.z);
        auto v = Vec3(0.0f, 1.0f, 0.0f);
        log::info("Vec3 v: ({}, {}, {})", v.x, v.y, v.z);
        Vec3 w = cross(u, v);
        log::info("Vec3 w = cross(u, v): ({}, {}, {})", w.x, w.y, w.z);

        auto q = Quat::identity();
        log::info("Quat q (identity): ({}, {}, {}, {})", q.x, q.y, q.z, q.w);
        Vec3 rotated_v = rotate(q, v);
        log::info("Rotated v by q: ({}, {}, {})", rotated_v.x, rotated_v.y, rotated_v.z);
        log::info("Finished iteration {}", i);
    }

    return 0;
}
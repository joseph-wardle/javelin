import std;

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
    auto a = math::Vec2(1.0f, 2.0f);
    std::println("Vec2 a: ({}, {})", a.x, a.y);
    auto b = math::Vec2(3.0f, 4.0f);
    std::println("Vec2 b: ({}, {})", b.x, b.y);
    math::Vec2 c = a + b;
    std::println("Vec2 c = a + b: ({}, {})", c.x, c.y);

    auto u = math::Vec3(1.0f, 0.0f, 0.0f);
    std::println("Vec3 u: ({}, {}, {})", u.x, u.y, u.z);
    auto v = math::Vec3(0.0f, 1.0f, 0.0f);
    std::println("Vec3 v: ({}, {}, {})", v.x, v.y, v.z);
    math::Vec3 w = math::cross(u, v);
    std::println("Vec3 w = cross(u, v): ({}, {}, {})", w.x, w.y, w.z);

    auto q = math::Quat::identity();
    std::println("Quat q (identity): ({}, {}, {}, {})", q.x, q.y, q.z, q.w);
    math::Vec3 rotated_v = math::rotate(q, v);
    std::println("Rotated v by q: ({}, {}, {})", rotated_v.x, rotated_v.y, rotated_v.z);

    return 0;
}
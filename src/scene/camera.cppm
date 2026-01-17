export module javelin.scene.camera;

import javelin.core.types;
import javelin.math.vec3;

export namespace javelin {

struct Camera final {
    Vec3 position{};
    Vec3 forward{0, 0, -1};
    Vec3 up{0, 1, 0};
    f32 fov_y_radians{1.0f};
};

} // namespace javelin

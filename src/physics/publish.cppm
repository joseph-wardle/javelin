module;

#include <tracy/Tracy.hpp>

export module javelin.physics.publish;

import std;
import javelin.core.types;
import javelin.math.quat;
import javelin.math.vec3;
import javelin.scene.pose_channel;

export namespace javelin {

void publish_poses(PoseChannel &poses, std::span<const Vec3> position, std::span<const Quat> orientation,
                   const u32 count) noexcept {
    ZoneScopedN("Physics publish poses");
    auto out = poses.write_pose(count);
    for (u32 i = 0; i < count; ++i) {
        out.positions[i] = position[i];
        out.orientations[i] = orientation[i];
    }
    poses.publish();
}

} // namespace javelin

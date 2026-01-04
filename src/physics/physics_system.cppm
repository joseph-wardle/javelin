export module javelin.physics.physics_system;

import javelin.core.types;
import javelin.scene;

export namespace javelin {
    struct PhysicsSystem {
        void init() {}
        void build_runtime(Scene scene) {}
        void step(const f64 delta) {}
        void shutdown() {}
    };
}
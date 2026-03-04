export module javelin.scene.physics_materials;

import javelin.core.types;

export namespace javelin {

// Physics material contract:
// - restitution in [0, 1]: 0 = perfectly inelastic (no bounce), 1 = perfectly elastic.
// - friction in [0, inf): Coulomb friction coefficient; higher values resist sliding more.
// - material_id = 0 is the implicit default; kDefaultPhysicsMaterial is always present at
//   index 0 of the material pool.
// - the solver combines two body materials pair-wise: geometric mean for restitution,
//   minimum for friction (see physics_system.cppm: detail::combined_restitution/friction).
struct PhysicsMaterial final {
    f32 restitution{0.3f};
    f32 friction{0.2f};
};

// Material index 0: present in every scene's material pool even when no explicit materials
// are authored. Bodies that omit a material record receive this implicitly.
inline constexpr PhysicsMaterial kDefaultPhysicsMaterial{
    .restitution = 0.3f,
    .friction = 0.2f,
};

} // namespace javelin

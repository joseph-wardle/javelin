export module javelin.render.camera_controller;

import javelin.core.types;

export namespace javelin {

// Cursor capture intent returned by a camera's per-frame update.  Drives the
// GLFW cursor mode toggle in RenderSystem so the active mode can either
// release the cursor (orbit, idle fly) or lock and hide it (active fly drag).
enum struct CursorMode : u8 { normal, captured };

} // namespace javelin

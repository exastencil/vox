// Client platform integration (OS/window specific)
const builtin = @import("builtin");
const sokol = @import("sokol");
const sapp = sokol.app;

pub fn macosWarpCursorIntoAppWindow() void {
    if (builtin.os.tag != .macos) return;
    const cg = @cImport({
        @cInclude("CoreGraphics/CoreGraphics.h");
    });
    const display = cg.CGMainDisplayID();
    const bounds = cg.CGDisplayBounds(display);
    const cx: cg.CGFloat = bounds.origin.x + (bounds.size.width / 2.0);
    const cy: cg.CGFloat = bounds.origin.y + (bounds.size.height / 2.0);
    const pt = cg.CGPointMake(cx, cy);
    _ = cg.CGWarpMouseCursorPosition(pt);
}

// Enable/disable raw mouse input if supported by the sokol app wrapper
pub fn setRawMouse(on: bool) void {
    if (@hasDecl(sapp, "rawMouseSupported") and @hasDecl(sapp, "enableRawMouse")) {
        if (sapp.rawMouseSupported()) {
            sapp.enableRawMouse(on);
        }
    } else {}
}

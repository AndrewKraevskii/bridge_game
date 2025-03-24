const app = @import("sokol").app;
const Keycode = app.Keycode;
const Mousebutton = app.Mousebutton;
const Event = app.Event;
const geom = @import("geom");
const std = @import("std");

keyboard_state: std.EnumSet(Keycode),
keyboard_state_prev: std.EnumSet(Keycode),
mouse_delta: geom.Vec2,
mouse_position: geom.Vec2,
mouse_button_down: std.EnumSet(Mousebutton),
mouse_button_down_prev: std.EnumSet(Mousebutton),
mouse_scroll: f32,

pub const init: @This() = .{
    .keyboard_state = .{},
    .keyboard_state_prev = .{},
    .mouse_delta = .zero,
    .mouse_button_down = .{},
    .mouse_button_down_prev = .{},
    .mouse_position = .zero,
    .mouse_scroll = 0,
};

pub fn consumeEvent(input: *@This(), ev: *const Event) void {
    switch (ev.type) {
        .MOUSE_DOWN => input.mouse_button_down.insert(ev.mouse_button),
        .MOUSE_UP => input.mouse_button_down.remove(ev.mouse_button),
        .MOUSE_MOVE => {
            input.mouse_delta.add(.{ .x = ev.mouse_dx, .y = ev.mouse_dy });
            input.mouse_position = .{ .x = ev.mouse_x, .y = ev.mouse_y };
        },
        .KEY_DOWN => {
            if (ev.key_repeat) return;
            input.keyboard_state.insert(ev.key_code);
        },
        .KEY_UP => {
            input.keyboard_state.remove(ev.key_code);
        },
        .MOUSE_SCROLL => {
            input.mouse_scroll += ev.scroll_y;
        },
        else => {},
    }
}

pub fn isKeyDown(input: *const @This(), key: Keycode) bool {
    return input.keyboard_state.contains(key);
}

pub fn isKeyPressed(input: *const @This(), key: Keycode) bool {
    return input.keyboard_state.contains(key) and !input.keyboard_state_prev.contains(key);
}

pub fn isMouseDown(input: *const @This(), key: Mousebutton) bool {
    return input.mouse_button_down.contains(key);
}

pub fn isMouseReleased(input: *const @This(), key: Mousebutton) bool {
    return !input.mouse_button_down.contains(key) and input.mouse_button_down_prev.contains(key);
}

pub fn isMousePressed(input: *const @This(), key: Mousebutton) bool {
    return input.mouse_button_down.contains(key) and !input.mouse_button_down_prev.contains(key);
}

pub fn newFrame(input: *@This()) void {
    input.keyboard_state_prev = input.keyboard_state;
    input.mouse_button_down_prev = input.mouse_button_down;
    input.mouse_delta = .zero;
    input.mouse_scroll = 0;
}

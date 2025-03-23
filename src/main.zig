const std = @import("std");
const Sokol2d = @import("sokol_2d");
const sokol = @import("sokol");
const geo = @import("geom");
const Input = @import("Input.zig");
const SlotMap = @import("slot_map").SlotMap;
const simgui = sokol.imgui;
const ig = @import("imgui");

var state: State = .{};

const save_file_name = "autosave.level";

const Level = struct {
    points: std.ArrayListUnmanaged(Point),
    joints: std.ArrayListUnmanaged(Joint),

    pub const empty: Level = .{
        .joints = .empty,
        .points = .empty,
    };

    const Header = extern struct {
        points_len: u32,
        joints_len: u32,
    };

    pub fn save(level: *Level, file_path: []const u8) !void {
        const file = try std.fs.cwd().createFile(file_path, .{});
        defer file.close();
        var header: Header = .{
            .points_len = @intCast(level.points.items.len),
            .joints_len = @intCast(level.joints.items.len),
        };

        const points_bytes = std.mem.sliceAsBytes(level.points.items);
        const joints_bytes = std.mem.sliceAsBytes(level.joints.items);
        var iovec = [_]std.posix.iovec_const{
            .{
                .base = std.mem.asBytes(&header),
                .len = @sizeOf(Header),
            },
            .{
                .base = points_bytes.ptr,
                .len = points_bytes.len,
            },
            .{
                .base = joints_bytes.ptr,
                .len = joints_bytes.len,
            },
        };
        try file.writevAll(&iovec);
        std.log.info("saved to {s}", .{file_path});
    }

    pub fn load(
        gpa: std.mem.Allocator,
        file_path: []const u8,
    ) !Level {
        const file = try std.fs.cwd().openFile(file_path, .{});
        defer file.close();

        const header = try file.reader().readStruct(Header);

        var level: Level = .{
            .points = .empty,
            .joints = .empty,
        };

        try level.points.resize(gpa, header.points_len);
        errdefer level.points.deinit(gpa);
        try level.joints.resize(gpa, header.joints_len);
        errdefer level.joints.deinit(gpa);

        const points_bytes = std.mem.sliceAsBytes(level.points.items);
        const joints_bytes = std.mem.sliceAsBytes(level.joints.items);
        var iovec = [2]std.posix.iovec{
            .{
                .base = points_bytes.ptr,
                .len = points_bytes.len,
            },
            .{
                .base = joints_bytes.ptr,
                .len = joints_bytes.len,
            },
        };
        _ = try file.readvAll(&iovec);
        std.log.info("read from {s}", .{file_path});

        return level;
    }

    pub fn deinit(l: *Level, gpa: std.mem.Allocator) void {
        l.joints.deinit(gpa);
        l.points.deinit(gpa);
        l.* = .empty;
    }
};

const State = struct {
    gpa: std.mem.Allocator = undefined,

    sokol_2d: Sokol2d = undefined,
    load_action: sokol.gfx.PassAction = .{},

    input: Input = .init,

    prev_point: ?Point.Index = null,
    level: Level = .empty,

    save_file_name: [100:0]u8 = save_file_name.* ++ [1]u8{0} ** 86,

    mode: enum {
        play,
        stop,
    } = .stop,

    gravity_g: f32 = 10,
    spring: f32 = 0.01,
    mass: f32 = 1,
};

const Point = extern struct {
    pos: geo.Vec2,
    vel: geo.Vec2 = .zero,
    kind: enum(u8) {
        dinamic,
        static,

        fn color(p: @This()) Sokol2d.Color {
            return switch (p) {
                .dinamic => .red,
                .static => .black,
            };
        }
    } = .dinamic,
    const Index = u32;
};

const Joint = extern struct {
    from: Point.Index,
    to: Point.Index,
    length: f32,

    pub fn actualLength(joint: Joint, points: []const Point) f32 {
        const from_point = &points[joint.from];
        const to_point = &points[joint.to];

        return from_point.pos.dist(to_point.pos);
    }
};

fn init() callconv(.c) void {
    sokol.gfx.setup(.{
        .environment = sokol.glue.environment(),
        .logger = .{ .func = sokol.log.func },
    });

    std.log.info("make attachments", .{});
    state.sokol_2d = Sokol2d.init(state.gpa) catch {
        std.log.err("OOM", .{});
        sokol.app.quit();
        return;
    };
    simgui.setup(.{
        .logger = .{ .func = sokol.log.func },
    });
    state.load_action.colors[0] = .{
        .load_action = .LOAD,
    };
    state.level = Level.load(state.gpa, save_file_name) catch |e| switch (e) {
        error.FileNotFound => .empty,
        else => fatal(e),
    };
}

fn findPoint(points: []const Point, pos: geo.Vec2, distance: f32) ?Point.Index {
    for (points, 0..) |point, index| {
        if (point.pos.distSq(pos) < distance * distance) {
            return @intCast(index);
        }
    }
    return null;
}

fn getOrAddPoint(gpa: std.mem.Allocator, points: *std.ArrayListUnmanaged(Point), pos: geo.Vec2) Point.Index {
    errdefer |e| fatal(e);

    return if (findPoint(points.items, pos, 10)) |point|
        point
    else blk: {
        const index = points.items.len;
        try points.append(gpa, .{ .pos = pos });
        break :blk @intCast(index);
    };
}

fn addJoint(gpa: std.mem.Allocator, level: *Level, from: Point.Index, to: Point.Index) void {
    errdefer |e| fatal(e);
    if (from == to) return;

    const from_point = level.points.items[from];
    const to_point = level.points.items[to];

    try level.joints.append(gpa, .{
        .from = from,
        .to = to,
        .length = from_point.pos.dist(to_point.pos),
    });
}

fn frame() callconv(.c) void {
    errdefer |e| fatal(e);

    defer state.input.newFrame();

    const width: f32 = @floatFromInt(sokol.app.width());
    const height: f32 = @floatFromInt(sokol.app.height());
    const delta_t: f32 = @floatCast(sokol.app.frameDuration());

    // USER INPUT
    const mouse_pos = state.input.mouse_position;

    const dot_visual_radius = 10;
    const dot_click_radius = 20;

    if (state.input.isMousePressed(.LEFT)) {
        const point_index = getOrAddPoint(state.gpa, &state.level.points, mouse_pos);

        if (state.prev_point) |prev| {
            addJoint(state.gpa, &state.level, prev, point_index);

            std.log.debug("mouse button at {}", .{state.input.mouse_position});
            state.prev_point = null;
        } else {
            state.prev_point = point_index;
        }
    } else if (state.input.isMousePressed(.RIGHT)) blk: {
        const point = findPoint(state.level.points.items, mouse_pos, dot_click_radius) orelse break :blk;

        state.level.points.items[point].kind = switch (state.level.points.items[point].kind) {
            .static => .dinamic,
            .dinamic => .static,
        };
    } else if (state.input.isKeyPressed(.SPACE)) {
        state.mode = switch (state.mode) {
            .play => .stop,
            .stop => .play,
        };
        std.log.info("state {s}", .{@tagName(state.mode)});
    }

    // UPDATE
    if (state.mode == .play) {
        for (state.level.points.items) |*point| {
            if (point.kind == .static) continue;

            point.vel.addScaled(.y_pos, delta_t * state.gravity_g);
            point.pos.addScaled(point.vel, delta_t);
        }
    }

    for (state.level.joints.items) |joint| {
        const from_point = &state.level.points.items[joint.from];
        const to_point = &state.level.points.items[joint.to];

        const force = (joint.length - joint.actualLength(state.level.points.items)) * state.spring;
        const dir = to_point.pos.minus(from_point.pos).normalized();
        if (to_point.kind == .dinamic)
            to_point.pos.addScaled(dir, force / state.mass * delta_t);
        if (from_point.kind == .dinamic)
            from_point.pos.addScaled(dir, -force / state.mass * delta_t);
    }

    // DRAW
    state.sokol_2d.begin(.{
        .viewport = .{
            .start = .zero,
            .end = .{
                .x = width,
                .y = height,
            },
        },
        .coordinates = .{
            .start = .{
                .x = 0,
                .y = height,
            },
            .end = .{
                .x = width,
                .y = 0,
            },
        },
    });

    for (state.level.joints.items) |joint| {
        const from_point = state.level.points.items[joint.from];
        const to_point = state.level.points.items[joint.to];
        state.sokol_2d.drawLine(from_point.pos, to_point.pos, 10, .yellow);
        state.sokol_2d.drawCircle(from_point.pos, dot_visual_radius, from_point.kind.color(), 30);
        state.sokol_2d.drawCircle(to_point.pos, dot_visual_radius, to_point.kind.color(), 30);
    }

    if (state.prev_point) |point_index| {
        const point = state.level.points.items[point_index];
        state.sokol_2d.drawCircle(point.pos, dot_visual_radius, .blue, 30);
    }

    {
        sokol.gfx.beginPass(.{
            .swapchain = sokol.glue.swapchain(),
        });
        defer sokol.gfx.endPass();

        state.sokol_2d.flush();
    }
    sokol.gfx.commit();

    { // IMGUI
        simgui.newFrame(.{
            .width = sokol.app.width(),
            .height = sokol.app.height(),
            .delta_time = delta_t,
            .dpi_scale = sokol.app.dpiScale(),
        });

        //=== UI CODE STARTS HERE
        ig.igSetNextWindowPos(.{ .x = 10, .y = 10 }, ig.ImGuiCond_Once);
        ig.igSetNextWindowSize(.{ .x = 400, .y = 400 }, ig.ImGuiCond_Once);
        _ = ig.igBegin("Hello Dear ImGui!", 0, ig.ImGuiWindowFlags_None);
        _ = ig.igText("Dear ImGui Version: %s", ig.IMGUI_VERSION);
        _ = ig.igDragFloatEx("Gravity", &state.gravity_g, 0.1, 0.01, 20, "%f", 0);
        _ = ig.igDragFloatEx("Spring", &state.spring, 0.1, 0.01, 5, "%f", 0);
        _ = ig.igDragFloatEx("Mass", &state.mass, 0.1, 0.05, 100, "%f", 0);
        const name_buffer = &state.save_file_name;
        _ = ig.igInputText("Save file name", name_buffer.ptr, name_buffer.len, 0);
        if (ig.igButton("Save file")) {
            try state.level.save(std.mem.span((&state.save_file_name).ptr));
        }
        if (ig.igButton("Load file")) load: {
            const new_level = Level.load(state.gpa, std.mem.span((&state.save_file_name).ptr)) catch break :load;
            state.level.deinit(state.gpa);
            state.level = new_level;
        }
        if (ig.igButton("reset")) {
            state.level.deinit(state.gpa);
        }

        ig.igEnd();
        //=== UI CODE ENDS HERE

        // call simgui.render() inside a sokol-gfx pass
        sokol.gfx.beginPass(.{ .action = state.load_action, .swapchain = sokol.glue.swapchain() });
        simgui.render();
        sokol.gfx.endPass();
        sokol.gfx.commit();
    }
}

fn event(e: [*c]const sokol.app.Event) callconv(.c) void {
    if (simgui.handleEvent(e.*)) return;

    state.input.consumeEvent(e);
    switch (e.*.type) {
        .KEY_DOWN => {
            if (e.*.key_code == .ESCAPE) {
                sokol.app.requestQuit();
            }
        },
        else => {},
    }
}

fn deinit() callconv(.c) void {
    state.level.save(save_file_name) catch |e| {
        std.log.err("Failed to save level :( {s}", .{@errorName(e)});
    };
    state.sokol_2d.deinit(state.gpa);
}

pub fn main() void {
    var gpa_state: std.heap.DebugAllocator(.{}) = .init;
    state.gpa = gpa_state.allocator();

    sokol.app.run(.{
        .init_cb = &init,
        .frame_cb = &frame,
        .cleanup_cb = &deinit,
        .event_cb = &event,
    });
}

fn fatal(e: anyerror) noreturn {
    std.log.err("{s}", .{@errorName(e)});
    std.process.exit(1);
}

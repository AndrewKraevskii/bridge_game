const std = @import("std");
const Sokol2d = @import("sokol_2d");
const sokol = @import("sokol");
const geo = @import("geom");
const Input = @import("Input.zig");
const SlotMap = @import("slot_map").SlotMap;
const simgui = sokol.imgui;
const interp = @import("tween").interp;
const ease = @import("tween").ease;
const ig = @import("imgui");
const Color = Sokol2d.Color;

var state: State = .{};

const save_file_name = "autosave.level";

const Level = struct {
    points: std.ArrayListUnmanaged(Point),
    joints: std.ArrayListUnmanaged(Joint),
    aabbs: std.ArrayListUnmanaged(Sokol2d.AABB),

    pub const empty: Level = .{
        .joints = .empty,
        .points = .empty,
        .aabbs = .empty,
    };

    const Header = extern struct {
        points_len: u32,
        joints_len: u32,
        aabbs_len: u32,
    };

    pub fn save(level: *const Level, file_path: []const u8) !void {
        const file = try std.fs.cwd().createFile(file_path, .{});
        defer file.close();
        var header: Header = .{
            .points_len = @intCast(level.points.items.len),
            .joints_len = @intCast(level.joints.items.len),
            .aabbs_len = @intCast(level.aabbs.items.len),
        };

        const points_bytes = std.mem.sliceAsBytes(level.points.items);
        const joints_bytes = std.mem.sliceAsBytes(level.joints.items);
        const aabbs_bytes = std.mem.sliceAsBytes(level.aabbs.items);
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
            .{
                .base = aabbs_bytes.ptr,
                .len = aabbs_bytes.len,
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
            .aabbs = .empty,
        };

        try level.points.resize(gpa, header.points_len);
        errdefer level.points.deinit(gpa);
        try level.joints.resize(gpa, header.joints_len);
        errdefer level.joints.deinit(gpa);
        try level.aabbs.resize(gpa, header.aabbs_len);
        errdefer level.aabbs.deinit(gpa);

        const points_bytes = std.mem.sliceAsBytes(level.points.items);
        const joints_bytes = std.mem.sliceAsBytes(level.joints.items);
        const aabbs_bytes = std.mem.sliceAsBytes(level.aabbs.items);
        var iovec = [_]std.posix.iovec{
            .{
                .base = points_bytes.ptr,
                .len = points_bytes.len,
            },
            .{
                .base = joints_bytes.ptr,
                .len = joints_bytes.len,
            },
            .{
                .base = aabbs_bytes.ptr,
                .len = aabbs_bytes.len,
            },
        };
        _ = try file.readvAll(&iovec);
        std.log.info("read from {s}", .{file_path});

        return level;
    }

    pub fn deinit(l: *Level, gpa: std.mem.Allocator) void {
        l.joints.deinit(gpa);
        l.points.deinit(gpa);
        l.aabbs.deinit(gpa);
        l.* = undefined;
    }
};

const meters_per_pixel = 1;

const State = struct {
    gpa: std.mem.Allocator = undefined,

    sokol_2d: Sokol2d = undefined,
    load_action: sokol.gfx.PassAction = .{},

    input: Input = .init,

    aabb_starting_point: ?geo.Vec2 = null,
    selected: ?Point.Index = null,
    level: Level = .empty,

    save_file_name: [100:0]u8 = save_file_name.* ++ [1]u8{0} ** 86,
    load_file_name: [100:0]u8 = save_file_name.* ++ [1]u8{0} ** 86,

    brush: enum {
        delete,
        new_points,
        move,
        aabb,
    } = .new_points,
    sim_mode: enum {
        play,
        stop,
    } = .stop,

    friction: f32 = 0.1,
    gravity_g: f32 = 200,
    spring: f32 = 400,
    mass: f32 = 1,
};

const Point = extern struct {
    pos: geo.Vec2,
    vel: geo.Vec2 = .zero,
    kind: enum(u8) {
        dynamic,
        static,
        dead,

        fn color(p: @This()) Sokol2d.Color {
            return switch (p) {
                .dynamic => .red,
                .static => .black,
                else => unreachable,
            };
        }
    } = .dynamic,
    const Index = u32;
};

const Joint = extern struct {
    from: Point.Index,
    to: Point.Index,
    length: f32,

    const dead: Joint = .{
        .from = 0,
        .to = 0,
        .length = 0,
    };

    pub fn actualLength(joint: Joint, points: []const Point) f32 {
        const from_point = &points[joint.from];
        const to_point = &points[joint.to];

        return from_point.pos.dist(to_point.pos);
    }

    pub fn stress(joint: Joint, points: []const Point) f32 {
        const from_point = &points[joint.from];
        const to_point = &points[joint.to];

        return from_point.pos.dist(to_point.pos) - joint.length;
    }

    pub fn isDead(j: Joint) bool {
        return j.from == j.to;
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
        if (point.kind == .dead) continue;
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

fn checkIntersectionPointAABB(point: geo.Vec2, aabb: Sokol2d.AABB) bool {
    const min_x = @min(aabb.start.x, aabb.end.x);
    const max_x = @max(aabb.start.x, aabb.end.x);
    const min_y = @min(aabb.start.y, aabb.end.y);
    const max_y = @max(aabb.start.y, aabb.end.y);

    return point.x > min_x and point.x < max_x and point.y < max_y and point.y > min_y;
}

fn closestPointToAABB(point: geo.Vec2, aabb: Sokol2d.AABB) geo.Vec2 {
    const closest_y = if (@abs(point.y - aabb.start.y) < @abs(point.y - aabb.end.y)) aabb.start.y else aabb.end.y;
    const closest_x = if (@abs(point.x - aabb.start.x) < @abs(point.x - aabb.end.x)) aabb.start.x else aabb.end.x;

    return if (@abs(closest_y - point.y) < @abs(closest_x - point.x)) .{
        .x = point.x,
        .y = closest_y,
    } else .{
        .x = closest_x,
        .y = point.y,
    };
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

    switch (state.brush) {
        .new_points => {
            if (state.input.isMousePressed(.LEFT)) {
                const point_index = getOrAddPoint(state.gpa, &state.level.points, mouse_pos);

                if (state.selected) |prev| {
                    addJoint(state.gpa, &state.level, prev, point_index);

                    state.selected = null;
                } else {
                    state.selected = point_index;
                }
            }
        },
        .move => {
            if (state.input.isMousePressed(.LEFT)) {
                state.selected = findPoint(state.level.points.items, mouse_pos, dot_click_radius);
            }
            if (state.input.isMouseDown(.LEFT)) blk: {
                const point = state.selected orelse break :blk;

                state.level.points.items[point].pos = mouse_pos;
                state.level.points.items[point].vel = .zero;
            }
        },
        .delete => {
            if (state.input.isMousePressed(.LEFT)) delete: {
                const point = findPoint(state.level.points.items, mouse_pos, dot_click_radius) orelse break :delete;
                state.level.points.items[point].kind = .dead;
                for (state.level.joints.items) |*joint| {
                    if (joint.isDead()) continue;
                    if (state.level.points.items[joint.from].kind == .dead or state.level.points.items[joint.to].kind == .dead)
                        joint.* = .dead;
                }
            }
        },
        .aabb => {
            if (state.input.isMousePressed(.LEFT)) {
                state.aabb_starting_point = mouse_pos;
            }
            if (state.input.isMouseReleased(.LEFT)) blk: {
                const point = state.aabb_starting_point orelse break :blk;
                try state.level.aabbs.append(state.gpa, .{
                    .start = point,
                    .end = mouse_pos,
                });
            }
            if (state.input.isMousePressed(.RIGHT)) {
                const aabbs = &state.level.aabbs;
                var i: usize = 0;
                while (i < aabbs.items.len) {
                    if (checkIntersectionPointAABB(mouse_pos, aabbs.items[i])) {
                        _ = aabbs.swapRemove(i);
                    } else {
                        i += 1;
                    }
                }
            }
        },
    }

    if (state.input.isMousePressed(.RIGHT)) blk: {
        const point = findPoint(state.level.points.items, mouse_pos, dot_click_radius) orelse break :blk;

        state.level.points.items[point].kind = switch (state.level.points.items[point].kind) {
            .static => .dynamic,
            .dynamic => .static,
            else => unreachable,
        };
    } else if (state.input.isKeyPressed(.SPACE)) {
        state.sim_mode = switch (state.sim_mode) {
            .play => .stop,
            .stop => .play,
        };
        std.log.info("state {s}", .{@tagName(state.sim_mode)});
    }

    // UPDATE
    if (state.sim_mode == .play) {
        const simulation_steps = 10;
        const step_delta_t = delta_t / simulation_steps;
        for (0..simulation_steps) |_| {
            // apply gravity
            for (state.level.points.items) |*point| {
                if (point.kind == .static) continue;

                point.vel.addScaled(.y_pos, step_delta_t * state.gravity_g);
                point.vel.scale(@exp(-state.friction * delta_t));
            }

            // resolve joints forces
            const points = state.level.points.items;
            for (state.level.joints.items) |joint| {
                if (joint.isDead()) continue;

                const from_point = &points[joint.from];
                const to_point = &points[joint.to];

                const force = -joint.stress(points) * state.spring;
                const dir = to_point.pos.minus(from_point.pos).normalized();
                if (to_point.kind == .dynamic)
                    to_point.vel.addScaled(dir, force / state.mass * step_delta_t);
                if (from_point.kind == .dynamic)
                    from_point.vel.addScaled(dir, -force / state.mass * step_delta_t);
            }

            // resolve move
            for (state.level.points.items) |*point| {
                if (point.kind != .dynamic) continue;
                point.pos.addScaled(point.vel, step_delta_t);
            }

            // collide ground
            for (state.level.aabbs.items) |aabb| {
                for (state.level.points.items) |*point| {
                    if (point.kind != .dynamic) continue;

                    if (checkIntersectionPointAABB(point.pos, aabb)) {
                        const closest_point = closestPointToAABB(point.pos, aabb);
                        const axis = closest_point.minus(point.pos).normalized();
                        point.vel.addScaled(axis, -point.vel.innerProd(axis));
                        point.pos = closest_point;
                    }
                }
            }
        }
    }

    const points = state.level.points.items;

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

    // aabbs

    for (state.level.aabbs.items) |aabb| {
        state.sokol_2d.drawRect(aabb, .dark_gray);
    }

    for (state.level.joints.items) |joint| {
        if (joint.isDead()) continue;

        const from_point = points[joint.from];
        const to_point = points[joint.to];
        state.sokol_2d.drawLine(from_point.pos, to_point.pos, 10, interp.lerp(
            Color.green,
            Color.red,
            1 / (1 + @exp(-joint.stress(points) - 0.5)),
        ));
        state.sokol_2d.drawCircle(from_point.pos, dot_visual_radius, from_point.kind.color(), 30);
        state.sokol_2d.drawCircle(to_point.pos, dot_visual_radius, to_point.kind.color(), 30);
    }

    if (state.selected) |point_index| {
        const point = points[point_index];
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
        _ = ig.igDragFloatEx("Gravity", &state.gravity_g, 0.1, 0.01, 1000, "%f", 0);
        _ = ig.igDragFloatEx("Spring", &state.spring, 0.1, 0.01, 1000, "%f", 0);
        _ = ig.igDragFloatEx("Mass", &state.mass, 0.1, 0.05, 1000, "%f", 0);
        _ = ig.igDragFloatEx("Friction", &state.friction, 0.1, 0.001, 10, "%f", 0);
        _ = ig.igInputText("Save file name", (&state.save_file_name).ptr, state.save_file_name.len, 0);
        _ = ig.igInputText("Load file name", (&state.load_file_name).ptr, state.load_file_name.len, 0);
        if (ig.igButton("Save file")) {
            try state.level.save(std.mem.span((&state.save_file_name).ptr));
        }
        if (ig.igButton("Load file")) load: {
            const new_level = Level.load(state.gpa, std.mem.span((&state.load_file_name).ptr)) catch break :load;
            state.level.deinit(state.gpa);
            state.level = new_level;
            state.selected = null;
        }
        if (ig.igButton("reset")) {
            state.level.deinit(state.gpa);
            state.level = .empty;
            state.selected = null;
        }
        if (ig.igButton(@tagName(state.brush))) {
            state.selected = null;
            state.brush = switch (state.brush) {
                .delete => .new_points,
                .new_points => .move,
                .move => .aabb,
                .aabb => .delete,
            };
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
        .FILES_DROPPED => load_droped_level: {
            const path = sokol.app.getDroppedFilePath(0);
            const new_level = Level.load(state.gpa, path) catch break :load_droped_level;
            state.level.deinit(state.gpa);
            state.level = new_level;
        },
        else => {},
    }
}

fn deinit() callconv(.c) void {
    state.level.save(save_file_name) catch |e| {
        std.log.err("Failed to save level :( {s}", .{@errorName(e)});
    };
    state.level.deinit(state.gpa);
    state.sokol_2d.deinit(state.gpa);
}

pub fn main() void {
    var gpa_state: std.heap.DebugAllocator(.{}) = .init;
    defer _ = gpa_state.deinit();
    state.gpa = gpa_state.allocator();

    sokol.app.run(.{
        .enable_dragndrop = true,
        .max_dropped_files = 1,
        .max_dropped_file_path_length = std.fs.max_path_bytes,
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

const std = @import("std");
const builtin = @import("builtin");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const use_llvm = b.option(bool, "use-llvm", "Use llvm backend");

    const sokol_dep = b.dependency("sokol", .{
        .target = target,
        .optimize = optimize,
        .with_sokol_imgui = true,
    });
    const sokol_2d_dep = b.dependency("sokol_2d", .{
        .target = target,
        .optimize = optimize,
    });
    const geom_dep = b.dependency("geom", .{ .target = target, .optimize = optimize });
    const slot_map_dep = b.dependency("slot_map", .{ .target = target, .optimize = optimize });
    const imgui_dep = b.dependency("imgui", .{
        .target = target,
        .optimize = optimize,
    });
    const tween_dep = b.dependency("tween", .{
        .target = target,
        .optimize = optimize,
    });

    const exe_mod = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "sokol_2d", .module = sokol_2d_dep.module("sokol_2d") },
            .{ .name = "sokol", .module = sokol_dep.module("sokol") },
            .{ .name = "geom", .module = geom_dep.module("geom") },
            .{ .name = "slot_map", .module = slot_map_dep.module("slot_map") },
            .{ .name = "imgui", .module = imgui_dep.module("cimgui") },
            .{ .name = "tween", .module = tween_dep.module("tween") },
        },
    });

    sokol_dep.artifact("sokol_clib").addIncludePath(imgui_dep.path("src"));
    sokol_2d_dep.module("sokol_2d").addImport("sokol", sokol_dep.module("sokol"));
    sokol_2d_dep.module("sokol_2d").addImport("geom", geom_dep.module("geom"));

    const exe = b.addExecutable(.{
        .name = "bridge",
        .root_module = exe_mod,
        .use_llvm = use_llvm,
    });

    b.installArtifact(exe);

    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());

    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);
}

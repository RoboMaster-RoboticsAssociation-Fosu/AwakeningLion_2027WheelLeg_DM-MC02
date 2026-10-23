// Replay the actual local RC, chassis loop, LQR/LESO, VMC and motor packers.
// The host replaces hardware/time only; it does not simulate robot dynamics.
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const { spawnSync } = require("node:child_process");
const root = path.resolve(__dirname, "..");
const temp = fs.mkdtempSync(path.join(os.tmpdir(), "pitch-stop-verify-"));
const owned = [];
function read(name, encoding = "utf-8") {
    return new TextDecoder(encoding, { fatal: true }).decode(fs.readFileSync(path.join(root, name)));
}
function clean(source) {
    return source.replace(/\/\*[\s\S]*?\*\/|\/\/[^\r\n]*/g, "")
        .replace(/^\s*#include[^\r\n]*/gm, "");
}
function block(source, start) {
    let depth = 0;
    for (let i = source.indexOf("{", start); i < source.length; ++i) {
        if (source[i] === "{") ++depth;
        if (source[i] === "}" && --depth === 0) return source.slice(start, i + 1);
    }
    throw new Error("Unbalanced source block");
}
function definition(source, name) {
    const match = new RegExp("(?:static\\s+)?(?:void|float|int|int16_t|uint16_t)\\s+" + name + "\\s*\\([^;{}]*\\)\\s*\\{").exec(source);
    if (!match) throw new Error("Missing production function: " + name);
    return block(source, match.index);
}
function write(name, text) {
    const file = path.join(temp, name);
    fs.writeFileSync(file, text);
    owned.push(file);
}
function run(command, args) {
    const result = spawnSync(command, args, { cwd: root, encoding: "utf8", timeout: 15000 });
    if (result.stdout) process.stdout.write(result.stdout);
    if (result.stderr) process.stderr.write(result.stderr);
    if (result.error) throw result.error;
    if (result.status !== 0) throw new Error(command + " exited with " + result.status);
}
try {
    const headers = [
        ["User/APP/INS_task.h"], ["User/Algorithm/VMC/VMC_calc.h", "gb18030"],
        ["User/Controller/ground_contact.h"], ["User/Devices/DM_Motor/DM_Motor.h"],
        ["User/Devices/DJI_Motor/DJI_Motor.h"], ["User/APP/chassis_task.h", "gb18030"],
        ["User/Lib/user_lib.h"], ["User/Algorithm/LQR/LQR.h"],
        ["User/Algorithm/some_config/some_para.h", "gb18030"],
        ["User/Algorithm/LESO/LESO.h", "gb18030"],
        ["User/Devices/Remote_Control/Remote_Control.h"], ["User/APP/Remote_task.h"],
        ["User/Controller/leg_motion.h"], ["User/Controller/chassis_recovery.h"],
        ["User/Config/user_sys_config.h"],
    ].map(([name, encoding]) => clean(read(name, encoding)));
    write("pitch_stop_api.inc", headers.join("\n"));
    const chassis = clean(read("User/APP/chassis_task.c", "gb18030"));
    const task = definition(chassis, "chassis_task");
    const loop = /while\s*\(1\)\s*\{/.exec(task);
    if (!loop) throw new Error("Missing real chassis loop");
    const cycle = "static void chassis_cycle(void) { uint32_t PreviousWakeTime = HAL_GetTick();\n" +
        block(task, task.indexOf("{", loop.index)).slice(1, -1) + "\n}";
    const motor = clean(read("User/Devices/DM_Motor/DM_Motor.c"));
    const lib = clean(read("User/Lib/user_lib.c"));
    write("pitch_stop_implementation.inc", [
        clean(read("User/Algorithm/some_config/some_para.c")),
        ...["ramp_init", "ramp_calc", "int16_constrain"].map(name => definition(lib, name)),
        ...["float_to_uint", "uint_to_float", "DM_Enable_Motor", "DM_Motor_Ctrl"].map(name => definition(motor, name)),
        clean(read("User/Devices/DJI_Motor/DJI_Motor.c")),
        clean(read("User/Algorithm/VMC/VMC_calc.c", "gb18030")),
        clean(read("User/Algorithm/LQR/LQR.c")),
        clean(read("User/Algorithm/LESO/LESO.c")),
        clean(read("User/Controller/ground_contact.c")),
        chassis,
        clean(read("User/Devices/Remote_Control/Remote_Control.c")),
        clean(read("User/APP/Remote_task.c")),
        cycle,
    ].join("\n"));
    const exe = path.join(temp, "pitch_stop.exe");
    owned.push(exe);
    run(process.env.CC || "gcc", ["-std=c99", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
        "-Wno-missing-braces", "-O0", "-finput-charset=UTF-8", "-I", temp,
        path.join(__dirname, "pitch_stop_verify.c"),
        path.join(root, "User/Controller/leg_motion.c"), path.join(root, "User/Controller/chassis_recovery.c"),
        "-I", path.join(root, "User/Controller"), "-lm", "-o", exe]);
    run(exe, process.argv.slice(2));
} finally {
    for (const file of owned) if (fs.existsSync(file)) fs.unlinkSync(file);
    fs.rmdirSync(temp);
}

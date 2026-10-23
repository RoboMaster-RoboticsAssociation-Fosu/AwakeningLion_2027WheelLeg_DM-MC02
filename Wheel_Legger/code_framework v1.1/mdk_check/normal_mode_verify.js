// Compile real NORMAL state machine, task ordering, feedback integration and LQR.
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const { spawnSync } = require("node:child_process");
const root = path.resolve(__dirname, "..");
const temp = fs.mkdtempSync(path.join(os.tmpdir(), "normal-mode-verify-"));
const owned = [];
function read(name, encoding = "utf-8") {
    return new TextDecoder(encoding, { fatal: true }).decode(fs.readFileSync(path.join(root, name)));
}
function stripIncludes(source) { return source.replace(/^\s*#include[^\r\n]*/gm, ""); }
function stripComments(source) { return source.replace(/\/\*[\s\S]*?\*\/|\/\/[^\r\n]*/g, ""); }
function write(name, value) {
    const file = path.join(temp, name);
    if (!owned.includes(file)) owned.push(file);
    fs.writeFileSync(file, value, "utf8");
}
function definition(source, name) {
    const clean = stripComments(source);
    const match = new RegExp("(?:static\\s+)?(?:void|float)\\s+" + name + "\\s*\\([^;{}]*\\)\\s*\\{").exec(clean);
    if (!match) throw new Error("Missing production function: " + name);
    let depth = 0;
    for (let i = clean.indexOf("{", match.index); i < clean.length; ++i) {
        if (clean[i] === "{") ++depth;
        if (clean[i] === "}" && --depth === 0) return clean.slice(match.index, i + 1);
    }
    throw new Error("Unbalanced function: " + name);
}
function run(command, args) {
    const result = spawnSync(command, args, { cwd: root, encoding: "utf8" });
    if (result.stdout) process.stdout.write(result.stdout);
    if (result.stderr) process.stderr.write(result.stderr);
    if (result.error) throw result.error;
    if (result.status !== 0) throw new Error(command + " exited with " + result.status);
}
try {
    const headers = [
        ["User/APP/INS_task.h", "utf-8"], ["User/Algorithm/VMC/VMC_calc.h", "gb18030"],
        ["User/Controller/ground_contact.h", "utf-8"], ["User/Devices/DM_Motor/DM_Motor.h", "utf-8"],
        ["User/Devices/DJI_Motor/DJI_Motor.h", "utf-8"], ["User/APP/chassis_task.h", "gb18030"],
        ["User/Lib/user_lib.h", "utf-8"], ["User/Algorithm/LQR/LQR.h", "utf-8"],
        ["User/Algorithm/some_config/some_para.h", "gb18030"],
    ].map(([name, encoding]) => stripIncludes(stripComments(read(name, encoding))));
    const api = headers.join("\n");
    const chassis = read("User/APP/chassis_task.c", "gb18030");
    const lib = read("User/Lib/user_lib.c");
    const math = read("User/Algorithm/some_config/some_para.c");
    const declarations = [...chassis.matchAll(/^static (?:ramp_function_source_t \w+(?:\[\d+\])?|uint8_t (?:leg_length_ramp_initialized|normal_ramp_initialized));/gm)].map(m => m[0]);
    if (declarations.length !== 5) throw new Error("Missing production ramp state");
    write("normal_implementation.inc", [
        ...declarations,
        definition(math, "mySaturate"), definition(math, "Find_Min_RADIAN"),
        definition(lib, "ramp_init"), definition(lib, "ramp_calc"),
        stripIncludes(read("User/Algorithm/LQR/LQR.c")),
        ...["chassis_zero_outputs", "normal_state_reset", "normal_state_update", "normal_target_ramp_update",
            "normal_move_target_update", "normal_spin_target_update", "YAW_Parameter_Processing", "LQR",
            "chassis_leg_length_reference_update", "LEG_Lenth_Control", "normal_mode",
            "chassis_feedback_update", "chassis_task"].map(name => definition(chassis, name)),
    ].join("\n"));
    write("normal_api.inc", api);
    const exe = path.join(temp, "normal" + (process.platform === "win32" ? ".exe" : ""));
    owned.push(exe);
    run(process.env.CC || "gcc", ["-std=c99", "-Wall", "-Wextra", "-Werror", "-Wno-missing-braces",
        "-O0", "-finput-charset=UTF-8", "-I", temp, path.join(__dirname, "normal_mode_verify.c"), "-lm", "-o", exe]);
    run(exe, ["3"]);
    run(exe, ["-3"]);
} finally {
    for (const file of owned) if (fs.existsSync(file)) fs.unlinkSync(file);
    fs.rmdirSync(temp);
}

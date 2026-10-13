// 编译实际检测、VMC、LQR、LESO及底盘接入；仅替换硬件/RTOS依赖。
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const { spawnSync } = require("node:child_process");
const root = path.resolve(__dirname, "..");
const temp = fs.mkdtempSync(path.join(os.tmpdir(), "ground-contact-"));
const owned = [];
function read(name, encoding = "utf-8") {
    return new TextDecoder(encoding, { fatal: true }).decode(fs.readFileSync(path.join(root, name)));
}
function stripIncludes(source) { return source.replace(/^\s*#include[^\r\n]*/gm, ""); }
function stripComments(source) { return source.replace(/\/\*[\s\S]*?\*\/|\/\/[^\r\n]*/g, ""); }
function write(name, value) {
    const file = path.join(temp, name); owned.push(file); fs.writeFileSync(file, value, "utf8");
}
function definition(source, name) {
    const clean = stripComments(source);
    const match = new RegExp("(?:static\\s+)?(?:void|float|uint8_t)\\s+" + name + "\\s*\\([^;{}]*\\)\\s*\\{").exec(clean);
    if (!match) throw new Error("Missing production function: " + name);
    let depth = 0;
    for (let i = clean.indexOf("{", match.index); i < clean.length; ++i) {
        if (clean[i] === "{") ++depth;
        if (clean[i] === "}" && --depth === 0) return clean.slice(match.index, i + 1);
    }
    throw new Error("Unbalanced function: " + name);
}
function run(command, args) {
    const r = spawnSync(command, args, { cwd: root, encoding: "utf8" });
    if (r.stdout) process.stdout.write(r.stdout);
    if (r.stderr) process.stderr.write(r.stderr);
    if (r.error) throw r.error;
    if (r.status !== 0) throw new Error(command + " exited with " + r.status);
}
try {
    const headers = [
        ["User/APP/INS_task.h", "utf-8"], ["User/Algorithm/VMC/VMC_calc.h", "gb18030"],
        ["User/Controller/ground_contact.h", "utf-8"], ["User/Devices/DM_Motor/DM_Motor.h", "utf-8"],
        ["User/Devices/DJI_Motor/DJI_Motor.h", "utf-8"], ["User/APP/chassis_task.h", "gb18030"],
        ["User/Lib/user_lib.h", "utf-8"], ["User/Algorithm/LQR/LQR.h", "utf-8"],
        ["User/Algorithm/some_config/some_para.h", "gb18030"]
    ].map(([name, enc]) => stripIncludes(stripComments(read(name, enc))));
    // LESO 头文件有历史混合编码注释，测试只保留原始 ASCII C token。
    const lesoHeader = stripIncludes(stripComments(fs.readFileSync(path.join(root, "User/Algorithm/LESO/LESO.h")).toString("latin1")));
    write("ground_api.inc", headers.join("\n") + "\n" + lesoHeader);
    const chassis = read("User/APP/chassis_task.c", "gb18030");
    const lib = read("User/Lib/user_lib.c");
    const math = read("User/Algorithm/some_config/some_para.c");
    const declarations = [...chassis.matchAll(/^static volatile (?:uint32_t|uint8_t) motor_feedback_\w+\[[^\]]+\];/gm)].map(m => m[0]);
    declarations.push(...[...chassis.matchAll(/^static (?:ramp_function_source_t leg_length_ramp\[2\]|uint8_t leg_length_ramp_initialized);/gm)].map(m => m[0]));
    write("ground_implementation.inc", [
        ...declarations, definition(math, "mySaturate"), definition(math, "Find_Min_RADIAN"),
        definition(lib, "ramp_init"), definition(lib, "ramp_calc"),
        stripIncludes(read("User/Algorithm/VMC/VMC_calc.c", "gb18030")),
        stripIncludes(read("User/Controller/ground_contact.c")),
        stripIncludes(read("User/Algorithm/LQR/LQR.c")),
        stripIncludes(read("User/Algorithm/LESO/LESO.c")),
        definition(chassis, "DM_Enable"), definition(chassis, "Chassis_MotorFeedbackReceived"),
        definition(chassis, "chassis_motor_online_update"), definition(chassis, "chassis_joint_enable_check"),
        definition(chassis, "chassis_feedback_update"), definition(chassis, "chassis_leg_length_reference_update"),
        definition(chassis, "LEG_Lenth_Control"), definition(chassis, "Chassis_CanTransimit")
    ].join("\n"));
    const baselineDir = process.argv[2];
    if (baselineDir) {
        const original = new TextDecoder("gb18030", { fatal: true }).decode(fs.readFileSync(path.join(baselineDir, "User/APP/chassis_task.c")));
        const lqr = fs.readFileSync(path.join(baselineDir, "User/Algorithm/LQR/LQR.c"), "utf8");
        write("ground_baseline.inc", definition(original, "LEG_Lenth_Control").replace(/\bLEG_Lenth_Control\b/g, "Baseline_LEG_Lenth_Control") + "\n" +
            stripIncludes(lqr).replace(/\b(P|u|Fitting_K|Fitting_K_Calc|LQR_Calc)\b/g, m => "Baseline_" + m));
    }
    const exe = path.join(temp, "ground" + (process.platform === "win32" ? ".exe" : "")); owned.push(exe);
    run(process.env.CC || "gcc", ["-std=c99", "-Wall", "-Wextra", "-Werror", "-Wno-missing-braces", "-O0", "-finput-charset=UTF-8",
        ...(baselineDir ? ["-DVERIFY_GROUND_BASELINE"] : []), "-I", temp,
        path.join(__dirname, "ground_contact_verify.c"), "-lm", "-o", exe]);
    run(exe, []);
} finally {
    for (const file of owned) if (fs.existsSync(file)) fs.unlinkSync(file);
    fs.rmdirSync(temp);
}

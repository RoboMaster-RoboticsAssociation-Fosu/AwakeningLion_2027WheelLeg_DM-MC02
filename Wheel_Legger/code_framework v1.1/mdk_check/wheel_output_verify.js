// Exercise real LQR, chassis transmission, DJI current type and CAN packing.
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const { spawnSync } = require("node:child_process");
const root = path.resolve(__dirname, "..");
const temp = fs.mkdtempSync(path.join(os.tmpdir(), "wheel-output-verify-"));
const compiler = process.env.CC || "gcc";
const ownedFiles = [];
function read(file, encoding = "utf-8") {
    return new TextDecoder(encoding, { fatal: true }).decode(fs.readFileSync(file));
}
function definition(source, name) {
    const match = new RegExp("(?:void|float)\\s+" + name + "\\s*\\([^;{}]*\\)\\s*\\{").exec(source);
    if (!match) throw new Error("Missing function: " + name);
    let depth = 0;
    for (let i = source.indexOf("{", match.index); i < source.length; ++i) {
        if (source[i] === "{") ++depth;
        if (source[i] === "}" && --depth === 0) return source.slice(match.index, i + 1);
    }
    throw new Error("Unbalanced function: " + name);
}
function write(name, value) {
    const file = path.join(temp, name);
    ownedFiles.push(file);
    fs.writeFileSync(file, value);
}
function run(program, args) {
    const result = spawnSync(program, args, { cwd: root, encoding: "utf8" });
    if (result.stdout) process.stdout.write(result.stdout);
    if (result.stderr) process.stderr.write(result.stderr);
    if (result.error) throw result.error;
    if (result.status !== 0) throw new Error(program + " exited with status " + result.status);
}
try {
    // Optional baseline chassis source isolates the transmission-order change.
    const chassisPath = process.argv[2] || path.join(root, "User/APP/chassis_task.c");
    const chassis = read(chassisPath, "gb18030");
    const header = read(path.join(root, "User/APP/chassis_task.h"), "gb18030");
    const driver = read(path.join(root, "User/Devices/DJI_Motor/DJI_Motor.c"));
    const math = read(path.join(root, "User/Algorithm/some_config/some_para.c"));
    const macros = ["LeftWheelT_TO_Current", "RightWheelT_TO_Current"];
    const definitions = macros.map(name => {
        const line = header.match(new RegExp("^#define\\s+" + name + "\\s+[^\\r\\n]+", "m"));
        if (!line) throw new Error("Missing macro: " + name);
        return line[0];
    });
    for (const name of ["Chassis_Wheel_e", "Chassis_Joint_ID_e"]) {
        const enums = [...header.matchAll(/typedef\s+enum\s*\{[^}]*\}\s*\w+\s*;/g)];
        const declaration = enums.find(match => match[0].endsWith(name + ";"));
        if (!declaration) throw new Error("Missing enum: " + name);
        definitions.push(declaration[0]);
    }
    for (const macro of header.matchAll(/^#define\s+JOINT_DM_\w+_TxID_Set\s+[^\r\n]+/gm)) definitions.unshift(macro[0]);
    const limits = read(path.join(root, "User/Lib/user_lib.h"));
    const limitLines = limits.split(/\r?\n/);
    const limitStart = limitLines.findIndex(line => line.startsWith("#define VAL_LIMIT("));
    if (limitStart < 0) throw new Error("Missing VAL_LIMIT macro");
    for (let i = limitStart; i < limitLines.length; ++i) {
        definitions.push(limitLines[i]);
        if (!limitLines[i].trimEnd().endsWith("\\")) break;
    }
    const lqrHeader = read(path.join(root, "User/Algorithm/LQR/LQR.h"));
    definitions.push(...[...lqrHeader.matchAll(/^#define\s+\w+_MAX\s+[^\r\n]+/gm)].map(m => m[0]));
    write("wheel_definitions.inc", definitions.join("\n"));
    write("stm32h723xx.h", "#include <stdint.h>\n");
    write("bsp_can.h", "typedef int hcan_t;\n");
    write("pid.h", "typedef int Pid_Set_Typedef;\ntypedef int PidTypedef;\n");
    write("wheel_implementation.inc", [
        definition(math, "mySaturate"), definition(math, "Find_Min_RADIAN"),
        read(path.join(root, "User/Algorithm/LQR/LQR.c")).replace(/^#include[^\r\n]*/gm, ""),
        definition(driver, "DJI_Motor_ctrl"), definition(chassis, "Chassis_CanTransimit"),
    ].join("\n"));
    const executable = path.join(temp, "wheel" + (process.platform === "win32" ? ".exe" : ""));
    ownedFiles.push(executable);
    run(compiler, ["-std=c99", "-Wall", "-Wextra", "-Werror", "-Wno-missing-braces", "-O0", "-finput-charset=UTF-8",
        "-I", temp, "-I", path.join(root, "User/Devices/DJI_Motor"),
        path.join(__dirname, "wheel_output_verify.c"), "-lm", "-o", executable]);
    run(executable, []);
} finally {
    for (const file of ownedFiles) if (fs.existsSync(file)) fs.unlinkSync(file);
    fs.rmdirSync(temp);
}

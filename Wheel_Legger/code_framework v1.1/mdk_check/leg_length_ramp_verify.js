// Compile the real ramp, balance PD and complete chassis task loop with host stubs.
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const { spawnSync } = require("node:child_process");
const root = path.resolve(__dirname, "..");
const temp = fs.mkdtempSync(path.join(os.tmpdir(), "leg-length-ramp-"));
const ownedFiles = [];

function read(file, encoding = "utf-8") {
    return new TextDecoder(encoding, { fatal: true }).decode(fs.readFileSync(path.join(root, file)));
}
function write(name, text) {
    const file = path.join(temp, name);
    ownedFiles.push(file);
    fs.writeFileSync(file, text, "utf8");
}
function definition(source, name) {
    const match = new RegExp("(?:static\\s+)?(?:void|float)\\s+" + name + "\\s*\\([^;{}]*\\)\\s*\\{").exec(source);
    if (!match) throw new Error("Missing function: " + name);
    let depth = 0;
    for (let i = source.indexOf("{", match.index); i < source.length; ++i) {
        if (source[i] === "{") ++depth;
        if (source[i] === "}" && --depth === 0) return source.slice(match.index, i + 1);
    }
    throw new Error("Unbalanced function: " + name);
}
function requiredMatch(source, pattern) {
    const result = source.match(pattern);
    if (!result) throw new Error("Missing source declaration: " + pattern);
    return result[0];
}
function run(command, args) {
    const result = spawnSync(command, args, { cwd: root, encoding: "utf8" });
    if (result.stdout) process.stdout.write(result.stdout);
    if (result.stderr) process.stderr.write(result.stderr);
    if (result.error) throw result.error;
    if (result.status !== 0) throw new Error(command + " exited with " + result.status);
}

try {
    const chassis = read("User/APP/chassis_task.c", "gb18030");
    const header = read("User/APP/chassis_task.h", "gb18030");
    const lib = read("User/Lib/user_lib.c");
    const libHeader = read("User/Lib/user_lib.h");
    const math = read("User/Algorithm/some_config/some_para.c");
    const macros = ["LEG_LENGTH_RAMP_RATE_M_S", "LEG_PID_KP", "LEG_PID_KD_RATE", "LEG_PID_MAX_OUT",
        "ROLL_PID_KP", "ROLL_PID_KD", "ROLL_PID_MAX_OUT", "body_mg"].map(name =>
        requiredMatch(header, new RegExp("^#define\\s+" + name + "\\s+[^\\r\\n]+", "m")));
    write("ramp_definitions.inc", [
        ...macros,
        requiredMatch(chassis, /^#define\s+Chassis_Time\s+[^\r\n]+/m),
        requiredMatch(libHeader, /typedef\s+struct\s*\{[^}]*\}\s*ramp_function_source_t\s*;/),
        requiredMatch(chassis, /static ramp_function_source_t leg_length_ramp\[2\];/),
        requiredMatch(chassis, /static uint8_t leg_length_ramp_initialized;/),
    ].join("\n"));
    write("ramp_initial_goal.inc", requiredMatch(chassis, /\.set_goal\s*=\s*\{[^}]*\},/));
    write("ramp_implementation.inc", [
        definition(lib, "ramp_init"), definition(lib, "ramp_calc"), definition(math, "mySaturate"),
        definition(chassis, "chassis_leg_length_reference_update"), definition(chassis, "LEG_Lenth_Control"),
        definition(chassis, "normal_mode"), definition(chassis, "chassis_task"),
    ].join("\n"));
    const executable = path.join(temp, "ramp" + (process.platform === "win32" ? ".exe" : ""));
    ownedFiles.push(executable);
    run(process.env.CC || "gcc", ["-std=c99", "-Wall", "-Wextra", "-Werror", "-pedantic", "-O0",
        "-finput-charset=UTF-8", "-I", temp, path.join(__dirname, "leg_length_ramp_verify.c"), "-lm", "-o", executable]);
    run(executable, []);
} finally {
    for (const file of ownedFiles) if (fs.existsSync(file)) fs.unlinkSync(file);
    fs.rmdirSync(temp);
}

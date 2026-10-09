// Compile both real C99 controllers and the current chassis integration functions.
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const { spawnSync } = require("node:child_process");

const root = path.resolve(__dirname, "..");
const temp = fs.mkdtempSync(path.join(os.tmpdir(), "leg-motion-verify-"));
const compiler = process.env.CC || "gcc";
const ownedFiles = [];

function run(program, args) {
    const result = spawnSync(program, args, { cwd: root, encoding: "utf8" });
    if (result.stdout) process.stdout.write(result.stdout);
    if (result.stderr) process.stderr.write(result.stderr);
    if (result.error) throw result.error;
    if (result.status !== 0) {
        throw new Error(program + " exited with status " + result.status);
    }
}

function compileAndRun(name, harness, sourceDir = path.join(root, "User", "Controller")) {
    const executable = path.join(temp, name + (process.platform === "win32" ? ".exe" : ""));
    ownedFiles.push(executable);
    run(compiler, [
        "-std=c99", "-Wall", "-Wextra", "-Werror", "-pedantic", "-O0",
        "-finput-charset=UTF-8",
        "-I", sourceDir, "-I", temp,
        path.join(sourceDir, "leg_motion.c"),
        path.join(sourceDir, "chassis_recovery.c"),
        path.join(__dirname, harness),
        "-lm", "-o", executable,
    ]);
    run(executable, []);
}

// Decode GBK source strictly; generated test excerpts use UTF-8 comments.
// This compiles today's implementation, not a separately maintained copy.
function recoverySource() {
    const source = new TextDecoder("gb18030", { fatal: true }).decode(
        fs.readFileSync(path.join(root, "User", "APP", "chassis_task.c")));
    function between(first, last) {
        const begin = source.indexOf(first);
        const end = source.indexOf(last, begin);
        if (begin < 0 || end < 0) throw new Error("Recovery source markers changed");
        return source.slice(begin, end);
    }
    function definition(name) {
        const begin = source.indexOf("void " + name + "(void)\r\n{");
        if (begin < 0) throw new Error("Missing recovery function: " + name);
        let depth = 0;
        for (let i = source.indexOf("{", begin); i < source.length; i++) {
            if (source[i] === "{") depth++;
            if (source[i] === "}" && --depth === 0) return source.slice(begin, i + 1);
        }
        throw new Error("Unbalanced recovery function: " + name);
    }
    const recoveryBegin = source.indexOf("static void chassis_zero_outputs(void)\r\n{");
    if (recoveryBegin < 0) throw new Error("Missing recovery output helpers");
    return [
        between("ChassisRecovery_Context chassis_recovery;", "extern INS_t INS;"),
        definition("VMC_translate"),
        definition("falling_down_detect"),
        source.slice(recoveryBegin),
    ].join("\n");
}

try {
    compileAndRun("motion", "leg_motion_verify.c");
    compileAndRun("recovery_module", "chassis_recovery_verify.c");
    const excerpt = path.join(temp, "chassis_recovery_under_test.inc");
    ownedFiles.push(excerpt);
    fs.writeFileSync(excerpt, recoverySource(), "utf8");
    compileAndRun("recovery", "recovery_verify.c");
    compileAndRun("sweep_ramps", "sweep_ramp_verify.c");
    // Compile copies of the actual modules with only the two ramp macros changed.
    // This verifies unequal configurations without editing production parameters.
    const controllerDir = path.join(root, "User", "Controller");
    for (const name of ["leg_motion.c", "leg_motion.h", "chassis_recovery.c", "chassis_recovery.h"]) {
        const destination = path.join(temp, name);
        ownedFiles.push(destination);
        fs.copyFileSync(path.join(controllerDir, name), destination);
    }
    const header = fs.readFileSync(path.join(controllerDir, "chassis_recovery.h"), "utf8");
    const times = [[4000, 8000], [8000, 4000], [0, 4000], [4000, 0]];
    for (const [angleTime, lengthTime] of times) {
        let configured = header;
        for (const [macro, value] of [["SPIN_SWING_ANGLE_TIME_MS", angleTime],
                                      ["SPIN_SWING_LENGTH_TIME_MS", lengthTime]]) {
            const pattern = new RegExp("^#define " + macro + " \\d+U", "gm");
            if ([...configured.matchAll(pattern)].length !== 1) throw new Error("Missing ramp macro: " + macro);
            configured = configured.replace(pattern, "#define " + macro + " " + value + "U");
        }
        fs.writeFileSync(path.join(temp, "chassis_recovery.h"), configured, "utf8");
        compileAndRun("sweep_ramps_" + angleTime + "_" + lengthTime, "sweep_ramp_verify.c", temp);
    }
} finally {
    // Remove only files created by this run, then the empty owned directory.
    for (const file of ownedFiles) {
        if (fs.existsSync(file)) fs.unlinkSync(file);
    }
    fs.rmdirSync(temp);
}

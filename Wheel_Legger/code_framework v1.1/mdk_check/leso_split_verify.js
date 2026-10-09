// Compile the real LESO module; optional original source enables differential checks.
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const { spawnSync } = require("node:child_process");
const root = path.resolve(__dirname, "..");
const temp = fs.mkdtempSync(path.join(os.tmpdir(), "leso-split-"));
const owned = [];

function write(name, value) {
    const file = path.join(temp, name);
    owned.push(file);
    fs.writeFileSync(file, value, "utf8");
}
function stripIncludes(source) {
    return source.replace(/^\s*#include[^\r\n]*/gm, "");
}
function run(command, args) {
    const result = spawnSync(command, args, { cwd: root, encoding: "utf8" });
    if (result.stdout) process.stdout.write(result.stdout);
    if (result.stderr) process.stderr.write(result.stderr);
    if (result.error) throw result.error;
    if (result.status !== 0) throw new Error(command + " exited with " + result.status);
}
try {
    // The header has historical mixed comment encodings; preserve all ASCII C tokens.
    const header = fs.readFileSync(path.join(root, "User/Algorithm/LESO/LESO.h"))
        .toString("latin1").replace(/\/\*[\s\S]*?\*\/|\/\/[^\r\n]*/g, "");
    if (/[^\x00-\x7f]/.test(header)) throw new Error("Non-ASCII header code token");
    const chassisHeader = new TextDecoder("gb18030", { fatal: true }).decode(
        fs.readFileSync(path.join(root, "User/APP/chassis_task.h")));
    const enums = [...chassisHeader.matchAll(/typedef\s+enum\s*\{[^}]*\}\s*\w+\s*;/g)];
    const mappings = ["Chassis_Joint_VMC_e", "Chassis_Wheel_e"].map(name => {
        const declaration = enums.find(match => match[0].endsWith(name + ";"));
        if (!declaration) throw new Error("Missing enum: " + name);
        return declaration[0];
    });
    write("leso_api.inc", stripIncludes(header) + "\n" + mappings.join("\n"));
    const source = new TextDecoder("utf-8", { fatal: true }).decode(
        fs.readFileSync(path.join(root, "User/Algorithm/LESO/LESO.c")));
    const call = "LESO_Update(&leso, Ad_f, Bd_f, L_f, y, u_last, leso_dlim);";
    if (source.split(call).length !== 2) throw new Error("Missing unique update seam");
    write("leso_implementation.inc", stripIncludes(source).replace(call,
        call.replace("LESO_Update(", "verify_LESO_Update(")));
    const flags = [];
    if (process.argv[2]) {
        let legacy = new TextDecoder("utf-8", { fatal: true }).decode(fs.readFileSync(process.argv[2]));
        const splitBaseline = legacy.includes("float leso_wheel_scale");
        if ((!legacy.includes("float leso_comp_scale") && !splitBaseline) || legacy.includes("leso_wheel_mode")) {
            throw new Error("Baseline must be unified-scale or the preceding wheel/hip split LESO.c");
        }
        if (splitBaseline) flags.push("-DVERIFY_LEGACY_SPLIT");
        const names = ["AdP", "BdP", "LP", "Ad_f", "Bd_f", "L_f", "leso", "u_last",
            "stable_t", "comp_state", "online_last", "leso_dbg_dh", "leso_dbg_emax",
            "leso_dbg_comp", "leso_active", "leso_comp_scale", "leso_dlim", "lesio_clamp",
            "lesio_valid", "LESO_Init", "LESO_Seed", "LESO_Update", "lesio_mat_calc",
            "LESO_Service", "LESO_Feedback", "Chassis", "INS", "leso_wheel_scale", "leso_leg_scale",
            "comp_wheel_state", "comp_leg_state", "leso_dbg_comp_wheel", "leso_dbg_comp_leg",
            "lesio_comp_target", "lesio_comp_step"];
        legacy = stripIncludes(legacy).replace(new RegExp("\\b(" + names.join("|") + ")\\b", "g"),
            name => "legacy_" + name);
        write("leso_legacy.inc", legacy);
        flags.push("-DVERIFY_LEGACY");
    }
    const executable = path.join(temp, "leso" + (process.platform === "win32" ? ".exe" : ""));
    owned.push(executable);
    // Existing coefficient tables use flattened initializers, as in the wheel check.
    run(process.env.CC || "gcc", ["-std=c99", "-Wall", "-Wextra", "-Werror", "-Wno-missing-braces", "-O0",
        "-finput-charset=UTF-8", ...flags, "-I", temp,
        path.join(__dirname, "leso_split_verify.c"), "-lm", "-o", executable]);
    run(executable, []);
} finally {
    for (const file of owned) if (fs.existsSync(file)) fs.unlinkSync(file);
    fs.rmdirSync(temp);
}

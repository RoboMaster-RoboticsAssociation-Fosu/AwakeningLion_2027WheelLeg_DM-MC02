// Compile the production input modules and CAN callback with hardware stubs.
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const { spawnSync } = require("node:child_process");
const root = path.resolve(__dirname, "..");
const temp = fs.mkdtempSync(path.join(os.tmpdir(), "remote-source-verify-"));
const ownedFiles = [];
const compiler = process.env.CC || "gcc";
function write(name, contents) {
    const file = path.join(temp, name);
    ownedFiles.push(file);
    fs.writeFileSync(file, contents);
}
function read(name, encoding = "utf-8") {
    return new TextDecoder(encoding, { fatal: true }).decode(fs.readFileSync(path.join(root, name)));
}
function run(program, args) {
    const result = spawnSync(program, args, { cwd: root, encoding: "utf8" });
    if (result.stdout) process.stdout.write(result.stdout);
    if (result.stderr) process.stderr.write(result.stderr);
    if (result.error) throw result.error;
    if (result.status !== 0) throw new Error(program + " exited with status " + result.status);
    return result;
}
function definition(source, name) {
    const match = new RegExp("void\\s+" + name + "\\s*\\([^;{}]*\\)\\s*\\{").exec(source);
    if (!match) throw new Error("Missing real callback: " + name);
    let depth = 0;
    for (let i = source.indexOf("{", match.index); i < source.length; ++i) {
        if (source[i] === "{") ++depth;
        if (source[i] === "}" && --depth === 0) return source.slice(match.index, i + 1);
    }
    throw new Error("Unbalanced callback");
}
try {
    for (const file of ["User/APP/Remote_task.c", "User/APP/Remote_task.h", "User/APP/board_comm_task.c",
                       "User/APP/board_comm_task.h", "User/Devices/Remote_Control/Remote_Control.c",
                       "User/Devices/Remote_Control/Remote_Control.h", "User/Config/user_sys_config.h"]) {
        write(path.basename(file), read(file));
    }
    write("main.h", read("mdk_check/remote_source_platform.h"));
    write("cmsis_os.h", '#include "main.h"\n');
    write("stm32h7xx.h", '#include "main.h"\n');
    write("INS_task.h", '#include "main.h"\n');
    const chassisHeader = read("User/APP/chassis_task.h", "gb18030");
    const motorEnum = chassisHeader.match(/typedef\s+enum\s*\{[^}]*\}\s*Chassis_MotorFeedback_e\s*;/);
    if (!motorEnum) throw new Error("Missing motor feedback enum");
    write("chassis_task.h", '#include "main.h"\n' + motorEnum[0] + "\n");
    write("can_callback.c", '#include "main.h"\n#include "chassis_task.h"\n#include "board_comm_task.h"\n#include <string.h>\n' +
        "void Chassis_MotorFeedbackReceived(Chassis_MotorFeedback_e motor) { (void)motor; }\n" +
        definition(read("User/Bsp/bsp_can.c"), "HAL_FDCAN_RxFifo0Callback") + "\n");
    const common = ["-std=c99", "-Wall", "-Wextra", "-Werror", "-O0", "-finput-charset=UTF-8", "-I", temp,
        path.join(temp, "Remote_task.c"), path.join(temp, "Remote_Control.c"),
        path.join(temp, "board_comm_task.c"), path.join(temp, "can_callback.c"),
        path.join(__dirname, "remote_source_verify.c"), "-lm"];
    for (const mode of ["local", "gimbal"]) {
        const exe = path.join(temp, mode + (process.platform === "win32" ? ".exe" : ""));
        ownedFiles.push(exe);
        const expectedSource = mode === "gimbal" ? 1 : 0;
        const flags = ["-DCHASSIS_CTRL_SOURCE=" + expectedSource];
        run(compiler, [...common, ...flags, "-o", exe]);
        for (const scenario of ["protocol", "mapping", "timeout", "missing_sticks", "missing_status",
                                "lost", "burst", "overflow", "startup_status", "wrap", "fifo", "stream", "snapshot", "leg_length"]) {
            const result = run(exe, [scenario]);
            if (!result.stdout.startsWith("remote source " + expectedSource + " ")) {
                throw new Error("Wrong compiled control source for " + mode);
            }
        }
    }
    for (const selector of ["99", "MISSPELLED_SOURCE"]) {
        const invalid = spawnSync(compiler, ["-std=c99", "-I", temp, "-DCHASSIS_CTRL_SOURCE=" + selector, "-fsyntax-only",
            path.join(temp, "Remote_task.c")], { encoding: "utf8" });
        if (invalid.error) throw invalid.error;
        if (invalid.status === 0 || (selector === "99" && !invalid.stderr.includes("Invalid CHASSIS_CTRL_SOURCE"))) {
            throw new Error("Illegal source selection must fail at compile time: " + selector);
        }
    }
    const rtos = read("Core/Inc/FreeRTOSConfig.h");
    const tasks = read("Core/Src/freertos.c");
    const ioc = read("CtrlBoard-H7_IMU.ioc");
    if (!/^#define INCLUDE_vTaskDelayUntil\s+1$/m.test(rtos) ||
        !/^FREERTOS.INCLUDE_vTaskDelayUntil=1\r?$/m.test(ioc)) throw new Error("Delay-until must stay enabled");
    for (const [task, entry] of [["REMOTO_TASK", "Remote_Task"], ["BOARD_COMMUNICA", "Board_communicate"]]) {
        if (!tasks.includes("osThreadDef(" + task + ", " + entry + ", osPriorityHigh, 0, 256);") ||
            !ioc.includes(task + ",2,256," + entry + ",")) throw new Error("Task configuration mismatch: " + task);
    }
    console.log("remote sources: both configurations, illegal macro and RTOS settings passed");
} finally {
    for (const file of ownedFiles) if (fs.existsSync(file)) fs.unlinkSync(file);
    fs.rmdirSync(temp);
}

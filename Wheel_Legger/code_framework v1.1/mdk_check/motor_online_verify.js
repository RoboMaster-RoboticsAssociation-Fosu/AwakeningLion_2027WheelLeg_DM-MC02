// Compile real feedback decoding, CAN callbacks, freshness checks and output packing.
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const { spawnSync } = require("node:child_process");
const root = path.resolve(__dirname, "..");
const temp = fs.mkdtempSync(path.join(os.tmpdir(), "motor-online-verify-"));
const ownedFiles = [];
const compiler = process.env.CC || "gcc";
function read(name, encoding = "utf-8") {
    return new TextDecoder(encoding, { fatal: true }).decode(fs.readFileSync(path.join(root, name)));
}
function write(name, contents) {
    const file = path.join(temp, name);
    ownedFiles.push(file);
    fs.writeFileSync(file, contents);
}
function definition(source, name) {
    const match = new RegExp("(?:static\\s+)?(?:void|float|uint16_t)\\s+" + name + "\\s*\\([^;{}]*\\)\\s*\\{").exec(source);
    if (!match) throw new Error("Missing production function: " + name);
    let depth = 0;
    for (let i = source.indexOf("{", match.index); i < source.length; ++i) {
        if (source[i] === "{") ++depth;
        if (source[i] === "}" && --depth === 0) return source.slice(match.index, i + 1);
    }
    throw new Error("Unbalanced function: " + name);
}
function run(program, args) {
    const result = spawnSync(program, args, { cwd: root, encoding: "utf8" });
    if (result.stdout) process.stdout.write(result.stdout);
    if (result.stderr) process.stderr.write(result.stderr);
    if (result.error) throw result.error;
    if (result.status !== 0) throw new Error(program + " exited with status " + result.status);
}
try {
    write("main.h", `#ifndef MOTOR_VERIFY_MAIN_H
#define MOTOR_VERIFY_MAIN_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#define FDCAN_STANDARD_ID 0U
#define FDCAN_EXTENDED_ID 1U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_REMOTE_FRAME 1U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_FD_CAN 1U
#define FDCAN_DLC_BYTES_8 (8U << 16)
#define FDCAN_RX_FIFO0 0U
#define FDCAN_RX_FIFO1 1U
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1U
#define FDCAN_IT_RX_FIFO1_NEW_MESSAGE 2U
#define RESET 0U
#define HAL_OK 0
#define HAL_ERROR 1
#define FDCAN1 ((void *)(uintptr_t)1U)
#define FDCAN2 ((void *)(uintptr_t)2U)
#define FDCAN3 ((void *)(uintptr_t)3U)
typedef struct { void *Instance; } FDCAN_HandleTypeDef;
typedef struct { uint32_t Identifier, IdType, RxFrameType, DataLength, FDFormat; } FDCAN_RxHeaderTypeDef;
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
void osDelay(uint32_t delay);
int HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *can, uint32_t fifo, FDCAN_RxHeaderTypeDef *header, uint8_t *data);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *can, uint32_t fifo);
#endif
`);
    write("bsp_can.h", '#ifndef MOTOR_VERIFY_CAN_H\n#define MOTOR_VERIFY_CAN_H\n#include "main.h"\n' +
        "typedef FDCAN_HandleTypeDef hcan_t;\n" +
        "uint8_t canx_send_data(hcan_t *can, uint16_t id, uint8_t *data, uint32_t len);\n#endif\n");
    write("stm32h723xx.h", '#include "main.h"\n');
    write("pid.h", "#ifndef MOTOR_VERIFY_PID_H\n#define MOTOR_VERIFY_PID_H\n" +
        "typedef int Pid_Set_Typedef;\ntypedef int PidTypedef;\n#endif\n");
    write("VMC_calc.h", "typedef struct { float F0, Tp, torque_set[2]; } vmc_leg_t;\n");
    for (const file of ["User/Devices/DM_Motor/DM_Motor.h", "User/Devices/DJI_Motor/DJI_Motor.h"]) {
        write(path.basename(file), read(file));
    }
    write("chassis_task.h", read("User/APP/chassis_task.h", "gb18030"));
    const chassis = read("User/APP/chassis_task.c", "gb18030");
    const can = read("User/Bsp/bsp_can.c");
    const dm = read("User/Devices/DM_Motor/DM_Motor.c");
    const dji = read("User/Devices/DJI_Motor/DJI_Motor.c");
    const feedbackState = [...chassis.matchAll(/^static volatile (?:uint32_t|uint8_t) motor_feedback_\w+\[[^\]]+\];/gm)];
    if (feedbackState.length !== 2) throw new Error("Missing production feedback state");
    const limitLines = read("User/Lib/user_lib.h").split(/\r?\n/);
    const limitStart = limitLines.findIndex(line => line.startsWith("#define VAL_LIMIT("));
    if (limitStart < 0) throw new Error("Missing production current limit macro");
    const limit = [];
    for (let i = limitStart; i < limitLines.length; ++i) {
        limit.push(limitLines[i]);
        if (!limitLines[i].trimEnd().endsWith("\\")) break;
    }
    write("motor_implementation.inc", [
        limit.join("\n"), ...feedbackState.map(m => m[0]),
        definition(dm, "uint_to_float"), definition(dm, "DM_Motor_Info_Update"),
        definition(dm, "DM_Enable_Motor"), definition(chassis, "DM_Enable"),
        definition(dji, "F_Loop_Constrain"), definition(dji, "DJI_Motor_Encoder_To_Anglesum"),
        definition(dji, "DJI_Motor_Info_Update"), definition(dji, "DJI_Motor_ctrl"),
        definition(read("User/Algorithm/some_config/some_para.c"), "mySaturate"),
        definition(chassis, "Chassis_MotorFeedbackReceived"),
        definition(chassis, "chassis_joint_enable_check"),
        definition(chassis, "chassis_motor_online_update"), definition(chassis, "Chassis_CanTransimit"),
        definition(can, "HAL_FDCAN_RxFifo0Callback"), definition(can, "HAL_FDCAN_RxFifo1Callback"),
    ].join("\n"));
    const executable = path.join(temp, "motor" + (process.platform === "win32" ? ".exe" : ""));
    ownedFiles.push(executable);
    run(compiler, ["-std=c99", "-Wall", "-Wextra", "-Werror", "-O0", "-finput-charset=UTF-8",
        "-I", temp, path.join(__dirname, "motor_online_verify.c"), "-lm", "-o", executable]);
    run(executable, []);
} finally {
    for (const file of ownedFiles) if (fs.existsSync(file)) fs.unlinkSync(file);
    fs.rmdirSync(temp);
}

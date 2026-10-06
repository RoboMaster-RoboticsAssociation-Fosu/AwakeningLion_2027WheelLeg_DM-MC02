# Repository Guidelines

## Project Structure & Module Organization

This C99 firmware controls a wheel-legged robot on a DM-MC02 board (STM32H723VG), using HAL and FreeRTOS.

- `User/APP/`: chassis, inertial-navigation, and remote-control tasks.
- `User/Algorithm/`: LQR, LESO, PID, VMC, and filtering; `User/Devices/`: motor, IMU, and remote drivers.
- `User/Bsp/`: CAN/UART/PWM interfaces; `User/Controller/`, `User/Config/`, and `User/Lib/`: control glue, configuration, and utilities.
- `Core/` and `USB_DEVICE/`: CubeMX-generated initialization and USB integration.
- `Drivers/` and `Middlewares/`: vendor HAL, CMSIS, FreeRTOS, and USB libraries.
- `MDK-ARM/`: authoritative Keil project and outputs; `mdk_check/`: numerical verification scripts.

## Build, Test, and Development Commands

Use Keil MDK with ArmClang 6.16. From this directory in Git Bash:

```bash
"/c/Keil_v5/UV4/UV4.exe" -b MDK-ARM/CtrlBoard-H7_IMU.uvprojx -t CtrlBoard-H7_IMU -j0 -o build.log
tail -3 MDK-ARM/build.log
node mdk_check/vmc_verify.js
node mdk_check/jacobian_verify.js
```

The build produces `.axf`/`.hex` files under `MDK-ARM/CtrlBoard-H7_IMU/`; replace `-b` with `-r` for a full rebuild. Require zero errors and no new warnings. PowerShell launches must use `Start-Process -Wait`. Run firmware through Keil Download/debug with CMSIS-DAP. EIDE configuration and J-Link flash scripts are outdated.

## Coding Style & Naming Conventions

For new code, use four spaces and preserve surrounding brace style. `.clang-format` specifies Microsoft-based formatting, four-space indentation, and unsorted includes; existing files vary, so keep formatting changes local. Preserve module prefixes such as `PID_Calc`, uppercase macros such as `LEG_PID_KP`, and existing filename case. Register new sources and include paths in the Keil project.

## Testing Guidelines

No application unit-test framework or coverage threshold is configured. The Node.js scripts compare VMC derivatives and Jacobians against finite differences; their geometry differs from firmware, so they validate formulas only. Name additional numerical checks `*_verify.js`. Validate control changes on hardware and record task timing, IMU feedback, and motor behavior.

## Commit & Pull Request Guidelines

History uses short descriptive subjects, often Chinese, without a consistent prefix convention. Describe the concrete change. Stage specific files: builds modify tracked binaries and Keil workspace settings. PRs should explain purpose, affected modules, build results, hardware validation, and related issues when applicable.

## Firmware Configuration & Editing Precautions

Read `CLAUDE.md` before changing control logic, CAN mappings, RTOS timing, or source encodings. Preserve each source file's GBK/UTF-8 encoding and line endings using byte-aware edits. Save Markdown as UTF-8 with BOM and CRLF. Review CubeMX regeneration carefully: manual RTOS changes exist outside user-code blocks.

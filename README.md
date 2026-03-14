# FreeRTOS Multi-Task IMU Data Acquisition System
**Platform:** STM32 Nucleo F401RE | **RTOS:** FreeRTOS 10.3.1 | **Language:** C

A production-grade embedded firmware implementing a real-time multi-task sensor fusion system using FreeRTOS on the STM32F401RE microcontroller.

---

## Project Overview

This project demonstrates preemptive multitasking firmware with 4 concurrent FreeRTOS tasks, inter-task communication via queues, thread-safe resource sharing via mutexes, and real-time Kalman filter processing of IMU data at 100Hz.

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU | STM32F401RE (ARM Cortex-M4, 84MHz) |
| Board | NUCLEO-F401RE |
| IMU Sensor | MPU6050 GY-521 (Accel + Gyro) |
| Interface | I2C1 (PB8=SCL, PB9=SDA) |
| UART | USART2 @ 115200 baud (PA2/PA3) |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    FreeRTOS Scheduler                    │
├──────────────┬──────────────┬──────────────┬────────────┤
│  Task_IMU    │ Task_Kalman  │  Task_UART   │  Task_LED  │
│  Priority 4  │  Priority 3  │  Priority 2  │ Priority 1 │
│    10ms      │   Event      │   100ms      │   500ms    │
├──────────────┴──────────────┴──────────────┴────────────┤
│         xRawDataQueue          xAnglesQueue              │
│         (IMU → Kalman)         (Kalman → UART)          │
├─────────────────────────────────────────────────────────┤
│              xUARTMutex (thread-safe UART)               │
└─────────────────────────────────────────────────────────┘
```

---

## Tasks

### Task 1 — IMU Read (Priority 4, 10ms period)
- Reads raw accelerometer and gyroscope data from MPU6050 over I2C
- Runs at 100Hz using `vTaskDelayUntil` for precise timing
- Sends raw data to `xRawDataQueue` using `xQueueOverwrite`

### Task 2 — Kalman Filter (Priority 3, event-driven)
- Receives raw IMU data from queue
- Computes roll and pitch angles using a Kalman filter
- Sends filtered angles to `xAnglesQueue`

### Task 3 — UART Print (Priority 2, 100ms period)
- Prints Roll/Pitch values every 100ms
- Every 1 second prints stack high-water marks and CPU usage per task
- Uses `xUARTMutex` for thread-safe UART access

### Task 4 — LED Blink (Priority 1, 500ms period)
- Toggles onboard LED as system heartbeat
- Refreshes IWDG watchdog every 500ms

---

## FreeRTOS Features Demonstrated

| Feature | Usage |
|---------|-------|
| Preemptive scheduling | 4 tasks with distinct priorities |
| `xQueueCreate` / `xQueueOverwrite` / `xQueueReceive` | IMU → Kalman → UART pipeline |
| `xSemaphoreCreateMutex` | Thread-safe UART access |
| `vTaskDelayUntil` | Precise periodic task timing |
| `uxTaskGetStackHighWaterMark` | Per-task stack monitoring |
| `vTaskGetRunTimeStats` | Per-task CPU usage % |
| IWDG Watchdog | System reliability — resets MCU if LED task freezes |

---

## Serial Output

Connect at **115200 baud** to see live output:

```
FreeRTOS IMU System Started! (IWDG + CPU Stats Active)
Roll:  12.34 Pitch:  -5.67
Roll:  12.35 Pitch:  -5.66
...
--- Stack HWM (words) IMU:187 KF:312 UART:398 LED:112 ---
--- CPU Usage ---
IMU          4500        45%
Kalman       3200        32%
UART         1800        18%
LED           500         5%
-----------------
```

---

## Project Structure

```
freertos_imu/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   └── FreeRTOSConfig.h       # FreeRTOS configuration
│   └── Src/
│       ├── main.c                 # HAL init, peripheral config
│       ├── freertos.c             # All 4 tasks + FreeRTOS init
│       └── stm32f4xx_hal_timebase_tim.c
├── Drivers/
│   ├── STM32F4xx_HAL_Driver/
│   └── CMSIS/
└── Middlewares/
    └── Third_Party/FreeRTOS/
```

---

## Building & Flashing

### Requirements
- STM32CubeIDE 1.x or later
- STM32CubeProgrammer (for command-line flashing)

### Build
Open in STM32CubeIDE and press `Ctrl+B` (Windows/Linux) or `Cmd+B` (Mac).

### Flash via command line
```bash
STM32_Programmer_CLI -c port=SWD -w Debug/freertos_imu.elf -v -rst
```

---

## Key Design Decisions

**Why `xQueueOverwrite` instead of `xQueueSend`?**
The queue depth is 1 — we always want the latest sensor reading, not a backlog of old data. `xQueueOverwrite` replaces the existing item rather than blocking.

**Why is IWDG kicked from the lowest priority task?**
If any higher priority task hangs and starves the LED task, the watchdog won't be refreshed and the MCU resets. This is a classic production embedded pattern.

**Why `vTaskDelayUntil` instead of `vTaskDelay`?**
`vTaskDelayUntil` maintains a fixed period regardless of task execution time, preventing timing drift over long runs.

## Related Project

[MPU6050 Sensor Fusion (Bare-metal)](../mpu6050_fusion) — The predecessor to this project, implementing Kalman filter sensor fusion on bare-metal STM32 without an RTOS.

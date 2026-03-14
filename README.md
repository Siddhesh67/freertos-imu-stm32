# STM32 Real-Time IMU Sensor Fusion

Real-time orientation estimation using MPU6050 IMU and Kalman Filter on STM32 Nucleo F401RE (ARM Cortex-M4 @ 84MHz), with a live 3D Python visualizer.

![Demo](demo.gif)

---

## Project Overview

This project implements a **Kalman Filter** in embedded C to fuse accelerometer and gyroscope data from an MPU6050 sensor over I2C. The filtered roll and pitch angles are streamed over UART at 100Hz to a Python/pygame 3D cube visualizer running on a Mac/PC.

**Key achievements:**
- Kalman Filter running at 100Hz on ARM Cortex-M4
- Stable roll/pitch estimation with gyroscope drift correction
- Real-time 3D visualization over serial UART
- Full embedded C firmware from scratch using STM32 HAL

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU | STM32 Nucleo F401RE (Cortex-M4 @ 84MHz) |
| IMU | MPU6050 (GY-521 module) — 3-axis accel + 3-axis gyro |
| Interface | I2C @ 100kHz |
| Communication | UART @ 115200 baud |

---

## Wiring

| GY-521 Pin | Nucleo Pin | Notes |
|------------|-----------|-------|
| VCC | 3.3V | 3.3V only |
| GND | GND | |
| SCL | D15 (PB8) | I2C1 Clock |
| SDA | D14 (PB9) | I2C1 Data |
| AD0 | GND | Sets I2C address to 0x68 |

---

## Project Structure

```
STM32-IMU-Sensor-Fusion/
├── Core/
│   ├── Src/
│   │   ├── main.c          # Main firmware — MPU6050 + Kalman loop
│   │   └── kalman.c        # Kalman Filter implementation
│   └── Inc/
│       ├── main.h          # STM32 HAL header
│       └── kalman.h        # Kalman Filter struct and declarations
├── visualizer.py           # Python 3D cube visualizer
└── README.md
```

---

## How It Works

### Sensor Fusion
1. **Read** raw accelerometer and gyroscope data from MPU6050 over I2C
2. **Compute** roll and pitch from accelerometer using arctangent
3. **Fuse** with gyroscope using Kalman Filter to eliminate noise and drift
4. **Transmit** filtered angles over UART every 10ms

### Kalman Filter
The filter maintains a 2-state model (angle + gyro bias) with a 2x2 covariance matrix. Tuning parameters:
- `Q_angle = 0.001` — process noise for angle
- `Q_bias = 0.003` — process noise for gyro bias
- `R_measure = 0.03` — measurement noise

---

## Build & Flash Instructions

### Requirements
- STM32CubeIDE
- STM32CubeProgrammer (for Mac flashing)
- Python 3 with `pyserial`, `pygame`, `numpy`

### Build
1. Open project in STM32CubeIDE
2. Press `Cmd+B` (Mac) or `Ctrl+B` (Windows) to build
3. Make sure `-u _printf_float` is added in **Project Properties → C/C++ Build → Settings → MCU GCC Linker → Miscellaneous → Other flags**

### Flash (Mac)
```bash
/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.macosaarch64_1.0.0.202601242230/tools/bin/STM32_Programmer_CLI -c port=SWD -w Debug/mpu6050_fusion.elf -v -rst
```

### Install Python dependencies
```bash
pip3 install pyserial pygame numpy
```

### Run Visualizer
1. Find your serial port:
```bash
ls /dev/tty.usb*
```
2. Update `SERIAL_PORT` in `visualizer.py`
3. Run:
```bash
python3 visualizer.py
```

---

## Results

- ✅ Stable roll/pitch at 100Hz
- ✅ Kalman filter eliminates gyroscope drift
- ✅ Live 3D visualization over UART
- ✅ MPU6050 detected at I2C address 0x68

---

## Future Improvements

- [ ] Upgrade to FreeRTOS with separate tasks for IMU read, Kalman filter, and UART transmit
- [ ] Add SPI proximity sensor
- [ ] Add complementary yaw estimation using magnetometer
- [ ] Log data to SD card

---

## Author

**Siddhesh Saraf**  
Embedded Systems | STM32 | C | Python  
[GitHub](https://github.com/YOURUSERNAME) | [LinkedIn](https://linkedin.com/in/YOURUSERNAME)

---

## License

MIT License

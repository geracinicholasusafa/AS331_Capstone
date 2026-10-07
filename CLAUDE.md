# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project goal

This is the Astro 331 capstone: an autonomous **Active Thermal Control System (TCS)** for the KestrelSAT cubesat. It uses adjustable louvers (servos), temperature sensors, a 4-photoresistor sun sensor and the reaction wheel. The requirements live in `README.docx` at the repo root. Read it with `pandoc -t markdown README.docx`, or extract `word/document.xml` if pandoc is missing. `Astro 331 Agentic AI Workflow.pptx` is course material about the AI workflow.

The TCS is **to be built**. The only code today is the Lab 6/7 firmware in `Astro331_Lab7_Code/`, which the TCS will extend. Do not start a separate project.

### TCS design (from README.docx and the team)

- **State machine**, non-blocking, called from `loop()`: `NORMAL`, `COOLING`, `COLD_DETECTED`, `SUN_SEARCH`, `SOLAR_HEATING`, `RECOVERY`, `FAULT`.
  - Too hot: open the louvers to radiate heat.
  - Too cold: close the louvers, find the Sun with the photoresistors, rotate the Mylar thermal surface toward it with the reaction wheel, then open the louvers to absorb heat.
  - `FAULT` puts the louvers and attitude control in the safest configuration. Fault cases include sensor failure, servo failure, wheel saturation, over- and under-temperature, Sun not found, and heating or cooling too fast.
  - Overheating and over-cooling limits are required, including a limit on how long the louvers stay open.
- Log continuously to the SD card: sensors, commands, actuator states and thermal response.
- **Peltier module (5 V, 1 A)** is a *test fixture*. It heats and cools the metal panel to simulate the satellite's thermal environment, and the TCS reacts with louvers and attitude. The **firmware drives it**, and it must accept a **user-specified target temperature** (a ground-station command) and drive the panel to it. The TCS control logic must not depend on it. Reversing polarity needs an H-bridge, and the hardware is not yet chosen, so put it behind an interface like the servos. Test it in air, not vacuum, so convection weakens the louver effect.
- **Thermocouples** on the panel are read through a **MAX31855** amplifier (SPI, read-only, shares the SD card's bus with its own chip-select; its fault bits map to the sensor-failure state). The planned library is `adafruit/Adafruit MAX31855 library`.
- **Louver servos** are not yet chosen, so put them behind a small interface (position 0–100%).
- **Sun sensor** is the existing A0–A3 photoresistor array. Reuse the sun-direction math in `lab7_run_test_B()`.
- **Proposed thresholds** for room-temperature bench tests (to be confirmed, and kept as constants in `definitions.h`): target 30 °C, normal band 26–34 °C, `FAULT` above 45 °C or below 10 °C, or beyond 1 °C/s.

## Rules (set by the team)

- **Limits:** panel `FAULT` above 45 °C or below 10 °C. The Peltier is firmware-driven and must obey these limits and cut out on a sensor fault. The **reaction wheel is capped at 400 RPM**. Note that the existing mapping gives about 421 RPM at full throttle (`1000 * 5.05 / 12`), so the TCS must clamp throttle (about 0.95) or use measured speed.
- **Thermal band:** keep the proposed values: target 30 °C, normal band 26 to 34 °C, `COOLING` above 34 °C and exits at 30 °C, `COLD_DETECTED` below 26 °C and exits at 30 °C.
- **Existing commands:** leave Lab 6 and Lab 7 commands (0 to 9, 98, 99) and their code untouched. Only add new commands (use 10 and up) and new code.
- **Code style:** write for readers who don't code. Use plain names (`louver_open_percent`, not `lvr_pos`), and comment each function and block with what it does and why in plain language. Put all tunable numbers in `definitions.h` with a comment giving units.
- **Logging:** the TCS writes its own CSV file on the SD card, and also streams live temperature readings in a form the team's live-logging GUI can read. The GUI's data format has not been supplied yet, so ask before finalizing the stream format.
- **Git:** you may commit and push to `origin` when a task is complete. Do not flash hardware or run commands that move actuators without being asked.
- **Unknown hardware:** where a part is not chosen (servos, Peltier H-bridge), stub it behind a small interface and note it. Do not guess pins or parts.

## Commands

All commands run from `Astro331_Lab7_Code/` (PlatformIO, target `sparkfun_esp32s2_thing_plus_c`, Arduino framework):

```bash
pio run                        # build
pio run -t upload              # flash
pio device monitor -b 115200   # USB serial console
```

There is no firmware test suite. `test/` holds only the PlatformIO placeholder README.

## Current firmware architecture

- `src/main.cpp` holds all application logic. `loop()` polls for commands every 10 ms and sends a heartbeat. Commands are integers read from the XBee UART (9600 baud) or USB Serial (115200), dispatched in `process_main_menu()`. The command table appears in the file header and in the case `1` menu text, so update both when adding commands.
- Each existing test (`lab6_run_test`, `lab7_run_test_A`, `lab7_run_test_B`, `stream_RWspeed`) is a **blocking `while(true)` loop** that owns the CPU until it ends or receives `X`. A continuous TCS cannot be built the same way. It must be a state machine stepped from `loop()`, with the command menu still responsive.
- `src/sd_funcitons.cpp` (the filename typo is intentional history; the header is `include/sd_functions.h`) wraps SdFat. The `SdFat sd` object is file-scope static, so only these helpers touch the card. `sd_init()` loops forever if no card is found. `sd_createDataFile()` names files `<preamble>_dataNNN.csv` (preamble truncated to 12 characters, maximum 999).
- `include/definitions.h` is the pin map and constants. Pins already in use: 5 (SD CS), 14, 32 (encoder), 15, 33 (motor PWM), 16, 17 (XBee), 18, 19, 23 (SPI), A0–A3 (sun sensor). `MAX31855_CS_PIN` is GPIO 27. Despite the "S2" board name, `platformio.ini`'s `sparkfun_esp32s2_thing_plus_c` resolves to the classic ESP32 `esp32thing_plus_c` variant, where `A0..A3` = GPIO 26, 25, 34, 39 and I2C = 21/22. Other pins left free for servos are 4 and 13 (13 is also the status LED); avoid the boot-strapping pins 0, 2, 5, 12 and 15, and note that 34 to 39 are input-only.
- Reaction wheel: the TB9051FTG driver takes throttle in [-1, 1] via `driver.setOutput()`. RPM is computed as `dc / (CPR * 10) / dt * 60`, and the `10` is hardcoded in several places. Commanded RPM is `-speed_pwm * 1000 * MOTOR_VOLTAGE / 12`.
- Shared sensor globals (`gyro_Z`, `mag_X/Y`, `sun_*`) are set inside the test loops and are only valid while a test is running.
- `lib/INA238-master/` is a vendored library that `main.cpp` does not use.

## Gotchas

- `lab6_run_test()` has `sun_X` and `sun_Y` deliberately left blank (`// sun_X = ;`) for students. `lab7_run_test_B()` has the finished formulas `(plus - minus) / S_mag`.
- Header comments disagree with the code in places, such as the I2C addresses, the heartbeat interval (documented 2 s, actual 3 s) and the Test B duration. Trust the code.
- In `lab7_run_test_B()`, `test_point_count` is declared inside the loop and resets each iteration, so the every-10-points flush and XBee progress message never fire.

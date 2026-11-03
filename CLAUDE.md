# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project goal

This is the Astro 331 capstone: an autonomous **Active Thermal Control System (TCS)** for the KestrelSAT cubesat. It uses adjustable louvers (servos), temperature sensors, a 4-photoresistor sun sensor and the reaction wheel. The requirements live in `README.docx` at the repo root. Read it with `pandoc -t markdown README.docx`, or extract `word/document.xml` if pandoc is missing. `Astro 331 Agentic AI Workflow.pptx` is course material about the AI workflow.

The TCS extends the Lab 6/7 firmware in `Astro331_Lab7_Code/`. Do not start a separate project.

**Built so far (compiles, not yet tested on hardware):** `include/tcs.h` and `src/tcs.cpp` hold the MAX31855 reading and the 6-state machine, called from `loop()` through `tcs_update()`. Ground-station commands: `10` toggles the TCS on/off, `11` prints status, `12` clears a fault, `13` runs a thermocouple wiring test (bit-bangs the MAX31855 with the data line pulled up, then down, and explains the result), `14` toggles the live temperature stream, `15` moves the louvers by hand (0 to 100% open, TCS must be off). The stream is **on by default**: one `TCS,t:...,temp_C:...,...` line per second on both USB serial and XBee, in the `key:value` style of the Lab 6/7 output (format documented in `tcs.h`). That format is an assumption, since the GUI's real format is still unconfirmed. Commands 5 to 9 switch the TCS off first, because they take over the reaction wheel. The louver servo is a **Tower Pro SG92R**, driven by `set_louvers()` in `tcs.cpp` straight from the ESP32 PWM hardware (LEDC channel 0, 50 Hz, 14-bit; no servo library, because the wheel motor driver's `analogWrite` already takes LEDC channels 15 and 14 at 1 kHz). It starts only on the first move so it does not twitch at power-up. Signal pin is GPIO 4. The team confirmed there is exactly **one** louver servo. Open and closed angles are 90 and 0 degrees (proposed, depend on the mounting). **Peltier test heater (heating only, built, untested on hardware):** driven as PWM on GPIO 12 (proposed) through an external switch (MOSFET/transistor, part not yet confirmed) on its own 5 V supply, LEDC channel 4 at 1 kHz. `run_peltier()` in `tcs.cpp` runs every TCS update whether or not the TCS is on. Command `16` holds the panel at a target temperature (15 to 40 C, 0 = off) with a PI controller (gains in `definitions.h`, untuned), `17` runs a fixed power percentage for bench testing. Safety: it pauses with no reading, turns off for good after `TCS_SENSOR_FAIL_LIMIT` bad reads or when the panel passes `TCS_FAULT_HIGH_C`, and is switched off before commands 5 to 9, 98 and 99 because those block the loop. **SD card CSV log (built, untested on hardware):** command `18` starts/stops it. Each start creates a new `TCS_log_dataNNN.csv` via `sd_createDataFile()`, one row per `TCS_LOG_INTERVAL_MS` with `time_ms, elapsed_s, temp1_C, temp2_C, rate_C_per_s, state, state_id, tcs_on, louver_pct, wheel_rpm, vent_error_deg, peltier_pct, peltier_target_C` (empty cell = no valid reading), flushed every row. Written from `log_row_if_due()` in `tcs_update()`, so it pauses during the blocking Lab tests. **Heater control page:** `GUI/heater_control.html` is a standalone page (Chrome/Edge only, uses the Web Serial API, no install, no internet needed). It connects to the USB serial port (115200) or the XBee adapter (9600), parses the `TCS,...` stream to plot `temp_C`/`temp2_C`/heater target and heater %, and drives the heater by sending `17\n<percent>\n` and `16\n<target>\n` in one write (the firmware reads the command line, then its prompt finds the number already waiting). It can also send 18/11, any raw command, and save what it received as CSV. The team's own `KestrelSAT_GCS.exe` is separate and untouched. **Not built yet:** confirmation of the GUI stream format.

### TCS design (from README.docx and the team)

- **There is no Mylar thermal surface and no solar heating.** The team's concept (which supersedes the older `README.docx` wording about `SUN_SEARCH` / `SOLAR_HEATING`) is that internal heat must be vented. The Sun is only used to decide which way to point the louvers.
- **State machine**, non-blocking, called from `loop()`: `NORMAL`, `SUN_AVOID`, `COOLING`, `COLD_DETECTED`, `RECOVERY`, `FAULT`.
  - Too hot: `SUN_AVOID` keeps the louvers closed while the reaction wheel turns the satellite until the louvers face directly *away* from the brightest light (found with the 4 photoresistors). Then `COOLING` opens the louvers to vent. No bright light means vent immediately. If turning takes longer than `TCS_SUN_AVOID_TIMEOUT_MS`, vent anyway. If the louvers drift toward the light while venting, close them and turn again.
  - Too cold: close the louvers to hold heat in and wait for the satellite to warm up. No attitude control.
  - `FAULT` puts the louvers and attitude control in the safest configuration. Fault cases include sensor failure, servo failure, wheel saturation, over- and under-temperature, and heating or cooling too fast.
  - Overheating and over-cooling limits are required, including a limit on how long the louvers stay open.
- Log continuously to the SD card: sensors, commands, actuator states and thermal response.
- **Peltier module (5 V, 1 A)** is a *test fixture*. It is used **for heating only** (decided by the team, so no H-bridge or polarity reversal) to warm the metal panel and simulate the satellite's thermal environment, and the TCS reacts with louvers and attitude. The **firmware drives it**, and it accepts a **user-specified target temperature** (command 16) and drives the panel to it. The TCS control logic must not depend on it. Test it in air, not vacuum, so convection weakens the louver effect.
- **Thermocouples** on the panel are read through a **MAX31855** amplifier (SPI, read-only, shares the SD card's bus with its own chip-select; its fault bits map to the sensor-failure state). The planned library is `adafruit/Adafruit MAX31855 library`.
- **Louver servo** is the Tower Pro SG92R (5 V, 50 Hz, about 500 to 2400 µs pulses), behind a small interface (position 0–100%).
- **Sun sensor** is the existing A0–A3 photoresistor array. Reuse the sun-direction math in `lab7_run_test_B()`.
- **Proposed thresholds** for room-temperature bench tests (to be confirmed, and kept as constants in `definitions.h`): target 30 °C, normal band 26–34 °C, `FAULT` above 45 °C or below 10 °C, or beyond 200 °C/s (raised from 1, 5, then 15 °C/s, which tripped too easily; at this value the rate check only catches a wildly wrong reading).

## Rules (set by the team)

- **Limits:** panel `FAULT` above 45 °C or below 10 °C. The Peltier is firmware-driven and must obey these limits and cut out on a sensor fault. The **reaction wheel is capped at 400 RPM**. Note that the existing mapping gives about 421 RPM at full throttle (`1000 * 5.05 / 12`), so the TCS must clamp throttle (about 0.95) or use measured speed.
- **Thermal band:** keep the proposed values: target 30 °C, normal band 26 to 34 °C, `COOLING` above 34 °C and exits at 30 °C, `COLD_DETECTED` below 26 °C and exits at 30 °C.
- **Existing commands:** leave Lab 6 and Lab 7 commands (4 to 9, 98, 99) and their code untouched. Only add new commands (use 10 and up) and new code. **Exception, at the team's request:** command 3 (toggle the status LED on GPIO 13) was removed because GPIO 13 is now the second thermocouple's chip-select.
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

On this machine the repo's OneDrive path is too long for the ICM-20948 library's nested example files, so the first `pio run` fails while installing libraries. Build with a short workspace instead (PlatformIO is at `~/.platformio/penv/Scripts/pio.exe`, not on PATH):

```bash
PLATFORMIO_WORKSPACE_DIR="C:/Users/C28Nicholas.Geraci/pio_ws" ~/.platformio/penv/Scripts/pio.exe run
```

There is no firmware test suite. `test/` holds only the PlatformIO placeholder README.

## Current firmware architecture

- `src/main.cpp` holds all application logic. `loop()` polls for commands every 10 ms and sends a heartbeat. Commands are integers read from the XBee UART (9600 baud) or USB Serial (115200), dispatched in `process_main_menu()`. The command table appears in the file header and in the case `1` menu text, so update both when adding commands.
- Each existing test (`lab6_run_test`, `lab7_run_test_A`, `lab7_run_test_B`, `stream_RWspeed`) is a **blocking `while(true)` loop** that owns the CPU until it ends or receives `X`. A continuous TCS cannot be built the same way. It must be a state machine stepped from `loop()`, with the command menu still responsive.
- `src/sd_funcitons.cpp` (the filename typo is intentional history; the header is `include/sd_functions.h`) wraps SdFat. The `SdFat sd` object is file-scope static, so only these helpers touch the card. `sd_init()` loops forever if no card is found. `sd_createDataFile()` names files `<preamble>_dataNNN.csv` (preamble truncated to 12 characters, maximum 999).
- `include/definitions.h` is the pin map and constants. Pins already in use: 5 (SD CS), 14, 32 (encoder), 15, 33 (motor PWM), 16, 17 (XBee), 18, 19, 23 (SPI), A0–A3 (sun sensor). `MAX31855_CS_PIN` is GPIO 27 (thermocouple 1, the panel, used for control and the Peltier) and `MAX31855_2_CS_PIN` is GPIO 13 (thermocouple 2, reported only as `temp2_C` in the stream and status; the control logic ignores it). GPIO 4 is the louver servo and GPIO 12 the Peltier MOSFET, so there are no free output pins left. Despite the "S2" board name, `platformio.ini`'s `sparkfun_esp32s2_thing_plus_c` resolves to the classic ESP32 `esp32thing_plus_c` variant, where `A0..A3` = GPIO 26, 25, 34, 39 and I2C = 21/22. Other pins left free for servos are 4 and 13 (13 is also the status LED); avoid the boot-strapping pins 0, 2, 5, 12 and 15, and note that 34 to 39 are input-only.
- Reaction wheel: the TB9051FTG driver takes throttle in [-1, 1] via `driver.setOutput()`. RPM is computed as `dc / (CPR * 10) / dt * 60`, and the `10` is hardcoded in several places. Commanded RPM is `-speed_pwm * 1000 * MOTOR_VOLTAGE / 12`.
- Shared sensor globals (`gyro_Z`, `mag_X/Y`, `sun_*`) are set inside the test loops and are only valid while a test is running.
- `lib/INA238-master/` is a vendored library that `main.cpp` does not use.

## Gotchas

- `lab6_run_test()` has `sun_X` and `sun_Y` deliberately left blank (`// sun_X = ;`) for students. `lab7_run_test_B()` has the finished formulas `(plus - minus) / S_mag`.
- Header comments disagree with the code in places, such as the I2C addresses, the heartbeat interval (documented 2 s, actual 3 s) and the Test B duration. Trust the code.
- In `lab7_run_test_B()`, `test_point_count` is declared inside the loop and resets each iteration, so the every-10-points flush and XBee progress message never fire.

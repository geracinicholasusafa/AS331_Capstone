#pragma once
/**
 * @file tcs.h
 * @brief Thermal Control System (TCS) for KestrelSAT.
 *
 * @details In plain words: the TCS keeps the satellite panel at a comfortable temperature.
 *   - Too hot  -> turn the satellite with the reaction wheel so the louvers face AWAY from the
 *                 brightest light (toward cold space), then open the louvers so internal heat escapes.
 *   - Too cold -> close the louvers to keep the heat in, and wait for the satellite to warm up.
 *   - Something looks wrong -> go to FAULT, which puts everything in the safest position.
 *
 * The TCS never "waits around". Every call to tcs_update() does a quick bit of work and returns,
 * so the rest of the program (the ground-station menu, the heartbeat) keeps running.
 *
 * All numbers it uses (temperatures, times, limits) are in definitions.h.
 */

#include <Arduino.h>

/** The steps the TCS can be in. Only one is active at a time. */
enum TcsState {
  TCS_NORMAL,         ///< Temperature is fine. Nothing to do except watch.
  TCS_SUN_AVOID,      ///< Too hot and pointed well off "away from the light". The wheel turns the satellite while the louvers keep venting.
  TCS_COOLING,        ///< Too hot and pointed away from the light (or no light). Louvers vent, and the wheel keeps correcting small drifts.
  TCS_COLD_DETECTED,  ///< Too cold. Louvers closed to keep heat in, waiting to warm up.
  TCS_RECOVERY,       ///< Heating/cooling is done. Louvers return to rest before going back to NORMAL.
  TCS_FAULT           ///< Something is wrong. Everything goes to its safest position and stays there.
};

/** The reasons the TCS can enter FAULT. */
enum TcsFaultReason {
  TCS_FAULT_NONE,
  TCS_FAULT_SENSOR_FAILURE,   ///< Thermocouple unplugged, shorted, or not answering.
  TCS_FAULT_TOO_HOT,          ///< Panel hotter than TCS_FAULT_HIGH_C.
  TCS_FAULT_TOO_COLD,         ///< Panel colder than TCS_FAULT_LOW_C.
  TCS_FAULT_RATE_TOO_FAST,    ///< Temperature changing faster than is believable.
  TCS_FAULT_WHEEL_SATURATED   ///< Reaction wheel reached its speed limit.
};

/*---------------------------------------------------------------------------------------------*/
// Setup and the main "heartbeat" of the TCS
/*---------------------------------------------------------------------------------------------*/

/**
 * Starts the two thermocouple chips (each chip reads one thermocouple). Call once from setup(), BEFORE the SD card starts.
 * Thermocouple 1 is the panel and is the one the thermal control and the Peltier heater use.
 * Thermocouple 2 is reported only (live stream and status); the control logic does not use it.
 */
bool tcs_init(uint8_t thermocouple1ChipSelectPin, uint8_t thermocouple2ChipSelectPin);

/** Call this every pass through loop(). It reads the thermocouple and, if the TCS is on, runs the state machine. */
void tcs_update();

/*---------------------------------------------------------------------------------------------*/
// Turning the TCS on and off (ground-station commands use these)
/*---------------------------------------------------------------------------------------------*/

/** Turns automatic thermal control on. Starts in NORMAL. */
void tcs_enable();

/**
 * Sets the hold temperature (the GUI's Hold button, command 19) and switches automatic control on.
 * Above the hold temperature the satellite turns its louvers away from the brightest light, then opens
 * them: wider the hotter it is. It keeps venting until the panel is TCS_COOL_HYSTERESIS_C below the hold
 * temperature, then closes them. It does NOT touch the heater; the heater is set separately (command 17).
 * @return false if the temperature is outside TCS_MIN_HOLD_TEMP_C to TCS_MAX_HOLD_TEMP_C (nothing changes).
 */
bool tcs_set_hold_temperature(float holdC);
float tcs_get_hold_temperature();

/** Turns automatic thermal control off: louvers to rest, wheel stopped. The thermocouple is still read. */
void tcs_disable();

/** Clears a FAULT and restarts in NORMAL (only works if the TCS is on and the cause is gone). */
void tcs_clear_fault();

bool tcs_is_enabled();

/*---------------------------------------------------------------------------------------------*/
// Reading what the TCS knows (for the menu, logging, and the live GUI)
/*---------------------------------------------------------------------------------------------*/

TcsState       tcs_get_state();
TcsFaultReason tcs_get_fault_reason();
const char*    tcs_state_name(TcsState state);
const char*    tcs_fault_name(TcsFaultReason reason);

/** Latest panel temperature in Celsius. NAN if the thermocouple has no valid reading. */
float tcs_get_panel_temp_c();

/** Latest temperature from thermocouple 2 in Celsius. NAN if it has no valid reading. Not used by the control logic. */
float tcs_get_temp2_c();

/** Temperature inside the MAX31855 chip itself (its "cold junction"), in Celsius. NAN if unknown. */
float tcs_get_chip_temp_c();

/** How fast the panel temperature is changing, in Celsius per second (positive = warming). */
float tcs_get_temp_rate_c_per_s();

/** Where the louvers have been commanded, 0 = closed to 100 = fully open. */
float tcs_get_louver_percent();

/**
 * How far the louvers are from pointing directly away from the brightest light, in degrees
 * (0 = perfectly aimed at cold space). NAN if no bright light is seen.
 */
float tcs_get_vent_error_deg();

/** Latest reaction wheel speed in RPM, measured with the encoder. */
float tcs_get_wheel_rpm();

/** Prints a one-line human-readable status to USB serial and XBee. */
void tcs_print_status();

/*---------------------------------------------------------------------------------------------*/
// Peltier test heater (heats the panel to simulate the satellite warming up)
// It is separate from the thermal control logic: it is a tool for testing, not part of the satellite.
// It always shuts itself off if the thermocouple fails or the panel gets above TCS_FAULT_HIGH_C.
/*---------------------------------------------------------------------------------------------*/

/**
 * Heats the panel and holds it at a chosen temperature (degrees Celsius).
 * 0 turns the heater off. Anything else must be between TCS_PELTIER_MIN_TARGET_C and
 * TCS_PELTIER_MAX_TARGET_C. It can only heat, so a target below the current temperature just means "off".
 * @return true if accepted, false if the target was out of range (the heater is left as it was).
 */
bool tcs_set_peltier_target(float targetC);

/** Runs the heater at a fixed percentage of full power (0 to 100) for bench testing. Still shuts off on any safety trip. */
void tcs_set_peltier_power_manual(float percent);

/** Turns the heater off immediately. */
void tcs_stop_peltier();

float tcs_get_peltier_power_percent();   ///< How hard the heater is running right now, 0 to 100
float tcs_get_peltier_target_c();        ///< The temperature it is aiming for. NAN if not in target mode.

/**
 * Moves the louvers by hand (0 = closed, 100 = fully open) for bench testing the servo.
 * Only works while automatic control is OFF, otherwise the TCS would just move them back.
 */
void tcs_set_louvers_manual(float openPercent);

/*---------------------------------------------------------------------------------------------*/
// SD card CSV log
/*---------------------------------------------------------------------------------------------*/

/**
 * Starts writing a new CSV file on the SD card (TCS_log_data001.csv, then 002, ...), one row every
 * TCS_LOG_INTERVAL_MS. Columns:
 *   time_ms, elapsed_s, temp1_C, temp2_C, rate_C_per_s, state, state_id, tcs_on, hold_C,
 *   louver_pct, wheel_rpm, vent_error_deg, light_dir_deg, light_total, peltier_pct, peltier_target_C
 * temp1_C is thermocouple 1 (the panel), temp2_C is thermocouple 2. An empty cell means no valid reading.
 * Every row is saved straight away, so pulling the power loses at most one row.
 * Logging pauses while a Lab 6/7 test is running (those tests take over the processor).
 * @return false if the file could not be created.
 */
bool tcs_start_logging();

/** Closes the log file. Always stop logging before removing the SD card. */
void tcs_stop_logging();

bool tcs_is_logging();

/**
 * Turns the live temperature stream on or off. It is ON by default.
 *
 * While on, one line is sent every TCS_TELEMETRY_INTERVAL_MS to both USB serial and the XBee, like:
 *   TCS,t:12345,temp_C:23.45,temp2_C:22.80,chip_C:24.10,rate_C_per_s:0.12,state_id:0,tcs_on:1,hold_C:30.0,louver_pct:0,wheel_rpm:0.0,vent_error_deg:nan,light_dir_deg:90.0,light_total:2400,light_px:600,light_mx:500,light_py:800,light_my:500,peltier_pct:0.0,peltier_target_C:nan
 *   light_dir_deg is where the brightest light comes from (0 = +X, 90 = +Y, 180 = -X, 270 = -Y; nan = too dark),
 *   light_px/mx/py/my are the four light sensors (0 to 4095). The light is measured every update, even with the TCS off.
 * Every value after the colon is a plain number ("nan" means "no valid reading"), so a plotting
 * program can read each one directly. state_id follows the order of TcsState:
 *   0=NORMAL 1=SUN_AVOID 2=COOLING 3=COLD_DETECTED 4=RECOVERY 5=FAULT
 * The stream pauses while a Lab 6/7 test is running, because those tests take over the processor.
 */
void tcs_set_telemetry(bool on);
bool tcs_telemetry_enabled();

/**
 * Wiring test for both thermocouple chips. Talks to each MAX31855 directly (without the library or the
 * SPI hardware) and explains in plain words what the wires are doing. Turn the TCS off first.
 */
void tcs_run_wiring_test();

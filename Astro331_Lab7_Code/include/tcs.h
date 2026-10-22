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
  TCS_SUN_AVOID,      ///< Too hot. Louvers stay closed while the reaction wheel turns them away from the brightest light.
  TCS_COOLING,        ///< Too hot and pointed away from the light (or no light). Louvers open to vent heat.
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

/** Starts the thermocouple chip. Call once from setup(), BEFORE the SD card starts. */
bool tcs_init(uint8_t thermocoupleChipSelectPin);

/** Call this every pass through loop(). It reads the thermocouple and, if the TCS is on, runs the state machine. */
void tcs_update();

/*---------------------------------------------------------------------------------------------*/
// Turning the TCS on and off (ground-station commands use these)
/*---------------------------------------------------------------------------------------------*/

/** Turns automatic thermal control on. Starts in NORMAL. */
void tcs_enable();

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

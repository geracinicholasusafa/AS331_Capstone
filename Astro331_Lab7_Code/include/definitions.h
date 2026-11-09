/*---------------------------------------------------------------------------------------------*/
// Configuration:
/*---------------------------------------------------------------------------------------------*/
#define SD_CS_PIN 5  // Chip select pin for the microSD card on Thing Plus
// #define SD_CS_PIN 33  // Chip select pin for the microSD card on Feather Adalogger
#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  23
#define MAX31855_CS_PIN 27  // Chip select for the MAX31855 thermocouple amplifier #1 (shares SPI bus with SD)
#define MAX31855_2_CS_PIN 13 // Chip select for the second MAX31855 (thermocouple #2). Was the status LED pin; that LED is no longer used.

#define XBEE_RX 16  // ESP32 RX <- XBee DOUT
#define XBEE_TX 17  // ESP32 TX -> XBee DIN

#define SUN_SENSOR_PLUS_X_PIN   A0
#define SUN_SENSOR_MINUS_X_PIN  A2
#define SUN_SENSOR_PLUS_Y_PIN   A1
#define SUN_SENSOR_MINUS_Y_PIN  A3

#define MOTOR_VOLTAGE           5.05
#define CPR                     64
#define MOTOR_PWM_1_PIN         33
#define MOTOR_PWM_2_PIN         15
#define ENCODER_PIN_A           14
#define ENCODER_PIN_B           32

/*---------------------------------------------------------------------------------------------*/
// Thermal Control System (TCS) settings:
// Every number the TCS uses to make decisions lives here, so the team can tune it in one place.
// Temperatures are in degrees Celsius, times are in milliseconds.
/*---------------------------------------------------------------------------------------------*/

// --- Temperature band (the "comfortable" range) ---
// The "hold" temperature is set from the GUI's Hold button (command 19). Above it, the satellite turns its
// louvers away from the light and opens them; the hotter it gets, the wider they open.
#define TCS_TARGET_TEMP_C        30.0   // Hold temperature used until one is set from the GUI
#define TCS_MIN_HOLD_TEMP_C      15.0   // Lowest hold temperature the command accepts
#define TCS_MAX_HOLD_TEMP_C      40.0   // Highest hold temperature the command accepts (kept below the 45 C fault limit)
#define TCS_COOL_HYSTERESIS_C    1.0    // Keep venting until this far BELOW the hold temperature, so it does not flick on and off
#define TCS_COLD_MARGIN_C        4.0    // This far below the hold temperature counts as "too cold" (louvers shut)
// Closed-loop louver control: while venting, the louver opening is adjusted continuously to keep the panel
// AT the hold temperature, whatever heater power the team has set. Two parts add together:
//   "proportional" - opens more the further the panel is above the hold temperature, right now;
//   "integral"     - slowly builds up while the panel stays above the hold temperature (and shrinks while
//                    below), so it settles on exactly the opening that balances the heat coming in.
#define TCS_LOUVER_KP_PERCENT_PER_C   15.0  // Proportional: opening per degree above hold (PROPOSED, tune on the bench)
#define TCS_LOUVER_KI_PERCENT_PER_C_S 0.2   // Integral: opening added per degree above hold, every second (PROPOSED, tune)
#define TCS_LOUVER_MIN_STEP_PERCENT   2.0   // Ignore louver changes smaller than this, so the servo does not jitter on sensor noise
#define TCS_REGULATE_IDLE_MS          60000 // Louvers fully shut and panel below hold for this long -> stop regulating, back to NORMAL

// --- Safety limits (going past these puts the TCS into FAULT) ---
#define TCS_FAULT_HIGH_C         45.0   // Dangerously hot
#define TCS_FAULT_LOW_C          10.0   // Dangerously cold
#define TCS_FAULT_RATE_C_PER_S   200.0  // Temperature changing faster than this is not believable (was 1.0, 5.0, then 15.0, which tripped too easily)
#define TCS_RATE_WINDOW_MS       2000   // How far back we look when measuring how fast it is changing
#define TCS_SENSOR_FAIL_LIMIT    3      // This many bad thermocouple readings in a row -> sensor failure

// --- Timing ---
#define TCS_UPDATE_INTERVAL_MS   500    // How often the TCS reads sensors and decides what to do
#define TCS_TELEMETRY_INTERVAL_MS 1000  // How often a temperature line is sent for the live plot (1000 = once per second)
#define TCS_LOG_INTERVAL_MS      1000   // How often a row is written to the SD card CSV log (1000 = once per second)
#define TCS_LOG_FILE_PREFIX      "TCS_log" // Log files are named TCS_log_data001.csv, TCS_log_data002.csv, ... (a new one each time logging starts)
#define TCS_LOUVER_MAX_OPEN_MS   0      // If above 0: louvers may stay open this long while cooling, then they close.
                                        // 0 = no time limit. Off because the team sets the internal heat by hand: if the heat
                                        // keeps coming, closing the louvers would just overheat the panel. Over-cooling is
                                        // still prevented, because the louvers close once the panel is below the hold temperature.
#define TCS_RECOVERY_HOLD_MS     5000   // How long RECOVERY waits with louvers at rest before returning to NORMAL (PROPOSED)
#define TCS_SUN_AVOID_TIMEOUT_MS 60000  // If turning away from the Sun takes this long, vent anyway (heat must escape) (PROPOSED)

// --- Louvers (percentages: 0 = closed, 100 = fully open) ---
#define TCS_LOUVER_CLOSED_PERCENT   0.0
#define TCS_LOUVER_OPEN_PERCENT     100.0
#define TCS_LOUVER_NOMINAL_PERCENT  0.0   // Where the louvers rest when nothing special is happening

// --- Louver servo: Tower Pro SG92R micro servo ---
// A servo is told where to point by a repeating pulse (20 ms apart); the pulse LENGTH sets the angle.
// Power: the servo needs 5 V (the board's USB/5V pin), NOT 3.3 V, and its ground must be shared with the board.
#define TCS_LOUVER_SERVO_PIN        4     // Signal wire GPIO for the single louver servo (confirmed by the team)
#define TCS_SERVO_PULSE_MIN_US      500   // Pulse length at 0 degrees, in microseconds (SG92R datasheet: about 500)
#define TCS_SERVO_PULSE_MAX_US      2400  // Pulse length at 180 degrees, in microseconds (SG92R datasheet: about 2400)
#define TCS_LOUVER_CLOSED_ANGLE_DEG 180.0 // Servo angle when the louvers are closed (confirmed on the hardware)
#define TCS_LOUVER_OPEN_ANGLE_DEG   90.0  // Servo angle when the louvers are fully open (confirmed on the hardware)
#define TCS_SERVO_LEDC_CHANNEL      0     // ESP32 PWM channel for the servo. Channels 15 and 14 are used by the wheel motor driver, so keep this low.
#define TCS_SERVO_PWM_BITS          14    // Resolution of the servo pulse timing (14 bits = very fine steps)

// --- Peltier module (5 V, 1 A) used ONLY as a test heater for the panel ---
// It is switched on and off very fast (PWM) by a transistor/MOSFET; the percentage is how much of the time it is on.
// It only heats. Wire it so the side touching the panel is the HOT side (if the panel gets colder, swap its two power wires).
// Power: use a separate 5 V supply (1 A or more), never a board pin, and join its ground to the board's ground.
// Put a heatsink on the side NOT touching the panel.
#define TCS_PELTIER_PIN             12    // Control wire GPIO that drives the switch (PROPOSED: one of the last free pins; confirm wiring)
                                          // Pin 12 is read at boot, so the switch's input should be pulled to ground (a 10k resistor works).
#define TCS_PELTIER_LEDC_CHANNEL    4     // ESP32 PWM channel (0 = servo, 14 and 15 = wheel motor driver)
#define TCS_PELTIER_PWM_FREQ_HZ     1000  // How fast it switches on and off, times per second
#define TCS_PELTIER_PWM_BITS        10    // Resolution of the on/off timing
#define TCS_PELTIER_MAX_POWER_PERCENT 100.0 // Never use more than this much power (lower it for a gentler heater)
#define TCS_PELTIER_MIN_TARGET_C    15.0  // Lowest target temperature the command accepts
#define TCS_PELTIER_MAX_TARGET_C    40.0  // Highest target temperature the command accepts (kept below the 45 C fault limit)
#define TCS_PELTIER_KP_PERCENT_PER_C 15.0 // Heater power per degree below target (10 C below -> 100%). Bigger = pushes harder (PROPOSED, tune)
#define TCS_PELTIER_KI_PERCENT_PER_C_S 0.3 // Extra power that builds up each second while below target, to close the last gap (PROPOSED, tune)

// --- Sun sensor / turning the louvers away from the Sun ---
// To vent heat, the louver face should point AWAY from the brightest light (toward cold space).
#define TCS_SUN_MIN_LIGHT_COUNTS   600    // Sum of the 4 sensors (each 0 to 4095). Less than this = "no bright light seen", so no need to turn (PROPOSED, tune with a lamp)
#define TCS_SUN_READS_TO_AVERAGE   5      // Readings averaged per sensor to smooth out noise
#define TCS_LOUVER_FACE_ANGLE_DEG  180.0  // Direction the louvers face, in the sun-sensor frame (0 = +X, 90 = +Y, 180 = -X, 270 = -Y). The louvers are on the -X face, solar panels on +X (confirmed by the team).
#define TCS_VENT_ALIGN_TOL_DEG     15.0   // Within this many degrees of "directly away from the light" counts as aligned (PROPOSED)
#define TCS_VENT_LOST_TOL_DEG      30.0   // While venting, drifting further off than this is reported as SUN_AVOID (louvers keep venting while it turns back)
#define TCS_LIGHT_DIRECTION_OFFSET_DEG 0.0 // Added to the measured light direction to line the sensors up with the body axes.
                                          // Normally left at 0 and set from the GUI's "Calibrate" button instead (command 20, saved in the board's memory).

// --- Turning: "look, turn, settle, look again" ---
// With the satellite still, the light sensors say how far off the louvers are. The satellite then turns by that
// angle, measured with the IMU gyroscope (the light sensors are ignored mid-turn), stops, settles, and looks again.
#define TCS_TURN_THROTTLE          0.40   // Wheel throttle during a turn (must be enough to start the motor; lower = slower, gentler)
#define TCS_TURN_STOP_EARLY_DEG    5.0    // Stop the wheel this many degrees before the end, because the satellite coasts a little (tune on the bench)
#define TCS_TURN_SETTLE_MS         2000   // Wait this long with the wheel stopped before looking at the light again
#define TCS_TURN_TIMEOUT_MS        20000  // A single turn taking longer than this is stopped (then it looks again)
#define TCS_TURN_WRONG_WAY_DEG     20.0   // Turned this far the WRONG way -> stop and report (flip TCS_WHEEL_DIRECTION or TCS_GYRO_DIRECTION)
#define TCS_GYRO_READ_INTERVAL_MS  20     // How often the gyroscope is read during a turn (20 ms = 50 times a second)
#define TCS_GYRO_DIRECTION         1      // +1 or -1. Flip if the gyro counts the turn backwards (the turn never ends, or "wrong way" appears while it turns the right way)

// --- Reaction wheel ---
#define TCS_MAX_WHEEL_RPM          400.0  // Team limit: the wheel must never spin faster than this
#define TCS_WHEEL_MAX_THROTTLE     0.95   // Full throttle gives about 421 RPM, so 0.95 keeps us just under 400
#define TCS_WHEEL_DIRECTION        1      // +1 or -1. Flip this if the satellite turns the WRONG way (toward the light instead of away)



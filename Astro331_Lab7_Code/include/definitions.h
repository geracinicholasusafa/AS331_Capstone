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
#define TCS_TARGET_TEMP_C        30.0   // The temperature the satellite tries to settle at
#define TCS_BAND_LOW_C           26.0   // Colder than this -> the satellite is too cold, start heating
#define TCS_BAND_HIGH_C          34.0   // Hotter than this -> the satellite is too hot, start cooling

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
#define TCS_LOUVER_MAX_OPEN_MS   120000 // Louvers may stay open this long while cooling, then they close (PROPOSED, not confirmed)
#define TCS_RECOVERY_HOLD_MS     5000   // How long RECOVERY waits with louvers at rest before returning to NORMAL (PROPOSED)
#define TCS_SUN_AVOID_TIMEOUT_MS 60000  // If turning away from the Sun takes this long, vent anyway (heat must escape) (PROPOSED)

// --- Louvers (percentages: 0 = closed, 100 = fully open) ---
#define TCS_LOUVER_CLOSED_PERCENT   0.0
#define TCS_LOUVER_OPEN_PERCENT     100.0
#define TCS_LOUVER_NOMINAL_PERCENT  0.0   // Where the louvers rest when nothing special is happening

// --- Louver servo: Tower Pro SG92R micro servo ---
// A servo is told where to point by a repeating pulse (20 ms apart); the pulse LENGTH sets the angle.
// Power: the servo needs 5 V (the board's USB/5V pin), NOT 3.3 V, and its ground must be shared with the board.
#define TCS_LOUVER_SERVO_PIN        4     // Signal wire GPIO (PROPOSED: the only fully free output pin; confirm wiring)
#define TCS_SERVO_PULSE_MIN_US      500   // Pulse length at 0 degrees, in microseconds (SG92R datasheet: about 500)
#define TCS_SERVO_PULSE_MAX_US      2400  // Pulse length at 180 degrees, in microseconds (SG92R datasheet: about 2400)
#define TCS_LOUVER_CLOSED_ANGLE_DEG 0.0   // Servo angle when the louvers are closed (PROPOSED, depends on how it is mounted)
#define TCS_LOUVER_OPEN_ANGLE_DEG   90.0  // Servo angle when the louvers are fully open (PROPOSED, depends on how it is mounted)
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
#define TCS_LOUVER_FACE_ANGLE_DEG  0.0    // Direction (0 to 360 deg, in the sun-sensor frame) the louvers face (PROPOSED, confirm)
#define TCS_VENT_ALIGN_TOL_DEG     15.0   // Within this many degrees of "directly away from the light" counts as aligned (PROPOSED)
#define TCS_VENT_LOST_TOL_DEG      30.0   // While venting, drifting further off than this closes the louvers and turns again

// --- Reaction wheel ---
#define TCS_MAX_WHEEL_RPM          400.0  // Team limit: the wheel must never spin faster than this
#define TCS_WHEEL_MAX_THROTTLE     0.95   // Full throttle gives about 421 RPM, so 0.95 keeps us just under 400
#define TCS_TURN_MAX_THROTTLE      0.30   // Gentlest throttle used while turning the satellite (PROPOSED, tune on the bench)
#define TCS_TURN_GAIN_PER_DEG      0.01   // Throttle per degree of pointing error (30 deg off -> 0.30)
#define TCS_WHEEL_DIRECTION        1      // +1 or -1. Flip this if the satellite turns the WRONG way (toward the light instead of away)



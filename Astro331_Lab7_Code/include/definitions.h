/*---------------------------------------------------------------------------------------------*/
// Configuration:
/*---------------------------------------------------------------------------------------------*/
#define SD_CS_PIN 5  // Chip select pin for the microSD card on Thing Plus
// #define SD_CS_PIN 33  // Chip select pin for the microSD card on Feather Adalogger
#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  23
#define MAX31855_CS_PIN 27  // Chip select for the MAX31855 thermocouple amplifier (shares SPI bus with SD)

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
#define TCS_FAULT_RATE_C_PER_S   1.0    // Temperature changing faster than this is not believable
#define TCS_RATE_WINDOW_MS       2000   // How far back we look when measuring how fast it is changing
#define TCS_SENSOR_FAIL_LIMIT    3      // This many bad thermocouple readings in a row -> sensor failure

// --- Timing ---
#define TCS_UPDATE_INTERVAL_MS   500    // How often the TCS reads sensors and decides what to do
#define TCS_LOUVER_MAX_OPEN_MS   120000 // Louvers may stay open this long while cooling, then they close (PROPOSED, not confirmed)
#define TCS_RECOVERY_HOLD_MS     5000   // How long RECOVERY waits with louvers at rest before returning to NORMAL (PROPOSED)
#define TCS_SUN_SEARCH_TIMEOUT_MS 60000 // Give up looking for the Sun after this long -> FAULT (PROPOSED)

// --- Louvers (servo hardware not chosen yet; these are percentages, 0 = closed, 100 = fully open) ---
#define TCS_LOUVER_CLOSED_PERCENT   0.0
#define TCS_LOUVER_OPEN_PERCENT     100.0
#define TCS_LOUVER_NOMINAL_PERCENT  0.0   // Where the louvers rest when nothing special is happening

// --- Sun sensor / Sun search ---
#define TCS_SUN_MIN_LIGHT_COUNTS   600    // Sum of the 4 sensors (each 0 to 4095). Less than this = "no Sun seen" (PROPOSED, tune with a lamp)
#define TCS_SUN_READS_TO_AVERAGE   5      // Readings averaged per sensor to smooth out noise
#define TCS_THERMAL_SURFACE_ANGLE_DEG 0.0 // Direction (0 to 360 deg, in the sun-sensor frame) the Mylar thermal surface faces (PROPOSED, confirm)
#define TCS_SUN_ALIGN_TOL_DEG      15.0   // Closer than this to the Sun counts as "aligned" (PROPOSED)
#define TCS_SUN_LOST_TOL_DEG       30.0   // While heating, drifting further than this sends us back to searching

// --- Reaction wheel ---
#define TCS_MAX_WHEEL_RPM          400.0  // Team limit: the wheel must never spin faster than this
#define TCS_WHEEL_MAX_THROTTLE     0.95   // Full throttle gives about 421 RPM, so 0.95 keeps us just under 400
#define TCS_SUN_SEARCH_MAX_THROTTLE 0.30  // Gentlest throttle used while turning toward the Sun (PROPOSED, tune on the bench)
#define TCS_SUN_SEARCH_GAIN_PER_DEG 0.01  // Throttle per degree of pointing error (30 deg off -> 0.30)
#define TCS_WHEEL_DIRECTION        1      // +1 or -1. Flip this if the satellite turns AWAY from the Sun



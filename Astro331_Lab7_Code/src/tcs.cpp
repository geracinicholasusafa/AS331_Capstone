/**
 * @file tcs.cpp
 * @brief Thermal Control System (TCS): reads the panel thermocouple and runs the thermal state machine.
 *
 * How to read this file (top to bottom):
 *   1. Settings and "memory" the TCS keeps between calls
 *   2. Small helpers: print a message, move the louvers, spin the wheel
 *   3. Sensors: thermocouple, Sun sensor, wheel speed
 *   4. Safety checks (when to give up and go to FAULT)
 *   5. The state machine: what to do in each state
 *   6. The public functions other files call (declared in tcs.h)
 */

#include <Arduino.h>
#include <math.h>
#include <Adafruit_MAX31855.h>            // Reads the thermocouple through the MAX31855 chip
#include <TB9051FTGMotorCarrier.h>        // Reaction wheel motor driver
#include <ESP32Encoder.h>                 // Reaction wheel speed sensor
#include <ICM_20948.h>                    // IMU: its gyroscope measures how far the satellite has turned
#include <Preferences.h>                  // Saves the light-sensor calibration in the board's memory (kept after power-off)
#include "definitions.h"
#include "sd_functions.h"                 // Creates the CSV log file on the SD card
#include "tcs.h"

/*---------------------------------------------------------------------------------------------*/
// 1. Things that live in main.cpp and that the TCS borrows
/*---------------------------------------------------------------------------------------------*/
extern HardwareSerial Xbee;               // Radio link to the ground station
extern TB9051FTGMotorCarrier driver;      // Reaction wheel motor driver
extern ESP32Encoder enc;                  // Reaction wheel encoder
extern ICM_20948_I2C imu_sensor;          // IMU (set up in main.cpp)

/*---------------------------------------------------------------------------------------------*/
// 1. What the TCS remembers between calls (all private to this file)
/*---------------------------------------------------------------------------------------------*/
static Adafruit_MAX31855* thermocouple = nullptr;   // The thermocouple chip for thermocouple 1 (the panel; used for control)
static Adafruit_MAX31855* thermocouple2 = nullptr;  // The second thermocouple chip (reported only, not used for control)
static float temp2C = NAN;                          // Latest thermocouple 2 temperature (NAN = no valid reading)

static bool           tcs_enabled = false;          // Is automatic control switched on?
static float          holdTempC = TCS_TARGET_TEMP_C; // Above this the satellite vents (set from the GUI Hold button, command 19)
static TcsState       state = TCS_NORMAL;           // Which step we are in
static TcsFaultReason faultReason = TCS_FAULT_NONE; // Why we are in FAULT (if we are)
static uint32_t       stateStartMs = 0;             // When we entered the current step
static uint32_t       lastUpdateMs = 0;             // When the TCS last did its work

static float panelTempC = NAN;        // Latest panel temperature (NAN = no valid reading)
static float chipTempC = NAN;         // Latest MAX31855 chip temperature
static uint8_t badReadCount = 0;      // Thermocouple failures in a row
static uint8_t lastFaultBits = 0;     // What the MAX31855 said was wrong on the last bad read (1=open, 2=short to GND, 4=short to VCC)

static float    rateReferenceTempC = NAN;   // Temperature at the start of the current rate window
static uint32_t rateReferenceMs = 0;        // Time at the start of the current rate window
static float    tempRateCPerS = 0.0;        // Latest warming/cooling speed

static float louverPercent = TCS_LOUVER_NOMINAL_PERCENT;  // Where we last told the louvers to go
static float wheelThrottle = 0.0;                          // Last throttle sent to the reaction wheel
static float wheelRpm = 0.0;                               // Latest measured wheel speed
static int64_t  lastEncoderCount = 0;
static uint32_t lastEncoderMs = 0;

static bool     telemetryOn = true;   // Is the live temperature stream switched on?
static uint32_t lastTelemetryMs = 0;  // When the last stream line was sent

static float ventErrorDeg = NAN;     // How far the louvers are from pointing directly away from the brightest light
static float lightDirectionDeg = NAN; // Where the brightest light comes from (0 = +X, 90 = +Y, 180 = -X, 270 = -Y). NAN if too dark
static float rawLightDirectionDeg = NAN; // Light direction straight from the sensors, before the calibration offset
static float lightOffsetDeg = TCS_LIGHT_DIRECTION_OFFSET_DEG; // Calibration: added to the raw direction (command 20)
static float lightTotal = 0;          // Sum of the four light sensors (0 to about 16000); bigger = brighter
static float lightPlusX = 0, lightMinusX = 0, lightPlusY = 0, lightMinusY = 0;  // Each light sensor (0 to 4095)
static bool  ventAnyway = false;      // True once turning away took too long: vent without turning for TCS_TURN_RETRY_MS
static uint32_t ventAnywaySinceMs = 0; // When that started

/*---------------------------------------------------------------------------------------------*/
// 2. Small helpers
/*---------------------------------------------------------------------------------------------*/

/** Prints a message to both the USB cable and the radio. */
static void say(const String& message) {
  Serial.println(message);
  Xbee.println(message);
}

static bool  servoStarted = false;     // Has the servo been switched on yet? (it is only started on the first move)
static float servoLastPercent = -1.0;  // Where we last sent the servo, so we only send a new pulse when it changes

/**
 * Moves the louvers. 0 = closed, 100 = fully open.
 *
 * The Tower Pro SG92R servo is told where to point by a pulse sent every 20 ms (50 times a second).
 * A 0.5 ms pulse means 0 degrees and a 2.4 ms pulse means 180 degrees; in between is in proportion.
 * We turn the percentage into an angle, the angle into a pulse length, and the pulse length into the
 * number the ESP32's PWM hardware wants.
 *
 * The servo is not started until the first time it is asked to move, so it does not twitch at power-up.
 *
 * Tiny changes (under TCS_LOUVER_MIN_STEP_PERCENT) are ignored so the servo does not buzz back and forth on
 * thermocouple noise, except when going fully closed/open or when exactAlways is true (moving by hand).
 */
static void set_louvers(float openPercent, bool exactAlways = false) {
  float wanted = constrain(openPercent, TCS_LOUVER_CLOSED_PERCENT, TCS_LOUVER_OPEN_PERCENT);
  if (servoStarted) {
    if (wanted == servoLastPercent) return;                       // already there, nothing to send
    bool atAnEnd = (wanted == TCS_LOUVER_CLOSED_PERCENT || wanted == TCS_LOUVER_OPEN_PERCENT);
    if (!exactAlways && !atAnEnd && fabs(wanted - servoLastPercent) < TCS_LOUVER_MIN_STEP_PERCENT) return;
  }
  louverPercent = wanted;

  // Percentage -> servo angle (the closed and open angles are settings in definitions.h).
  float fractionOpen = (louverPercent - TCS_LOUVER_CLOSED_PERCENT) / (TCS_LOUVER_OPEN_PERCENT - TCS_LOUVER_CLOSED_PERCENT);
  float angleDeg = TCS_LOUVER_CLOSED_ANGLE_DEG + fractionOpen * (TCS_LOUVER_OPEN_ANGLE_DEG - TCS_LOUVER_CLOSED_ANGLE_DEG);

  // Angle -> pulse length in microseconds.
  float pulseUs = TCS_SERVO_PULSE_MIN_US + (angleDeg / 180.0) * (TCS_SERVO_PULSE_MAX_US - TCS_SERVO_PULSE_MIN_US);

  // Pulse length -> PWM number. One full 20,000 microsecond cycle equals the maximum number.
  const uint32_t fullScale = (1UL << TCS_SERVO_PWM_BITS) - 1;
  uint32_t duty = (uint32_t)((pulseUs / 20000.0) * fullScale);

  if (!servoStarted) {
    ledcSetup(TCS_SERVO_LEDC_CHANNEL, 50, TCS_SERVO_PWM_BITS);   // 50 pulses per second
    ledcAttachPin(TCS_LOUVER_SERVO_PIN, TCS_SERVO_LEDC_CHANNEL);
    servoStarted = true;
  }
  ledcWrite(TCS_SERVO_LEDC_CHANNEL, duty);
  servoLastPercent = louverPercent;
}

/** Spins the reaction wheel. -1 to +1, but never past the team's 400 RPM safety cap. */
static void set_wheel(float throttle) {
  wheelThrottle = constrain(throttle, -TCS_WHEEL_MAX_THROTTLE, TCS_WHEEL_MAX_THROTTLE);
  driver.setOutput(wheelThrottle);
}

/** Moves to a new step and tells the ground station. */
static void enter_state(TcsState next) {
  if (next != state) {
    say(String("[TCS] ") + tcs_state_name(state) + " -> " + tcs_state_name(next));
  }
  state = next;
  stateStartMs = millis();
}

/** Time spent in the current step, in milliseconds. */
static uint32_t time_in_state_ms() {
  return millis() - stateStartMs;
}

/** Turns an angle into the range -180 to +180 so "350 degrees off" becomes "10 degrees off the other way". */
static float wrap_to_180(float degrees) {
  while (degrees > 180.0) degrees -= 360.0;
  while (degrees < -180.0) degrees += 360.0;
  return degrees;
}

/*---------------------------------------------------------------------------------------------*/
// 3. Sensors
/*---------------------------------------------------------------------------------------------*/

/**
 * Reads the thermocouple. If the chip reports a problem (wire unplugged, shorted) the reading is
 * thrown away and we count it as a failure. Also works out how fast the temperature is changing.
 */
static void read_thermocouple() {
  if (thermocouple == nullptr) return;

  float reading = thermocouple->readCelsius();   // NAN when the chip reports a fault
  chipTempC = thermocouple->readInternal();

  if (isnan(reading)) {
    if (badReadCount < 255) badReadCount++;
    lastFaultBits = thermocouple->readError();   // ask the chip what is wrong
    panelTempC = NAN;
    return;
  }
  badReadCount = 0;
  lastFaultBits = 0;
  panelTempC = reading;

  // Work out the warming/cooling speed once per window (a short window would just measure noise).
  uint32_t now = millis();
  if (isnan(rateReferenceTempC)) {
    rateReferenceTempC = panelTempC;
    rateReferenceMs = now;
  } else if (now - rateReferenceMs >= TCS_RATE_WINDOW_MS) {
    tempRateCPerS = (panelTempC - rateReferenceTempC) / ((now - rateReferenceMs) / 1000.0f);
    rateReferenceTempC = panelTempC;
    rateReferenceMs = now;
  }
}

/**
 * Reads the second thermocouple. If the chip reports a problem the reading becomes "no reading" (NAN).
 * This is for information only: a problem here does not stop the thermal control.
 */
static void read_thermocouple2() {
  if (thermocouple2 == nullptr) return;
  temp2C = thermocouple2->readCelsius();   // NAN when the chip reports a fault
}

/** Turns the chip's fault code into a sentence that says what to check. */
static const char* describe_thermocouple_fault(uint8_t bits) {
  if (bits == 0) return "no fault flags";
  if (bits == 7) return "all fault flags set: chip not answering, check the 3.3V, GND, CLK, DO and CS wires";
  if (bits & 1)  return "open circuit: thermocouple wire not connected or loose in the T+/T- screw terminals";
  if (bits & 2)  return "thermocouple shorted to GND";
  return "thermocouple shorted to VCC";
}

/**
 * Looks at the four light sensors, finds the direction of the brightest light, and works out how far
 * the louvers are from pointing directly AWAY from it (the best direction for venting heat).
 * Sets ventErrorDeg to NAN when there is not enough light to trust (no bright light seen).
 * Uses the same Sun-direction math as Lab 7 Test B.
 */
static void read_sun_sensor() {
  float plusX = 0, minusX = 0, plusY = 0, minusY = 0;
  for (int i = 0; i < TCS_SUN_READS_TO_AVERAGE; i++) {   // average a few readings to smooth noise
    plusX  += analogRead(SUN_SENSOR_PLUS_X_PIN);
    minusX += analogRead(SUN_SENSOR_MINUS_X_PIN);
    plusY  += analogRead(SUN_SENSOR_PLUS_Y_PIN);
    minusY += analogRead(SUN_SENSOR_MINUS_Y_PIN);
  }
  plusX /= TCS_SUN_READS_TO_AVERAGE;
  minusX /= TCS_SUN_READS_TO_AVERAGE;
  plusY /= TCS_SUN_READS_TO_AVERAGE;
  minusY /= TCS_SUN_READS_TO_AVERAGE;

  // Remember the raw readings so the status line, live stream, log and 3D view can show them.
  lightPlusX = plusX; lightMinusX = minusX; lightPlusY = plusY; lightMinusY = minusY;

  float totalLight = plusX + minusX + plusY + minusY;
  lightTotal = totalLight;
  if (totalLight < TCS_SUN_MIN_LIGHT_COUNTS) {
    ventErrorDeg = NAN;                                  // too dark: no bright light to avoid
    lightDirectionDeg = NAN;
    rawLightDirectionDeg = NAN;
    return;
  }

  float sunX = (plusX - minusX) / totalLight;
  float sunY = (plusY - minusY) / totalLight;
  rawLightDirectionDeg = atan2(sunY, sunX) * RAD_TO_DEG;
  // Add the calibration offset so the direction lines up with the satellite's faces, then keep it 0 to 360.
  float sunDirectionDeg = rawLightDirectionDeg + lightOffsetDeg;
  while (sunDirectionDeg < 0) sunDirectionDeg += 360.0;
  while (sunDirectionDeg >= 360.0) sunDirectionDeg -= 360.0;
  lightDirectionDeg = sunDirectionDeg;

  // The ideal louver direction is exactly opposite the light (180 degrees away from it).
  float idealLouverDirectionDeg = sunDirectionDeg + 180.0;
  ventErrorDeg = wrap_to_180(TCS_LOUVER_FACE_ANGLE_DEG - idealLouverDirectionDeg);
}

/*
 * Pointing the louvers away from the light: "look, turn, settle, look again", steered by the gyroscope.
 *
 * Why the wheel is never simply switched off mid-move: to turn the satellite, the wheel spins the other way and
 * holds that "spin" (angular momentum). If the motor is switched off, the wheel's spin leaks back into the
 * satellite and it keeps turning past the target. So instead the wheel is always steered by the IMU gyroscope
 * (which measures how fast the satellite is spinning):
 *   - TURNING:  aim for a spin speed that gets smaller as the satellite nears the target, so it slows down and
 *               arrives gently. The angle turned is added up from the gyroscope; light sensors are ignored.
 *   - SETTLING: aim for zero spin, and wait until the satellite has been still for TCS_TURN_SETTLE_MS.
 *   - IDLE:     still aim for zero spin (hold still), then look at the light; if it is off, turn again.
 *
 * How the wheel is steered: the satellite's spin speed follows the wheel's speed (the faster the wheel, the faster
 * the satellite turns the other way). So every 20 ms, if the satellite spins faster than wanted, the wheel
 * throttle is nudged one way; slower than wanted, the other way. It settles at the throttle that gives the
 * wanted spin speed.
 */
enum PointingStep { POINT_IDLE, POINT_TURNING, POINT_SETTLING };

static PointingStep pointingStep = POINT_IDLE;
static bool     spinControlOn = false;    // Is the gyroscope steering the wheel? (off = wheel stopped, e.g. FAULT)
static float    wheelCommand = 0;         // Throttle the spin controller wants (before TCS_WHEEL_DIRECTION). + turns the satellite clockwise (-)
static float    spinRateDegPerS = 0;      // Latest satellite spin speed from the gyroscope (+ = counter-clockwise seen from +Z)
static float    turnTargetDeg = 0;        // How far this turn should go (+ = counter-clockwise seen from +Z)
static float    turnDoneDeg = 0;          // How far we have turned so far, added up from the gyroscope
static uint32_t turnStartedMs = 0;
static uint32_t lastGyroMs = 0;           // When the gyroscope was last read (to know how much time passed)
static uint32_t settleStartedMs = 0;      // When the satellite last became still
static bool     turnWentWrongWay = false; // Set when the gyro saw the satellite turning the opposite way
static uint32_t settleBeganMs = 0;        // When settling began (settleStartedMs restarts every time it moves; this does not)
static uint32_t wheelAtLimitSinceMs = 0;  // When the wheel reached its speed limit (0 = not at the limit)
static bool     wheelSaturatedReported = false;

/** Wheel steered to zero spin; wait for the satellite to be still before looking at the light again. */
static void begin_settling() {
  pointingStep = POINT_SETTLING;
  settleStartedMs = millis();
  settleBeganMs = settleStartedMs;
}

/** Stops the wheel completely (used for FAULT and when the TCS is switched off). */
static void stop_turning() {
  spinControlOn = false;
  wheelCommand = 0;
  set_wheel(0);
  pointingStep = POINT_IDLE;
}

/** Keeps the satellite still with the wheel (no turning, no looking at the light). */
static void hold_still() {
  spinControlOn = true;
  if (pointingStep == POINT_TURNING) begin_settling();   // abandon the turn, just stop spinning
}

/**
 * Starts a turn that should line the louvers up, using the latest light reading.
 * If the louvers are ventErrorDeg ahead of the ideal direction, the satellite must turn back by that much.
 */
static void start_turn() {
  turnTargetDeg = -ventErrorDeg;
  // If the wheel is already spinning fast, the short way round could push it to its limit (a turn toward -
  // speeds the wheel up in +). Going the long way round instead slows the wheel down, because the mount's
  // friction soaks up some of the push. So unload a busy wheel by going the long way.
  if (wheelCommand > TCS_WHEEL_UNLOAD_THROTTLE && turnTargetDeg < 0) {
    turnTargetDeg += 360.0;
    say("[TCS] Wheel is spinning fast: going the long way round to slow it down.");
  } else if (wheelCommand < -TCS_WHEEL_UNLOAD_THROTTLE && turnTargetDeg > 0) {
    turnTargetDeg -= 360.0;
    say("[TCS] Wheel is spinning fast: going the long way round to slow it down.");
  }
  turnDoneDeg = 0;
  turnStartedMs = millis();
  turnWentWrongWay = false;
  // Kick: jump the throttle past the point where the motor starts to move, in the right direction.
  float kickDirection = (turnTargetDeg < 0) ? 1.0 : -1.0;
  if (wheelCommand * kickDirection < TCS_TURN_KICK_THROTTLE) wheelCommand = kickDirection * TCS_TURN_KICK_THROTTLE;
  spinControlOn = true;
  pointingStep = POINT_TURNING;
  say(String("[TCS] Turning ") + String(turnTargetDeg, 0) + " deg to point the louvers away from the light.");
}

static float headingDeg = 0;              // Which way the satellite faces, added up from the gyroscope since logging started
                                          // (not wrapped to 0-360, so a plot has no jumps; + = counter-clockwise seen from +Z)
static float gyroBiasDegPerS = 0;         // The gyroscope's small reading when perfectly still (subtracted so the heading does not drift)
static float gyroBiasSum = 0;
static float gyroBiasSquares = 0;        // (for checking the readings were steady)
static int   gyroBiasSamples = 0;

/**
 * Reads the gyroscope every TCS_GYRO_READ_INTERVAL_MS (called on every pass of loop(), TCS on or off).
 * Updates the spin speed and the heading. Returns true when a new reading was taken, with the time since
 * the last one in *seconds.
 * The first TCS_GYRO_BIAS_SAMPLES readings after power-up are averaged to measure the gyro's "still" reading,
 * so keep the satellite still for the first few seconds after power-up.
 */
static bool read_gyro_if_due(float *seconds) {
  uint32_t now = millis();
  if (lastGyroMs != 0 && now - lastGyroMs < TCS_GYRO_READ_INTERVAL_MS) return false;
  float dt = (lastGyroMs == 0) ? 0.0 : (now - lastGyroMs) / 1000.0f;
  if (dt > 0.2f) dt = 0.0f;                               // a long gap (e.g. a blocking command) is not one real time step
  lastGyroMs = now;

  imu_sensor.getAGMT();                                   // read the IMU
  float raw = imu_sensor.gyrZ() * TCS_GYRO_DIRECTION;
  if (gyroBiasSamples < TCS_GYRO_BIAS_SAMPLES) {          // still measuring the "still" reading
    gyroBiasSum += raw;
    gyroBiasSquares += raw * raw;
    gyroBiasSamples++;
    if (gyroBiasSamples == TCS_GYRO_BIAS_SAMPLES) {
      float mean = gyroBiasSum / TCS_GYRO_BIAS_SAMPLES;
      float spread = sqrt(max(0.0f, gyroBiasSquares / TCS_GYRO_BIAS_SAMPLES - mean * mean));
      // Only trust it if the satellite was really still: a small, steady reading.
      if (fabs(mean) <= TCS_GYRO_MAX_BIAS_DEG_S && spread <= TCS_GYRO_STILL_SPREAD_DEG_S) {
        gyroBiasDegPerS = mean;
        say(String("[TCS] Gyro zero measured: ") + String(mean, 2) + " deg/s.");
      } else {
        gyroBiasDegPerS = 0;
        say(String("[TCS] Satellite was moving at power-up (gyro read ") + String(mean, 1) + " deg/s), so the gyro zero was not "
            "measured. Using 0. For the best heading, power up with the satellite still.");
      }
    }
  }
  spinRateDegPerS = raw - gyroBiasDegPerS;
  headingDeg += spinRateDegPerS * dt;
  *seconds = dt;
  return true;
}

/**
 * The spin controller. Runs after each new gyroscope reading (every TCS_GYRO_READ_INTERVAL_MS), because the
 * angle turned is added up from many quick readings.
 */
static void track_turn(float seconds) {
  if (!spinControlOn) return;
  uint32_t now = millis();

  // What spin speed do we want right now?
  float wantedRate = 0;                                   // settling / holding: stand still
  float remaining = 0;
  if (pointingStep == POINT_TURNING) {
    turnDoneDeg += spinRateDegPerS * seconds;
    remaining = turnTargetDeg - turnDoneDeg;
    // Slow down near the end: wanted speed shrinks with the angle left, up to a gentle maximum.
    wantedRate = constrain(remaining * TCS_TURN_RATE_PER_DEG, -TCS_TURN_MAX_RATE_DEG_S, TCS_TURN_MAX_RATE_DEG_S);
  }

  // Change the wheel speed. The wheel only pushes the satellite while its speed is CHANGING, so the bigger the
  // spin-speed error, the faster the wheel speed changes (= the harder the push), enough to overcome the mount's
  // friction. Spinning too fast in + -> push the throttle + (that turns the satellite -), and the other way round.
  float rateError = spinRateDegPerS - wantedRate;
  bool closeEnough = (pointingStep != POINT_TURNING) && fabs(rateError) < TCS_HOLD_DEADBAND_DEG_S;
  if (!closeEnough) {                                     // when holding still, ignore only tiny gyro wobble
    wheelCommand += TCS_SPIN_GAIN * rateError * seconds;
  } else {
    // Holding still: slowly let the wheel spin down toward zero. Slowly enough that friction keeps the satellite
    // from moving, so the wheel does not creep up to the 400 RPM limit after each turn.
    float bleed = TCS_WHEEL_BLEED_PER_S * seconds;
    wheelCommand = (fabs(wheelCommand) <= bleed) ? 0 : wheelCommand - (wheelCommand > 0 ? bleed : -bleed);
  }
  wheelCommand = constrain(wheelCommand, -TCS_SPIN_MAX_THROTTLE, TCS_SPIN_MAX_THROTTLE);
  set_wheel(wheelCommand * TCS_WHEEL_DIRECTION);

  // Wheel at its speed limit: it cannot push any harder that way, so the satellite may drift. Report it once.
  if (fabs(wheelCommand) >= TCS_SPIN_MAX_THROTTLE - 0.01f) {
    if (wheelAtLimitSinceMs == 0) wheelAtLimitSinceMs = now;
    if (!wheelSaturatedReported && now - wheelAtLimitSinceMs > TCS_WHEEL_SATURATED_MS) {
      wheelSaturatedReported = true;
      say("[TCS] Reaction wheel at its speed limit: it cannot push harder that way. Next turn goes the long way round to unload it.");
    }
  } else {
    wheelAtLimitSinceMs = 0;
    wheelSaturatedReported = false;
  }

  bool still = fabs(spinRateDegPerS) < TCS_STILL_RATE_DEG_S;
  if (pointingStep == POINT_TURNING) {
    bool rightWay = (turnDoneDeg * turnTargetDeg) >= 0;
    if (fabs(remaining) < TCS_TURN_DONE_DEG && still) {
      begin_settling();                                   // arrived and nearly stopped
    } else if (!rightWay && fabs(turnDoneDeg) > TCS_TURN_WRONG_WAY_DEG) {
      turnWentWrongWay = true;
      say("[TCS] Satellite is turning the wrong way. Flip TCS_WHEEL_DIRECTION (or TCS_GYRO_DIRECTION) in definitions.h.");
      stop_turning();
    } else if (now - turnStartedMs > TCS_TURN_TIMEOUT_MS) {
      say(String("[TCS] Turn took too long (") + String(turnDoneDeg, 0) + " of " + String(turnTargetDeg, 0) + " deg). Stopping to look again.");
      begin_settling();
    }
  } else if (pointingStep == POINT_SETTLING && !still) {
    settleStartedMs = now;                                // must be still for the whole settle time
  }
}

/**
 * Keeps the louvers pointed away from the brightest light. Called twice a second.
 * Returns true when the louvers are aimed (or there is no bright light) and the satellite is still.
 */
static bool keep_louvers_pointed_away() {
  if (turnWentWrongWay) return true;                      // do not keep spinning the wrong way; vent and wait for a fix
  spinControlOn = true;                                   // the gyroscope holds the satellite still between turns
  switch (pointingStep) {
    case POINT_TURNING:
      return false;                                       // track_turn() is handling it
    case POINT_SETTLING:
      // Wait until it has been still for TCS_TURN_SETTLE_MS, but never longer than TCS_TURN_SETTLE_MAX_MS in total:
      // if it keeps drifting (e.g. the wheel is at its limit), look at the light anyway and correct.
      if (millis() - settleStartedMs < TCS_TURN_SETTLE_MS && millis() - settleBeganMs < TCS_TURN_SETTLE_MAX_MS) return false;
      pointingStep = POINT_IDLE;
      if (!isnan(ventErrorDeg)) {
        say(String("[TCS] After turning: louvers ") + String(fabs(ventErrorDeg), 0) + " deg from pointing away from the light.");
      }
      break;                                              // look again below
    case POINT_IDLE:
      break;
  }
  // Satellite is still: look at the light.
  if (isnan(ventErrorDeg)) return true;                   // no bright light: any direction is fine
  if (fabs(ventErrorDeg) < TCS_VENT_ALIGN_TOL_DEG) return true;   // close enough
  start_turn();
  return false;
}

/** Measures reaction wheel speed from how far the encoder moved since the last check. */
static void read_wheel_speed() {
  uint32_t now = millis();
  int64_t count = enc.getCount();
  if (lastEncoderMs != 0 && now > lastEncoderMs) {
    float seconds = (now - lastEncoderMs) / 1000.0f;
    float revolutions = (float)(count - lastEncoderCount) / ((float)CPR * 10.0f);  // same formula as Lab 7
    wheelRpm = (revolutions / seconds) * 60.0f;
  }
  lastEncoderCount = count;
  lastEncoderMs = now;
}

/*---------------------------------------------------------------------------------------------*/
// 4. Safety checks
/*---------------------------------------------------------------------------------------------*/

/** Puts everything in its safest position and stays there until someone clears the fault. */
static void enter_fault(TcsFaultReason reason) {
  faultReason = reason;
  say(String("[TCS] FAULT: ") + tcs_fault_name(reason));
  if (reason == TCS_FAULT_SENSOR_FAILURE) {
    say(String("[TCS] Thermocouple chip says: ") + describe_thermocouple_fault(lastFaultBits));
  }
  enter_state(TCS_FAULT);
}

/**
 * Checks for anything that should stop automatic control. Returns true if a fault was raised.
 * Order matters: a dead sensor is checked first because every other temperature check needs it.
 */
static bool check_for_faults() {
  if (badReadCount >= TCS_SENSOR_FAIL_LIMIT) { enter_fault(TCS_FAULT_SENSOR_FAILURE); return true; }
  if (isnan(panelTempC)) return false;           // one bad reading is tolerated; wait for more evidence

  if (panelTempC > TCS_FAULT_HIGH_C) { enter_fault(TCS_FAULT_TOO_HOT);  return true; }
  if (panelTempC < TCS_FAULT_LOW_C)  { enter_fault(TCS_FAULT_TOO_COLD); return true; }
  if (fabs(tempRateCPerS) > TCS_FAULT_RATE_C_PER_S) { enter_fault(TCS_FAULT_RATE_TOO_FAST); return true; }
  if (fabs(wheelRpm) >= TCS_MAX_WHEEL_RPM) { enter_fault(TCS_FAULT_WHEEL_SATURATED); return true; }
  return false;
}

/*---------------------------------------------------------------------------------------------*/
// 4b. Peltier test heater (separate from the thermal control logic)
//     It heats the panel so we can watch how the louvers and attitude react to a warming satellite.
//     It only ever heats. Its power is a percentage: how much of the time it is switched on.
/*---------------------------------------------------------------------------------------------*/

enum PeltierMode { PELTIER_OFF, PELTIER_HOLD_TARGET, PELTIER_MANUAL_POWER };

static PeltierMode peltierMode = PELTIER_OFF;
static bool     peltierStarted = false;      // Switched on the first time it is used, so the pin stays quiet at power-up
static float    peltierTargetC = NAN;        // Temperature we are holding (only in HOLD_TARGET mode)
static float    peltierPowerPercent = 0.0;   // How hard it is running right now
static float    peltierManualPercent = 0.0;  // Power asked for in MANUAL_POWER mode
static float    peltierIntegral = 0.0;       // The slowly building "closing the last gap" part of the controller
static uint32_t peltierLastRunMs = 0;

/** Sends a power level (0 to 100) to the heater switch. */
static void apply_peltier_power(float percent) {
  peltierPowerPercent = constrain(percent, 0.0, TCS_PELTIER_MAX_POWER_PERCENT);
  if (!peltierStarted) {
    if (peltierPowerPercent <= 0.0) return;          // nothing to do and the pin has never been used
    ledcSetup(TCS_PELTIER_LEDC_CHANNEL, TCS_PELTIER_PWM_FREQ_HZ, TCS_PELTIER_PWM_BITS);
    ledcAttachPin(TCS_PELTIER_PIN, TCS_PELTIER_LEDC_CHANNEL);
    peltierStarted = true;
  }
  const uint32_t fullScale = (1UL << TCS_PELTIER_PWM_BITS) - 1;
  ledcWrite(TCS_PELTIER_LEDC_CHANNEL, (uint32_t)(peltierPowerPercent / 100.0 * fullScale));
}

/** Turns the heater fully off and forgets what it was doing. */
static void peltier_off(const char* reasonOrNull) {
  peltierMode = PELTIER_OFF;
  peltierTargetC = NAN;
  peltierIntegral = 0.0;
  apply_peltier_power(0);
  if (reasonOrNull != nullptr) say(String("[TCS] Peltier heater OFF: ") + reasonOrNull);
}

/**
 * Decides how hard the heater should run. Called every TCS update (twice a second).
 *
 * Safety first, in this order:
 *   1. Thermocouple failed for several readings in a row -> heater off for good.
 *   2. No temperature reading right now -> heater paused (power 0) until readings come back.
 *   3. Panel above the fault limit -> heater off for good.
 * Then, if it is holding a target: power grows the further the panel is below the target, plus a slow
 * build-up (the "integral") so it can close the last bit of the gap. At or above target, power is zero.
 */
static void run_peltier() {
  uint32_t now = millis();
  float dtSeconds = (peltierLastRunMs == 0) ? 0.0 : (now - peltierLastRunMs) / 1000.0f;
  peltierLastRunMs = now;

  if (peltierMode == PELTIER_OFF) return;

  if (badReadCount >= TCS_SENSOR_FAIL_LIMIT) { peltier_off("thermocouple failed"); return; }
  if (isnan(panelTempC)) { apply_peltier_power(0); return; }
  if (panelTempC > TCS_FAULT_HIGH_C) { peltier_off("panel above the safety limit"); return; }

  if (peltierMode == PELTIER_MANUAL_POWER) {
    apply_peltier_power(peltierManualPercent);
    return;
  }

  // Hold-target mode:
  float error = peltierTargetC - panelTempC;           // positive = panel is colder than the target
  if (error <= 0) {
    peltierIntegral = 0.0;                             // at or above target: stop heating and reset the build-up
    apply_peltier_power(0);
    return;
  }
  peltierIntegral += error * dtSeconds;
  float integralLimit = TCS_PELTIER_MAX_POWER_PERCENT / TCS_PELTIER_KI_PERCENT_PER_C_S;   // stops it winding up forever
  peltierIntegral = constrain(peltierIntegral, 0.0, integralLimit);
  apply_peltier_power(TCS_PELTIER_KP_PERCENT_PER_C * error + TCS_PELTIER_KI_PERCENT_PER_C_S * peltierIntegral);
}

/*---------------------------------------------------------------------------------------------*/
// 5. The state machine: what to do in each step
/*---------------------------------------------------------------------------------------------*/

/** True once the panel has cooled far enough below the hold temperature to stop venting. */
static bool cooled_enough() {
  return !isnan(panelTempC) && panelTempC <= holdTempC - TCS_COOL_HYSTERESIS_C;
}

static float    louverIntegral = 0.0;    // The slowly built-up part of the louver opening (percent)
static uint32_t lastRegulateMs = 0;      // When the louver controller last ran (to know how much time passed)
static uint32_t louversShutSinceMs = 0;  // When the controller last asked for fully shut louvers (0 = not shut)

/** Forgets the built-up part of the louver controller (used when control starts fresh). */
static void reset_louver_controller() {
  louverIntegral = 0.0;
  lastRegulateMs = 0;
  louversShutSinceMs = 0;
}

/**
 * The closed-loop louver controller. Called twice a second while venting.
 *
 * It looks at how far the panel is above (or below) the hold temperature and works out an opening:
 *   opening = proportional part + integral part
 *   proportional = TCS_LOUVER_KP_PERCENT_PER_C x (degrees above hold)       -> reacts right away
 *   integral    += TCS_LOUVER_KI_PERCENT_PER_C_S x (degrees above hold) x seconds -> settles the last bit
 * With a fixed heater power, the integral part ends up at exactly the opening that lets out as much
 * heat as the heater puts in, so the panel sits at the hold temperature.
 * Both parts are kept between 0 and 100 %, so the integral cannot "wind up" while the louvers are already
 * fully open or fully shut.
 */
static float regulate_louver_opening() {
  uint32_t now = millis();
  float seconds = (lastRegulateMs == 0) ? 0.0 : (now - lastRegulateMs) / 1000.0f;
  lastRegulateMs = now;
  if (isnan(panelTempC)) return louverPercent;          // no reading: leave the louvers where they are

  float degreesAboveHold = panelTempC - holdTempC;
  louverIntegral = constrain(louverIntegral + TCS_LOUVER_KI_PERCENT_PER_C_S * degreesAboveHold * seconds,
                             TCS_LOUVER_CLOSED_PERCENT, TCS_LOUVER_OPEN_PERCENT);
  float opening = TCS_LOUVER_KP_PERCENT_PER_C * degreesAboveHold + louverIntegral;
  opening = constrain(opening, TCS_LOUVER_CLOSED_PERCENT, TCS_LOUVER_OPEN_PERCENT);

  // Remember how long the controller has wanted the louvers fully shut (heat source gone).
  if (opening <= TCS_LOUVER_CLOSED_PERCENT) { if (louversShutSinceMs == 0) louversShutSinceMs = now; }
  else louversShutSinceMs = 0;
  return opening;
}

static void run_state_machine() {
  switch (state) {

    case TCS_NORMAL:
      // Temperature is fine: louvers rest, wheel stopped. Just watch for trouble.
      set_louvers(TCS_LOUVER_NOMINAL_PERCENT);
      hold_still();   
      if (isnan(panelTempC)) break;                       // no reading this time; stay put
      if (panelTempC > holdTempC) {
        ventAnyway = false;
        turnWentWrongWay = false;                         // a new heat-up gets a fresh try at turning
        reset_louver_controller();                        // start the louver controller fresh
        enter_state(TCS_SUN_AVOID);                       // above the hold temperature: first aim the louvers away from the light
      } else if (panelTempC < holdTempC - TCS_COLD_MARGIN_C) {
        enter_state(TCS_COLD_DETECTED);
      }
      break;

    case TCS_SUN_AVOID:
      // Too hot, and the louvers are well off "directly away from the light". The wheel turns the
      // satellite to fix that, and AT THE SAME TIME the louver controller keeps venting (the louvers are
      // never forced shut here), so the panel keeps cooling while it turns.
      set_louvers(regulate_louver_opening());
      read_sun_sensor();

      if (cooled_enough()) {
        hold_still();   
        enter_state(TCS_RECOVERY);                        // cooled off by itself; nothing to vent
      } else if (time_in_state_ms() > TCS_SUN_AVOID_TIMEOUT_MS) {
        say("[TCS] Could not turn away from the light in time, venting without turning for a while.");
        hold_still();
        ventAnyway = true;                                // COOLING pauses turning, then tries again after TCS_TURN_RETRY_MS
        ventAnywaySinceMs = millis();
        enter_state(TCS_COOLING);
      } else if (keep_louvers_pointed_away()) {
        enter_state(TCS_COOLING);                         // aimed at cold space (or no bright light at all)
      }
      break;

    case TCS_COOLING:
      // Closed loop: keep adjusting the louver opening to hold the panel AT the hold temperature,
      // AND keep the louvers pointed away from the light the whole time. Small pointing errors are
      // corrected here with the wheel without touching the louvers; only a big drift is reported as
      // SUN_AVOID (where the louvers still keep venting).
      set_louvers(regulate_louver_opening());
      if (louversShutSinceMs != 0 && millis() - louversShutSinceMs > TCS_REGULATE_IDLE_MS &&
          !isnan(panelTempC) && panelTempC < holdTempC) {
        hold_still();   
        say("[TCS] No heat to vent for a while, louvers stay shut.");
        enter_state(TCS_RECOVERY);                        // heat source gone: stop regulating
      } else if (TCS_LOUVER_MAX_OPEN_MS > 0 && time_in_state_ms() > TCS_LOUVER_MAX_OPEN_MS) {
        hold_still();   
        say("[TCS] Louvers open too long, closing them to avoid over-cooling.");
        enter_state(TCS_RECOVERY);
      } else if (ventAnyway && millis() - ventAnywaySinceMs < TCS_TURN_RETRY_MS) {
        hold_still();                                      // turning did not work just now: vent, and try again later
      } else {
        if (ventAnyway) {
          ventAnyway = false;
          say("[TCS] Trying again to point the louvers away from the light.");
        }
        read_sun_sensor();
        // Only judge "drifted far" while the satellite is still (readings taken mid-turn are not trusted).
        bool driftedFar = pointingStep == POINT_IDLE && !isnan(ventErrorDeg) && fabs(ventErrorDeg) > TCS_VENT_LOST_TOL_DEG;
        keep_louvers_pointed_away();                      // look / turn / settle while venting
        if (driftedFar) {
          say("[TCS] Louvers drifted toward the light, turning back (still venting).");
          enter_state(TCS_SUN_AVOID);
        }
      }
      break;

    case TCS_COLD_DETECTED:
      // Too cold: keep the louvers shut so no heat escapes, and wait for the satellite to warm up.
      set_louvers(TCS_LOUVER_CLOSED_PERCENT);
      hold_still();   
      if (!isnan(panelTempC) && panelTempC >= holdTempC - TCS_COLD_MARGIN_C + TCS_COOL_HYSTERESIS_C) enter_state(TCS_RECOVERY);
      break;

    case TCS_RECOVERY:
      // Heating or cooling is finished: louvers to rest, wheel stopped, wait a moment, then back to NORMAL.
      set_louvers(TCS_LOUVER_NOMINAL_PERCENT);
      hold_still();   
      if (time_in_state_ms() >= TCS_RECOVERY_HOLD_MS) enter_state(TCS_NORMAL);
      break;

    case TCS_FAULT:
      // Safest position. Too hot -> louvers open so heat can escape. Anything else -> louvers closed.
      set_louvers(faultReason == TCS_FAULT_TOO_HOT ? TCS_LOUVER_OPEN_PERCENT : TCS_LOUVER_CLOSED_PERCENT);
      stop_turning();
      break;
  }
}

/*---------------------------------------------------------------------------------------------*/
// 6. Public functions (see tcs.h for what each one is for)
/*---------------------------------------------------------------------------------------------*/

bool tcs_init(uint8_t thermocouple1ChipSelectPin, uint8_t thermocouple2ChipSelectPin) {
  // Keep both chips "deselected" right away so they do not interfere with the SD card on the shared wires.
  pinMode(thermocouple1ChipSelectPin, OUTPUT);
  digitalWrite(thermocouple1ChipSelectPin, HIGH);
  pinMode(thermocouple2ChipSelectPin, OUTPUT);
  digitalWrite(thermocouple2ChipSelectPin, HIGH);

  thermocouple = new Adafruit_MAX31855(thermocouple1ChipSelectPin);
  thermocouple2 = new Adafruit_MAX31855(thermocouple2ChipSelectPin);

  bool ok = true;
  if (!thermocouple->begin()) {
    Serial.println("[WARN] MAX31855 #1 (panel) not responding. TCS will report a sensor failure.");
    ok = false;
  } else {
    Serial.println("[INFO] MAX31855 #1 (panel) online.");
  }
  if (!thermocouple2->begin()) {
    Serial.println("[WARN] MAX31855 #2 not responding. Its temperature will show as nan.");
    ok = false;
  } else {
    Serial.println("[INFO] MAX31855 #2 online.");
  }

  // Load the light-sensor calibration saved by command 20 (if there is one).
  Preferences saved;
  saved.begin("tcs", true);                                  // true = read only
  lightOffsetDeg = saved.getFloat("light_off", TCS_LIGHT_DIRECTION_OFFSET_DEG);
  saved.end();
  Serial.println(String("[INFO] Light direction calibration: ") + String(lightOffsetDeg, 1) + " deg");
  return ok;
}

bool tcs_calibrate_light_direction(float actualDirectionDeg) {
  Preferences saved;
  if (actualDirectionDeg < 0) {                              // negative = forget the calibration
    lightOffsetDeg = TCS_LIGHT_DIRECTION_OFFSET_DEG;
    saved.begin("tcs", false);
    saved.remove("light_off");
    saved.end();
    say(String("[TCS] Light calibration cleared (offset ") + String(lightOffsetDeg, 1) + " deg).");
    return true;
  }
  read_sun_sensor();                                         // fresh reading
  if (isnan(rawLightDirectionDeg)) {
    say("[TCS] Too dark to calibrate: shine the lamp on the satellite and try again.");
    return false;
  }
  lightOffsetDeg = wrap_to_180(actualDirectionDeg - rawLightDirectionDeg);
  saved.begin("tcs", false);
  saved.putFloat("light_off", lightOffsetDeg);
  saved.end();
  read_sun_sensor();                                         // recompute with the new offset
  say(String("[TCS] Light calibrated: sensors read ") + String(rawLightDirectionDeg, 0) + " deg, lamp is at " +
      String(actualDirectionDeg, 0) + " deg, so offset = " + String(lightOffsetDeg, 1) + " deg (saved).");
  return true;
}

/** Turns a number into text for the stream. Invalid readings become "nan" so a plot can skip them. */
static String number_for_stream(float value, int decimals) {
  return isnan(value) ? String("nan") : String(value, decimals);
}

/**
 * Sends one line of live data for the plot (see tcs.h for the exact format).
 * Runs on its own timer, separate from the TCS decisions, so the plot gets a steady one point per second.
 */
static void send_telemetry_if_due() {
  if (!telemetryOn) return;
  uint32_t now = millis();
  if (now - lastTelemetryMs < TCS_TELEMETRY_INTERVAL_MS) return;
  lastTelemetryMs = now;

  String line = "TCS,t:" + String(now);
  line += ",temp_C:" + number_for_stream(panelTempC, 2);
  line += ",temp2_C:" + number_for_stream(temp2C, 2);
  line += ",chip_C:" + number_for_stream(chipTempC, 2);
  line += ",rate_C_per_s:" + number_for_stream(tempRateCPerS, 2);
  line += ",state_id:" + String((int)state);
  line += ",tcs_on:" + String(tcs_enabled ? 1 : 0);
  line += ",hold_C:" + number_for_stream(holdTempC, 1);
  line += ",louver_pct:" + number_for_stream(louverPercent, 0);
  line += ",wheel_rpm:" + number_for_stream(wheelRpm, 1);
  line += ",vent_error_deg:" + number_for_stream(ventErrorDeg, 1);
  line += ",light_dir_deg:" + number_for_stream(lightDirectionDeg, 1);
  line += ",light_total:" + number_for_stream(lightTotal, 0);
  line += ",light_px:" + number_for_stream(lightPlusX, 0);
  line += ",light_mx:" + number_for_stream(lightMinusX, 0);
  line += ",light_py:" + number_for_stream(lightPlusY, 0);
  line += ",light_my:" + number_for_stream(lightMinusY, 0);
  line += ",peltier_pct:" + number_for_stream(peltierPowerPercent, 1);
  line += ",peltier_target_C:" + number_for_stream(peltierTargetC, 1);
  say(line);
}

bool tcs_set_peltier_target(float targetC) {
  if (targetC == 0) { peltier_off("target set to 0"); return true; }
  if (targetC < TCS_PELTIER_MIN_TARGET_C || targetC > TCS_PELTIER_MAX_TARGET_C) {
    say(String("[TCS] Peltier target must be 0 (off) or between ") + String(TCS_PELTIER_MIN_TARGET_C, 0) +
        " and " + String(TCS_PELTIER_MAX_TARGET_C, 0) + " C. Heater left as it was.");
    return false;
  }
  if (isnan(panelTempC)) {
    say("[TCS] No valid thermocouple reading, so the heater was NOT started.");
    return false;
  }
  peltierMode = PELTIER_HOLD_TARGET;
  peltierTargetC = targetC;
  peltierIntegral = 0.0;
  say(String("[TCS] Peltier heater holding the panel at ") + String(targetC, 1) + " C (heats only).");
  return true;
}

void tcs_set_peltier_power_manual(float percent) {
  if (isnan(panelTempC)) {
    say("[TCS] No valid thermocouple reading, so the heater was NOT started.");
    return;
  }
  peltierManualPercent = constrain(percent, 0.0, TCS_PELTIER_MAX_POWER_PERCENT);
  if (peltierManualPercent <= 0) { peltier_off("power set to 0"); return; }
  peltierMode = PELTIER_MANUAL_POWER;
  peltierTargetC = NAN;
  say(String("[TCS] Peltier heater at a fixed ") + String(peltierManualPercent, 0) + "% power.");
}

void  tcs_stop_peltier()               { if (peltierMode != PELTIER_OFF || peltierPowerPercent > 0) peltier_off("stopped"); }
float tcs_get_peltier_power_percent()  { return peltierPowerPercent; }
float tcs_get_peltier_target_c()       { return peltierTargetC; }

void tcs_set_louvers_manual(float openPercent) {
  if (tcs_enabled) {
    say("[TCS] Turn automatic thermal control off first (command 10), or it will move the louvers back.");
    return;
  }
  set_louvers(openPercent, true);              // by hand: move exactly where asked, even by 1%
  say(String("[TCS] Louvers set to ") + String(louverPercent, 0) + "% open.");
}

/*---------------------------------------------------------------------------------------------*/
// SD card CSV log
// Writes one row every TCS_LOG_INTERVAL_MS (4 per second) to its own file on the SD card, so the data can be opened in Excel,
// MATLAB or Python and plotted afterwards. Empty cells mean "no valid reading".
/*---------------------------------------------------------------------------------------------*/

static FsFile   logFile;               // The open log file
static bool     loggingOn = false;     // Are we writing rows right now?
static uint32_t logStartMs = 0;        // When logging started (so the file can show seconds since start)
static uint32_t lastLogMs = 0;         // When the last row was written
static uint32_t logRowCount = 0;       // How many rows have been written to this file
static char     logFileName[40] = "";  // The file's name, to tell the user

/** Turns a number into a CSV cell. Invalid readings become an empty cell, which plotting tools skip. */
static String number_for_csv(float value, int decimals) {
  return isnan(value) ? String("") : String(value, decimals);
}

/** Writes one row to the log file, if logging is on and it is time. */
static void log_row_if_due() {
  if (!loggingOn) return;
  uint32_t now = millis();
  if (now - lastLogMs < TCS_LOG_INTERVAL_MS) return;
  lastLogMs = now;

  String row = String(now);
  row += "," + String((now - logStartMs) / 1000.0f, 1);   // seconds since logging started: handy for the x-axis
  row += "," + number_for_csv(panelTempC, 2);
  row += "," + number_for_csv(temp2C, 2);
  row += "," + number_for_csv(tempRateCPerS, 2);
  row += "," + String(tcs_state_name(state));
  row += "," + String((int)state);
  row += "," + String(tcs_enabled ? 1 : 0);
  row += "," + number_for_csv(holdTempC, 1);
  row += "," + number_for_csv(louverPercent, 0);
  row += "," + number_for_csv(wheelRpm, 1);
  row += "," + number_for_csv(ventErrorDeg, 1);
  row += "," + number_for_csv(lightDirectionDeg, 1);
  row += "," + number_for_csv(lightTotal, 0);
  row += "," + number_for_csv(peltierPowerPercent, 1);
  row += "," + number_for_csv(peltierTargetC, 1);
  // Sun sensing: the four raw sensors, the direction before calibration, and the Sun's direction in the room.
  // light_dir_deg is where the light is relative to the satellite; adding the heading gives where it is in the
  // room, which should stay steady while the satellite turns (the proof the satellite knows where the Sun is).
  row += "," + number_for_csv(lightPlusX, 0) + "," + number_for_csv(lightMinusX, 0);
  row += "," + number_for_csv(lightPlusY, 0) + "," + number_for_csv(lightMinusY, 0);
  row += "," + number_for_csv(rawLightDirectionDeg, 1);
  float sunInRoomDeg = NAN;
  if (!isnan(lightDirectionDeg)) {
    sunInRoomDeg = fmod(lightDirectionDeg + headingDeg, 360.0f);
    if (sunInRoomDeg < 0) sunInRoomDeg += 360.0f;
  }
  row += "," + number_for_csv(sunInRoomDeg, 1);
  // Attitude control: heading and spin from the gyroscope, what the wheel is told, and the turn in progress.
  row += "," + number_for_csv(headingDeg, 1);
  row += "," + number_for_csv(spinRateDegPerS, 2);
  row += "," + number_for_csv(wheelCommand * TCS_WHEEL_DIRECTION, 3);
  row += "," + String((int)pointingStep);
  row += "," + (pointingStep == POINT_TURNING ? number_for_csv(turnTargetDeg, 1) : String(""));
  row += "," + (pointingStep == POINT_TURNING ? number_for_csv(turnDoneDeg, 1) : String(""));

  if (logFile.println(row) == 0) {      // the card did not accept the row (removed, full, or failing)
    loggingOn = false;
    logFile.close();
    say("[TCS] ERROR: could not write to the SD card, logging stopped.");
    return;
  }
  logFile.flush();                      // save every row, so pulling the power loses at most one row
  logRowCount++;
}

bool tcs_start_logging() {
  if (loggingOn) {
    say(String("[TCS] Already logging to ") + logFileName);
    return true;
  }
  if (!sd_createDataFile(&logFile, TCS_LOG_FILE_PREFIX)) {
    say("[TCS] ERROR: could not create a log file on the SD card.");
    return false;
  }
  logFile.println("time_ms,elapsed_s,temp1_C,temp2_C,rate_C_per_s,state,state_id,tcs_on,hold_C,"
                  "louver_pct,wheel_rpm,vent_error_deg,light_dir_deg,light_total,peltier_pct,peltier_target_C,"
                  "light_px,light_mx,light_py,light_my,light_raw_dir_deg,sun_in_room_deg,"
                  "heading_deg,spin_deg_per_s,wheel_cmd,point_step,turn_target_deg,turn_done_deg");
  logFile.flush();
  logFile.getName(logFileName, sizeof(logFileName));
  headingDeg = 0;                       // heading is measured from where the satellite faces when logging starts

  loggingOn = true;
  logStartMs = millis();
  lastLogMs = 0;                        // write the first row straight away
  logRowCount = 0;
  say(String("[TCS] Logging started: ") + logFileName);
  return true;
}

void tcs_stop_logging() {
  if (!loggingOn) return;
  loggingOn = false;
  logFile.close();
  say(String("[TCS] Logging stopped: ") + logFileName + " (" + String(logRowCount) + " rows)");
}

bool tcs_is_logging() { return loggingOn; }

void tcs_set_telemetry(bool on) { telemetryOn = on; }
bool tcs_telemetry_enabled()    { return telemetryOn; }

void tcs_update() {
  send_telemetry_if_due();                                        // live plot data, once per second
  log_row_if_due();                                               // SD card CSV log, 4 rows per second (if started)
  float gyroSeconds;
  if (read_gyro_if_due(&gyroSeconds) && tcs_enabled) track_turn(gyroSeconds);   // heading always; wheel steering when on
  if (millis() - lastUpdateMs < TCS_UPDATE_INTERVAL_MS) return;   // not time yet
  lastUpdateMs = millis();

  read_thermocouple();                    // always keep the temperature fresh, even when the TCS is off
  read_thermocouple2();                   // second thermocouple: reported only
  run_peltier();                        // test heater: runs whether or not the TCS is on
  read_sun_sensor();                    // light direction: always measured, so it can be watched and calibrated with the TCS off
  if (!tcs_enabled) return;

  read_wheel_speed();
  if (state != TCS_FAULT) check_for_faults();   // if something is wrong this switches us to FAULT
  run_state_machine();
}

bool tcs_set_hold_temperature(float holdC) {
  if (holdC < TCS_MIN_HOLD_TEMP_C || holdC > TCS_MAX_HOLD_TEMP_C) {
    say(String("[TCS] Hold temperature must be between ") + String(TCS_MIN_HOLD_TEMP_C, 0) + " and " +
        String(TCS_MAX_HOLD_TEMP_C, 0) + " C. Nothing changed.");
    return false;
  }
  holdTempC = holdC;
  say(String("[TCS] Hold temperature set to ") + String(holdC, 1) + " C. Above it the satellite turns its louvers away "
      "from the light and opens them.");
  tcs_enable();                           // pressing Hold means "control to this temperature", so switch control on
  return true;
}

float tcs_get_hold_temperature() { return holdTempC; }

void tcs_enable() {
  if (tcs_enabled) return;
  tcs_enabled = true;
  reset_louver_controller();
  faultReason = TCS_FAULT_NONE;
  badReadCount = 0;
  ventAnyway = false;
  turnWentWrongWay = false;
  state = TCS_NORMAL;
  stateStartMs = millis();
  rateReferenceTempC = NAN;               // restart the rate measurement from scratch
  tempRateCPerS = 0.0;
  say("[TCS] Automatic thermal control ON.");
}

void tcs_disable() {
  if (!tcs_enabled) return;
  tcs_enabled = false;
  set_louvers(TCS_LOUVER_NOMINAL_PERCENT);
  stop_turning();
  state = TCS_NORMAL;
  say("[TCS] Automatic thermal control OFF.");
}

void tcs_clear_fault() {
  if (!tcs_enabled || state != TCS_FAULT) return;
  faultReason = TCS_FAULT_NONE;
  badReadCount = 0;
  rateReferenceTempC = NAN;
  tempRateCPerS = 0.0;
  say("[TCS] Fault cleared. If the cause is still there it will trip again.");
  enter_state(TCS_NORMAL);
}

/**
 * Reads the 32-bit data word from the MAX31855 by wiggling the clock pin by hand ("bit-banging").
 * This avoids the SPI hardware and the library, so it tests the wires and the chip on their own.
 * @param chipSelectPin which chip to talk to (thermocouple 1 or thermocouple 2)
 * @param dataPinMode INPUT_PULLUP or INPUT_PULLDOWN: what the ESP32 does to the data line when
 *        nothing is driving it. A connected, working chip ignores this; a loose wire does not.
 */
static uint32_t bit_bang_read_word(uint8_t chipSelectPin, int dataPinMode) {
  pinMode(SPI_SCK, OUTPUT);
  digitalWrite(SPI_SCK, LOW);
  pinMode(SPI_MISO, dataPinMode);
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);
  delay(5);

  digitalWrite(chipSelectPin, LOW);                   // start talking to the chip
  delayMicroseconds(10);
  uint32_t word = 0;
  for (int bit = 0; bit < 32; bit++) {
    word = (word << 1) | (digitalRead(SPI_MISO) ? 1 : 0);  // read the data line while the clock is low
    digitalWrite(SPI_SCK, HIGH);                      // one clock pulse moves the chip to the next bit
    delayMicroseconds(10);
    digitalWrite(SPI_SCK, LOW);
    delayMicroseconds(10);
  }
  digitalWrite(chipSelectPin, HIGH);                  // stop talking to the chip
  return word;
}

/** Tests one chip's wiring and prints what it found. */
static void wiring_test_one_chip(const char* chipName, uint8_t chipSelectPin) {
  say(String("[TCS] --- ") + chipName + " (chip-select GPIO " + String(chipSelectPin) + ") ---");

  SPI.end();                                          // free the pins so we can wiggle them by hand
  digitalWrite(SD_CS_PIN, HIGH);                      // keep the SD card quiet during the test
  uint32_t withPullUp   = bit_bang_read_word(chipSelectPin, INPUT_PULLUP);
  uint32_t withPullDown = bit_bang_read_word(chipSelectPin, INPUT_PULLDOWN);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);             // give the pins back to the SPI hardware

  say(String("[TCS] Data line pulled UP:   0x") + String(withPullUp, HEX));
  say(String("[TCS] Data line pulled DOWN: 0x") + String(withPullDown, HEX));

  // Work out what that means.
  if (withPullUp == 0xFFFFFFFF && withPullDown == 0x00000000) {
    say(String("[TCS] RESULT: the data line (DO) is FLOATING. Nothing is driving it. Check: DO wire reaches GPIO 19, "
        "this chip's CS wire reaches GPIO ") + String(chipSelectPin) + ", the chip has 3.3V and GND, and the breadboard rows are actually connected.");
  } else if (withPullUp == 0xFFFFFFFF && withPullDown == 0xFFFFFFFF) {
    say("[TCS] RESULT: the data line is stuck HIGH. DO may be touching 3.3V, or the wrong pin is wired to 3.3V.");
  } else if (withPullUp == 0x00000000 && withPullDown == 0x00000000) {
    say("[TCS] RESULT: the data line is stuck LOW. DO may be touching GND, or the chip has no power.");
  } else if (withPullUp == withPullDown) {
    say("[TCS] RESULT: the chip IS answering. The wires are fine. If readings still fail, the problem is in the SPI setup, not the wiring.");
  } else {
    say("[TCS] RESULT: the data line is only partly driven (noisy or a poor connection). Re-seat the DO and CS wires.");
  }
}

void tcs_run_wiring_test() {
  if (tcs_enabled) {
    say("[TCS] Turn the TCS off first (command 10), then run the wiring test.");
    return;
  }
  say("[TCS] Thermocouple wiring test (checks both chips, takes a few seconds)...");
  wiring_test_one_chip("Thermocouple chip #1 (panel)", MAX31855_CS_PIN);
  wiring_test_one_chip("Thermocouple chip #2", MAX31855_2_CS_PIN);
}

bool           tcs_is_enabled()             { return tcs_enabled; }
TcsState       tcs_get_state()              { return state; }
TcsFaultReason tcs_get_fault_reason()       { return faultReason; }
float          tcs_get_panel_temp_c()       { return panelTempC; }
float          tcs_get_chip_temp_c()        { return chipTempC; }
float          tcs_get_temp2_c()            { return temp2C; }
float          tcs_get_temp_rate_c_per_s()  { return tempRateCPerS; }
float          tcs_get_louver_percent()     { return louverPercent; }
float          tcs_get_vent_error_deg()     { return ventErrorDeg; }
float          tcs_get_wheel_rpm()          { return wheelRpm; }

const char* tcs_state_name(TcsState s) {
  switch (s) {
    case TCS_NORMAL:        return "NORMAL";
    case TCS_COOLING:       return "COOLING";
    case TCS_COLD_DETECTED: return "COLD_DETECTED";
    case TCS_SUN_AVOID:     return "SUN_AVOID";
    case TCS_RECOVERY:      return "RECOVERY";
    case TCS_FAULT:         return "FAULT";
  }
  return "UNKNOWN";
}

const char* tcs_fault_name(TcsFaultReason r) {
  switch (r) {
    case TCS_FAULT_NONE:            return "none";
    case TCS_FAULT_SENSOR_FAILURE:  return "thermocouple sensor failure";
    case TCS_FAULT_TOO_HOT:         return "panel too hot";
    case TCS_FAULT_TOO_COLD:        return "panel too cold";
    case TCS_FAULT_RATE_TOO_FAST:   return "temperature changing too fast";
    case TCS_FAULT_WHEEL_SATURATED: return "reaction wheel at speed limit";
  }
  return "unknown";
}

void tcs_print_status() {
  String line = String("[TCS] ") + (tcs_enabled ? "ON" : "OFF") + ", state=" + tcs_state_name(state);
  line += ", hold=" + String(holdTempC, 1) + "C";
  line += ", panel=" + (isnan(panelTempC) ? String("n/a") : String(panelTempC, 2) + "C");
  line += ", temp2=" + (isnan(temp2C) ? String("n/a") : String(temp2C, 2) + "C");
  line += ", rate=" + String(tempRateCPerS, 2) + "C/s";
  line += ", louvers=" + String(louverPercent, 0) + "%";
  line += ", wheel=" + String(wheelRpm, 0) + "RPM";
  line += ", peltier=" + String(peltierPowerPercent, 0) + "%" + (isnan(peltierTargetC) ? String("") : " (target " + String(peltierTargetC, 1) + "C)");
  line += ", vent_error=" + (isnan(ventErrorDeg) ? String("n/a") : String(ventErrorDeg, 1) + "deg");
  line += ", light_from=" + (isnan(lightDirectionDeg) ? String("too dark") : String(lightDirectionDeg, 0) + "deg");
  line += " (calibration " + String(lightOffsetDeg, 1) + "deg)";
  if (state == TCS_FAULT) line += String(", fault=") + tcs_fault_name(faultReason);
  line += String(", logging=") + (loggingOn ? logFileName : "off");
  say(line);

  // Pointing details: what the wheel is doing and why (helps when the satellite does not turn).
  const char* stepName = pointingStep == POINT_TURNING ? "TURNING" : (pointingStep == POINT_SETTLING ? "SETTLING" : "IDLE");
  String point = String("[TCS] Pointing: step=") + stepName + ", spin_control=" + (spinControlOn ? "on" : "off");
  point += ", wheel_cmd=" + String(wheelCommand, 2) + ", spin=" + String(spinRateDegPerS, 1) + "deg/s";
  if (pointingStep == POINT_TURNING) point += ", turned=" + String(turnDoneDeg, 0) + " of " + String(turnTargetDeg, 0) + "deg";
  point += ", light_total=" + String(lightTotal, 0) + " (need " + String(TCS_SUN_MIN_LIGHT_COUNTS) + ")";
  say(point);
  if (!tcs_enabled) say("[TCS] Why no turning: automatic control is OFF (press Hold or send 10).");
  else if (state == TCS_FAULT) say("[TCS] Why no turning: FAULT stops the wheel (send 12 to clear).");
  else if (state == TCS_NORMAL || state == TCS_RECOVERY || state == TCS_COLD_DETECTED)
    say("[TCS] Why no turning: panel is not above the hold temperature, so there is nothing to vent.");
  else if (turnWentWrongWay) say("[TCS] Why no turning: the last turn went the WRONG way, so turning is paused. "
                                 "Flip TCS_WHEEL_DIRECTION or TCS_GYRO_DIRECTION in definitions.h, re-upload, then press Hold.");
  else if (ventAnyway) {
    long secondsLeft = ((long)TCS_TURN_RETRY_MS - (long)(millis() - ventAnywaySinceMs)) / 1000;
    say("[TCS] Why no turning: an earlier turn took too long; it tries again " + String(secondsLeft < 0 ? 0 : secondsLeft) + " s from now.");
  }
  else if (isnan(ventErrorDeg)) say("[TCS] Why no turning: too dark, no bright light to turn away from.");
  else if (pointingStep == POINT_IDLE && fabs(ventErrorDeg) < TCS_VENT_ALIGN_TOL_DEG) say("[TCS] Louvers already pointed away from the light.");

  if (isnan(panelTempC)) {   // explain why there is no temperature
    say(String("[TCS] No temperature: ") + describe_thermocouple_fault(lastFaultBits));
  }
}

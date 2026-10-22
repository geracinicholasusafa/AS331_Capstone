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
#include "definitions.h"
#include "tcs.h"

/*---------------------------------------------------------------------------------------------*/
// 1. Things that live in main.cpp and that the TCS borrows
/*---------------------------------------------------------------------------------------------*/
extern HardwareSerial Xbee;               // Radio link to the ground station
extern TB9051FTGMotorCarrier driver;      // Reaction wheel motor driver
extern ESP32Encoder enc;                  // Reaction wheel encoder

/*---------------------------------------------------------------------------------------------*/
// 1. What the TCS remembers between calls (all private to this file)
/*---------------------------------------------------------------------------------------------*/
static Adafruit_MAX31855* thermocouple = nullptr;   // The thermocouple chip

static bool           tcs_enabled = false;          // Is automatic control switched on?
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

static float ventErrorDeg = NAN;      // How far the louvers are from pointing directly away from the brightest light
static bool  ventAnyway = false;      // True once turning away took too long: vent regardless, heat must escape

/*---------------------------------------------------------------------------------------------*/
// 2. Small helpers
/*---------------------------------------------------------------------------------------------*/

/** Prints a message to both the USB cable and the radio. */
static void say(const String& message) {
  Serial.println(message);
  Xbee.println(message);
}

/**
 * Moves the louvers. 0 = closed, 100 = fully open.
 * NOTE: the louver servos have not been chosen yet. For now this only remembers the command so the
 * rest of the TCS can be tested. When the servo hardware is known, the real servo code goes here
 * and nothing else in this file has to change.
 */
static void set_louvers(float openPercent) {
  louverPercent = constrain(openPercent, TCS_LOUVER_CLOSED_PERCENT, TCS_LOUVER_OPEN_PERCENT);
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

  float totalLight = plusX + minusX + plusY + minusY;
  if (totalLight < TCS_SUN_MIN_LIGHT_COUNTS) {
    ventErrorDeg = NAN;                                  // too dark: no bright light to avoid
    return;
  }

  float sunX = (plusX - minusX) / totalLight;
  float sunY = (plusY - minusY) / totalLight;
  float sunDirectionDeg = atan2(sunY, sunX) * RAD_TO_DEG;
  if (sunDirectionDeg < 0) sunDirectionDeg += 360.0;     // 0 to 360 like the lab code

  // The ideal louver direction is exactly opposite the light (180 degrees away from it).
  float idealLouverDirectionDeg = sunDirectionDeg + 180.0;
  ventErrorDeg = wrap_to_180(TCS_LOUVER_FACE_ANGLE_DEG - idealLouverDirectionDeg);
}

/**
 * Pushes the reaction wheel to turn the satellite so ventErrorDeg shrinks toward zero.
 * The further off we are, the harder we push (up to a gentle maximum).
 */
static void turn_toward_vent_heading() {
  float push = constrain(fabs(ventErrorDeg) * TCS_TURN_GAIN_PER_DEG, 0.0, TCS_TURN_MAX_THROTTLE);
  // A positive error means the louvers are "ahead" of the ideal heading, so the satellite must turn
  // backwards. The wheel always spins the opposite way to the satellite, so it spins forwards.
  float direction = (ventErrorDeg > 0) ? 1.0 : -1.0;
  set_wheel(direction * push * TCS_WHEEL_DIRECTION);
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
// 5. The state machine: what to do in each step
/*---------------------------------------------------------------------------------------------*/

static void run_state_machine() {
  switch (state) {

    case TCS_NORMAL:
      // Temperature is fine: louvers rest, wheel stopped. Just watch for trouble.
      set_louvers(TCS_LOUVER_NOMINAL_PERCENT);
      set_wheel(0);
      if (isnan(panelTempC)) break;                       // no reading this time; stay put
      if (panelTempC > TCS_BAND_HIGH_C) {
        ventAnyway = false;
        enter_state(TCS_SUN_AVOID);                       // too hot: first aim the louvers away from the light
      } else if (panelTempC < TCS_BAND_LOW_C) {
        enter_state(TCS_COLD_DETECTED);
      }
      break;

    case TCS_SUN_AVOID:
      // Too hot. Keep the louvers closed (so the Sun does not shine in) while the wheel turns the
      // satellite until the louvers face away from the brightest light.
      set_louvers(TCS_LOUVER_CLOSED_PERCENT);
      read_sun_sensor();

      if (!isnan(panelTempC) && panelTempC <= TCS_TARGET_TEMP_C) {
        set_wheel(0);
        enter_state(TCS_RECOVERY);                        // cooled off by itself; nothing to vent
      } else if (isnan(ventErrorDeg)) {
        set_wheel(0);                                     // no bright light: any direction is fine, vent now
        enter_state(TCS_COOLING);
      } else if (fabs(ventErrorDeg) < TCS_VENT_ALIGN_TOL_DEG) {
        set_wheel(0);                                     // aimed at cold space
        enter_state(TCS_COOLING);
      } else if (time_in_state_ms() > TCS_SUN_AVOID_TIMEOUT_MS) {
        say("[TCS] Could not turn away from the light in time, venting anyway.");
        set_wheel(0);
        ventAnyway = true;                                // stop COOLING from sending us back to turning
        enter_state(TCS_COOLING);
      } else {
        turn_toward_vent_heading();
      }
      break;

    case TCS_COOLING:
      // Open the louvers so internal heat radiates away.
      set_louvers(TCS_LOUVER_OPEN_PERCENT);
      set_wheel(0);
      if (!isnan(panelTempC) && panelTempC <= TCS_TARGET_TEMP_C) {
        enter_state(TCS_RECOVERY);                        // cooled down enough
      } else if (time_in_state_ms() > TCS_LOUVER_MAX_OPEN_MS) {
        say("[TCS] Louvers open too long, closing them to avoid over-cooling.");
        enter_state(TCS_RECOVERY);
      } else if (!ventAnyway) {
        // If the satellite has drifted so the louvers face the light again, close them and turn back.
        read_sun_sensor();
        if (!isnan(ventErrorDeg) && fabs(ventErrorDeg) > TCS_VENT_LOST_TOL_DEG) {
          say("[TCS] Louvers drifted toward the light, turning back.");
          enter_state(TCS_SUN_AVOID);
        }
      }
      break;

    case TCS_COLD_DETECTED:
      // Too cold: keep the louvers shut so no heat escapes, and wait for the satellite to warm up.
      set_louvers(TCS_LOUVER_CLOSED_PERCENT);
      set_wheel(0);
      if (!isnan(panelTempC) && panelTempC >= TCS_TARGET_TEMP_C) enter_state(TCS_RECOVERY);
      break;

    case TCS_RECOVERY:
      // Heating or cooling is finished: louvers to rest, wheel stopped, wait a moment, then back to NORMAL.
      set_louvers(TCS_LOUVER_NOMINAL_PERCENT);
      set_wheel(0);
      if (time_in_state_ms() >= TCS_RECOVERY_HOLD_MS) enter_state(TCS_NORMAL);
      break;

    case TCS_FAULT:
      // Safest position. Too hot -> louvers open so heat can escape. Anything else -> louvers closed.
      set_louvers(faultReason == TCS_FAULT_TOO_HOT ? TCS_LOUVER_OPEN_PERCENT : TCS_LOUVER_CLOSED_PERCENT);
      set_wheel(0);
      break;
  }
}

/*---------------------------------------------------------------------------------------------*/
// 6. Public functions (see tcs.h for what each one is for)
/*---------------------------------------------------------------------------------------------*/

bool tcs_init(uint8_t thermocoupleChipSelectPin) {
  // Keep the chip "deselected" right away so it does not interfere with the SD card on the shared wires.
  pinMode(thermocoupleChipSelectPin, OUTPUT);
  digitalWrite(thermocoupleChipSelectPin, HIGH);

  thermocouple = new Adafruit_MAX31855(thermocoupleChipSelectPin);
  if (!thermocouple->begin()) {
    Serial.println("[WARN] MAX31855 thermocouple chip not responding. TCS will report a sensor failure.");
    return false;
  }
  Serial.println("[INFO] MAX31855 thermocouple online.");
  return true;
}

void tcs_update() {
  if (millis() - lastUpdateMs < TCS_UPDATE_INTERVAL_MS) return;   // not time yet
  lastUpdateMs = millis();

  read_thermocouple();                    // always keep the temperature fresh, even when the TCS is off
  if (!tcs_enabled) return;

  read_wheel_speed();
  if (state != TCS_FAULT) check_for_faults();   // if something is wrong this switches us to FAULT
  run_state_machine();
}

void tcs_enable() {
  if (tcs_enabled) return;
  tcs_enabled = true;
  faultReason = TCS_FAULT_NONE;
  badReadCount = 0;
  ventAnyway = false;
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
  set_wheel(0);
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
 * @param dataPinMode INPUT_PULLUP or INPUT_PULLDOWN: what the ESP32 does to the data line when
 *        nothing is driving it. A connected, working chip ignores this; a loose wire does not.
 */
static uint32_t bit_bang_read_word(int dataPinMode) {
  pinMode(SPI_SCK, OUTPUT);
  digitalWrite(SPI_SCK, LOW);
  pinMode(SPI_MISO, dataPinMode);
  pinMode(MAX31855_CS_PIN, OUTPUT);
  digitalWrite(MAX31855_CS_PIN, HIGH);
  delay(5);

  digitalWrite(MAX31855_CS_PIN, LOW);                 // start talking to the chip
  delayMicroseconds(10);
  uint32_t word = 0;
  for (int bit = 0; bit < 32; bit++) {
    word = (word << 1) | (digitalRead(SPI_MISO) ? 1 : 0);  // read the data line while the clock is low
    digitalWrite(SPI_SCK, HIGH);                      // one clock pulse moves the chip to the next bit
    delayMicroseconds(10);
    digitalWrite(SPI_SCK, LOW);
    delayMicroseconds(10);
  }
  digitalWrite(MAX31855_CS_PIN, HIGH);                // stop talking to the chip
  return word;
}

void tcs_run_wiring_test() {
  if (tcs_enabled) {
    say("[TCS] Turn the TCS off first (command 10), then run the wiring test.");
    return;
  }
  say("[TCS] Thermocouple wiring test (reads the chip two ways, takes about a second)...");

  SPI.end();                                          // free the pins so we can wiggle them by hand
  digitalWrite(SD_CS_PIN, HIGH);                      // keep the SD card quiet during the test
  uint32_t withPullUp   = bit_bang_read_word(INPUT_PULLUP);
  uint32_t withPullDown = bit_bang_read_word(INPUT_PULLDOWN);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);             // give the pins back to the SPI hardware

  say(String("[TCS] Data line pulled UP:   0x") + String(withPullUp, HEX));
  say(String("[TCS] Data line pulled DOWN: 0x") + String(withPullDown, HEX));

  // Work out what that means.
  if (withPullUp == 0xFFFFFFFF && withPullDown == 0x00000000) {
    say("[TCS] RESULT: the data line (DO) is FLOATING. Nothing is driving it. Check: DO wire reaches GPIO 19, "
        "CS wire reaches GPIO 27, the chip has 3.3V and GND, and the breadboard rows are actually connected.");
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

bool           tcs_is_enabled()             { return tcs_enabled; }
TcsState       tcs_get_state()              { return state; }
TcsFaultReason tcs_get_fault_reason()       { return faultReason; }
float          tcs_get_panel_temp_c()       { return panelTempC; }
float          tcs_get_chip_temp_c()        { return chipTempC; }
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
  line += ", panel=" + (isnan(panelTempC) ? String("n/a") : String(panelTempC, 2) + "C");
  line += ", rate=" + String(tempRateCPerS, 2) + "C/s";
  line += ", louvers=" + String(louverPercent, 0) + "%";
  line += ", wheel=" + String(wheelRpm, 0) + "RPM";
  line += ", vent_error=" + (isnan(ventErrorDeg) ? String("n/a") : String(ventErrorDeg, 1) + "deg");
  if (state == TCS_FAULT) line += String(", fault=") + tcs_fault_name(faultReason);
  say(line);
  if (isnan(panelTempC)) {   // explain why there is no temperature
    say(String("[TCS] No temperature: ") + describe_thermocouple_fault(lastFaultBits));
  }
}

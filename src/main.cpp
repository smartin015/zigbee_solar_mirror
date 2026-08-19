/**
 * @file main.cpp
 * @brief Zigbee solar-mirror router on the Seeed XIAO ESP32C6.
 *
 * A two-servo (ST3215) pan/tilt mirror that reflects incoming sunlight
 * toward a commanded target direction. The device is a Zigbee ROUTER and
 * exposes three vector fields as writable/readable analog-output endpoints:
 *
 *   incoming    EP10-12: unit vector from the mirror toward the sun
 *   reflect     EP13-15: unit vector from the mirror toward the target
 *   calibration EP16-18: mirror normal when both servos are at 2048
 *
 * Servo roles:
 *   servo ID 1 -> vertical tilt
 *   servo ID 2 -> horizontal pan
 *
 * Zigbee usage follows the pattern in ~/zigbee_tcs34725 (Arduino Zigbee
 * library, analog endpoints, factory-reset button, ZCZR build flag).
 *
 * Factory reset: hold BOOT (GPIO9) for ~3 s. Clears Zigbee NVRAM and the
 * stored calibration vector.
 */

#include <Arduino.h>
#ifndef ZIGBEE_MODE_ZCZR
  #ifndef ZIGBEE_MODE_ED
    #error "Zigbee mode not selected: define ZIGBEE_MODE_ZCZR or ZIGBEE_MODE_ED in platformio.ini"
  #endif
#endif

#include "Zigbee.h"
#include "MirrorController.h"

// === Pin definitions (XIAO ESP32C6) ===
#define SERVO_PIN         D8   // GPIO19 -> both ST3215 DATA lines
#define FACTORY_BTN_PIN   D1   // GPIO9  -> BOOT button (pressed = LOW)
#define FACTORY_HOLD_MS   3000 // 3-second hold to trigger factory reset

// === Endpoint numbers ===
#define EP_INCOMING_X    10
#define EP_INCOMING_Y    11
#define EP_INCOMING_Z    12
#define EP_REFLECT_X     13
#define EP_REFLECT_Y     14
#define EP_REFLECT_Z     15
#define EP_CAL_X         16
#define EP_CAL_Y         17
#define EP_CAL_Z         18

// === Global objects ===
MirrorController mirror;

ZigbeeAnalog zbIncomingX(EP_INCOMING_X);
ZigbeeAnalog zbIncomingY(EP_INCOMING_Y);
ZigbeeAnalog zbIncomingZ(EP_INCOMING_Z);
ZigbeeAnalog zbReflectX(EP_REFLECT_X);
ZigbeeAnalog zbReflectY(EP_REFLECT_Y);
ZigbeeAnalog zbReflectZ(EP_REFLECT_Z);
ZigbeeAnalog zbCalX(EP_CAL_X);
ZigbeeAnalog zbCalY(EP_CAL_Y);
ZigbeeAnalog zbCalZ(EP_CAL_Z);

// === Factory-reset button state ===
static uint32_t btnPressStartMs = 0;
static bool btnWasPressed = false;
static bool factoryResetArmed = false;

// -------------------------------------------------------------------------
// Zigbee write callbacks
// -------------------------------------------------------------------------

static void onIncomingX(float v) { mirror.setComponent(VECTOR_INCOMING, AXIS_X, v); }
static void onIncomingY(float v) { mirror.setComponent(VECTOR_INCOMING, AXIS_Y, v); }
static void onIncomingZ(float v) { mirror.setComponent(VECTOR_INCOMING, AXIS_Z, v); }

static void onReflectX(float v) { mirror.setComponent(VECTOR_REFLECT, AXIS_X, v); }
static void onReflectY(float v) { mirror.setComponent(VECTOR_REFLECT, AXIS_Y, v); }
static void onReflectZ(float v) { mirror.setComponent(VECTOR_REFLECT, AXIS_Z, v); }

static void onCalX(float v) { mirror.setComponent(VECTOR_CALIBRATION, AXIS_X, v); }
static void onCalY(float v) { mirror.setComponent(VECTOR_CALIBRATION, AXIS_Y, v); }
static void onCalZ(float v) { mirror.setComponent(VECTOR_CALIBRATION, AXIS_Z, v); }

// -------------------------------------------------------------------------
// Endpoint helpers
// -------------------------------------------------------------------------

static void configureVectorEndpoint(
  ZigbeeAnalog &ep, const char *model, const char *description, void (*onChange)(float)
) {
  ep.addAnalogOutput();
  ep.setManufacturerAndModel("Seeed Studio", model);
  ep.setAnalogOutputDescription(description);
  ep.setAnalogOutputMinMax(-1.0f, 1.0f);
  ep.setAnalogOutputResolution(0.0001f);
  ep.setAnalogOutputApplication(ESP_ZB_ZCL_AO_SET_APP_TYPE_WITH_ID(ESP_ZB_ZCL_AO_APP_TYPE_COUNT_UNITLESS, 0));
  ep.onAnalogOutputChange(onChange);
  Zigbee.addEndpoint(&ep);
}

// -------------------------------------------------------------------------
// Setup
// -------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println(F("==========================================="));
  Serial.println(F("  Zigbee Solar Mirror - XIAO ESP32C6"));
  Serial.println(F("  servos: ID1=tilt, ID2=pan (D8/GPIO19)"));
  Serial.println(F("==========================================="));

  // Built-in LED for factory-reset feedback.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Factory-reset button (BOOT = GPIO9).
  pinMode(FACTORY_BTN_PIN, INPUT_PULLUP);

  // Initialise the half-duplex servo link, load calibration, and home both
  // servos to position 2048.
  mirror.begin(SERVO_PIN);

  // --- Configure the three Zigbee vectors (9 scalar analog endpoints) ---
  configureVectorEndpoint(zbIncomingX, "XIAO-SolarMirror-InX", "Incoming vector X (mirror->sun, unitless)", onIncomingX);
  configureVectorEndpoint(zbIncomingY, "XIAO-SolarMirror-InY", "Incoming vector Y (mirror->sun, unitless)", onIncomingY);
  configureVectorEndpoint(zbIncomingZ, "XIAO-SolarMirror-InZ", "Incoming vector Z (mirror->sun, unitless)", onIncomingZ);

  configureVectorEndpoint(zbReflectX, "XIAO-SolarMirror-RefX", "Reflect vector X (mirror->target, unitless)", onReflectX);
  configureVectorEndpoint(zbReflectY, "XIAO-SolarMirror-RefY", "Reflect vector Y (mirror->target, unitless)", onReflectY);
  configureVectorEndpoint(zbReflectZ, "XIAO-SolarMirror-RefZ", "Reflect vector Z (mirror->target, unitless)", onReflectZ);

  configureVectorEndpoint(zbCalX, "XIAO-SolarMirror-CalX", "Calibration normal X at pan=tilt=2048", onCalX);
  configureVectorEndpoint(zbCalY, "XIAO-SolarMirror-CalY", "Calibration normal Y at pan=tilt=2048", onCalY);
  configureVectorEndpoint(zbCalZ, "XIAO-SolarMirror-CalZ", "Calibration normal Z at pan=tilt=2048", onCalZ);

  // --- Start Zigbee ---
  Serial.print(F("[ZIGBEE] Starting as "));
#if defined(ZIGBEE_MODE_ZCZR)
  Serial.println(F("ROUTER..."));
  if (!Zigbee.begin(ZIGBEE_ROUTER)) {
    Serial.println(F("[ZIGBEE] Failed to start!"));
    return;
  }
#elif defined(ZIGBEE_MODE_ED)
  Serial.println(F("END DEVICE..."));
  if (!Zigbee.begin()) {
    Serial.println(F("[ZIGBEE] Failed to start!"));
    return;
  }
#endif

  Serial.println(F("[ZIGBEE] Started. Waiting for network..."));

  // Publish the current vector values on the endpoints. This also seeds the
  // controller's raw state; the debounce in loop() coalesces these writes.
  Vec3 in = mirror.incoming();
  Vec3 refl = mirror.reflect();
  Vec3 cal = mirror.calibration();

  zbIncomingX.setAnalogOutput(in.x);
  zbIncomingY.setAnalogOutput(in.y);
  zbIncomingZ.setAnalogOutput(in.z);
  zbReflectX.setAnalogOutput(refl.x);
  zbReflectY.setAnalogOutput(refl.y);
  zbReflectZ.setAnalogOutput(refl.z);
  zbCalX.setAnalogOutput(cal.x);
  zbCalY.setAnalogOutput(cal.y);
  zbCalZ.setAnalogOutput(cal.z);
}

// -------------------------------------------------------------------------
// Main loop
// -------------------------------------------------------------------------

void loop() {
  uint32_t now = millis();

  // ---- Factory-reset button handling ----
  bool btnPressed = (digitalRead(FACTORY_BTN_PIN) == LOW);

  if (btnPressed && !btnWasPressed) {
    btnPressStartMs = now;
    Serial.println(F("[BTN] Button pressed (hold 3s for factory reset)..."));
  }

  if (btnPressed) {
    uint32_t heldMs = now - btnPressStartMs;

    if (heldMs >= FACTORY_HOLD_MS) {
      if (!factoryResetArmed) {
        factoryResetArmed = true;
        Serial.println(F("[BTN] Hold threshold reached - FACTORY RESET triggered!"));
        digitalWrite(LED_BUILTIN, HIGH);

        mirror.clearCalibration();  // erase stored calibration vector
        delay(100);
        Zigbee.factoryReset();      // erase Zigbee NVRAM + reboot
        while (1) {
          digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
          delay(200);
        }
      }
    } else {
      uint32_t blinkPeriod = 200 - (heldMs * 200 / FACTORY_HOLD_MS);
      if (blinkPeriod < 30) blinkPeriod = 30;
      digitalWrite(LED_BUILTIN, (now / blinkPeriod) % 2 == 0);
    }
  }

  if (!btnPressed && btnWasPressed) {
    Serial.println(F("[BTN] Button released - reset cancelled."));
    digitalWrite(LED_BUILTIN, LOW);
    btnPressStartMs = 0;
    factoryResetArmed = false;
  }

  btnWasPressed = btnPressed;

  // ---- Zigbee connection logging ----
  static bool lastConnected = false;
  bool connected = Zigbee.connected();
  if (connected != lastConnected) {
    lastConnected = connected;
    Serial.println(connected ? F("[ZIGBEE] Connected to network!") : F("[ZIGBEE] Disconnected from network."));
  }

  // ---- Debounced mirror target recomputation ----
  mirror.update();

  delay(10);
}

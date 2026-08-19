#include "MirrorController.h"

#include <math.h>
#include <Preferences.h>

// ST3215 constants
static const int16_t SERVO_MID = 2048;
static const int32_t SERVO_RANGE = 4096;
static const uint16_t SERVO_SPEED = 1500;  // steps/sec
static const uint8_t SERVO_ACC = 50;

static const uint8_t SERVO_TILT_ID = 1;  // vertical tilt
static const uint8_t SERVO_PAN_ID = 2;   // horizontal pan

// Which direction a positive servo step moves each axis. Flip these if the
// physical mount moves the mirror in the opposite direction.
static const int8_t PAN_DIRECTION = 1;
static const int8_t TILT_DIRECTION = 1;

// Time to wait after the last Zigbee vector write before recomputing, so the
// three X/Y/Z attribute writes arrive as one logical update.
static const uint32_t DEBOUNCE_MS = 80;

static const char *PREFS_NAMESPACE = "solar_mirror";
static const char *PREFS_CAL_X = "cal_x";
static const char *PREFS_CAL_Y = "cal_y";
static const char *PREFS_CAL_Z = "cal_z";

static const Vec3 DEFAULT_CALIBRATION = {1.0f, 0.0f, 0.0f};

MirrorController::MirrorController()
  : _incoming{0.0f, 0.0f, 0.0f},
    _reflect{0.0f, 0.0f, 0.0f},
    _calibrationRaw(DEFAULT_CALIBRATION),
    _calibration(DEFAULT_CALIBRATION),
    _incomingDirty(false),
    _reflectDirty(false),
    _calibrationDirty(false),
    _servosOk(false),
    _pan(SERVO_MID),
    _tilt(SERVO_MID),
    _dirty(false),
    _dirtyMs(0) {}

void MirrorController::begin(int8_t dataPin, HardwareSerial &serial) {
  _servo.begin(dataPin, serial);
  _servosOk = true;  // broadcast sync-write works even if ack reads fail

  uint8_t ids[2] = {SERVO_TILT_ID, SERVO_PAN_ID};
  const char *names[2] = {"tilt (ID 1)", "pan (ID 2)"};
  for (uint8_t i = 0; i < 2; i++) {
    Serial.print(F("[SERVO] Ping "));
    Serial.print(names[i]);
    Serial.print(F(" ... "));
    int id = _servo.Ping(ids[i]);
    if (id >= 0) {
      Serial.print(F("OK, enable torque ... "));
      int ok = _servo.EnableTorque(ids[i], 1);
      Serial.println(ok ? F("OK") : F("no ack"));
    } else {
      Serial.println(F("no response (broadcast movement will still be attempted)"));
    }
  }

  loadCalibration();
  Serial.printf(
    "[SERVO] Calibration vector: (%.4f, %.4f, %.4f) at pan=tilt=2048\n",
    _calibration.x, _calibration.y, _calibration.z
  );

  homeServos();
}

void MirrorController::loadCalibration() {
  Preferences prefs;
  if (!prefs.begin(PREFS_NAMESPACE, true)) {
    _calibration = DEFAULT_CALIBRATION;
    Serial.println(F("[SERVO] NVS unavailable, using default calibration (1,0,0)"));
    return;
  }

  if (prefs.isKey(PREFS_CAL_X) && prefs.isKey(PREFS_CAL_Y) && prefs.isKey(PREFS_CAL_Z)) {
    Vec3 cal = {
      prefs.getFloat(PREFS_CAL_X, DEFAULT_CALIBRATION.x),
      prefs.getFloat(PREFS_CAL_Y, DEFAULT_CALIBRATION.y),
      prefs.getFloat(PREFS_CAL_Z, DEFAULT_CALIBRATION.z),
    };
    Vec3 n = normalizedOrZero(cal);
    if (vectorNorm(n) > 0.999f) {
      _calibration = n;
    } else {
      _calibration = DEFAULT_CALIBRATION;
      Serial.println(F("[SERVO] Stored calibration is invalid, using default (1,0,0)"));
    }
  } else {
    _calibration = DEFAULT_CALIBRATION;
    Serial.println(F("[SERVO] No calibration stored, using default (1,0,0)"));
  }
  prefs.end();
}

void MirrorController::saveCalibration() {
  Preferences prefs;
  if (!prefs.begin(PREFS_NAMESPACE, false)) {
    Serial.println(F("[SERVO] Could not open NVS to save calibration"));
    return;
  }
  prefs.putFloat(PREFS_CAL_X, _calibration.x);
  prefs.putFloat(PREFS_CAL_Y, _calibration.y);
  prefs.putFloat(PREFS_CAL_Z, _calibration.z);
  prefs.end();
}

void MirrorController::clearCalibration() {
  Preferences prefs;
  if (prefs.begin(PREFS_NAMESPACE, false)) {
    prefs.clear();
    prefs.end();
  }
  _calibration = DEFAULT_CALIBRATION;
  _calibrationRaw = DEFAULT_CALIBRATION;
  markDirty();
}

void MirrorController::setComponent(VectorKind kind, Axis axis, float value) {
  switch (kind) {
    case VECTOR_INCOMING:
      if (axis == AXIS_X) _incoming.x = value;
      else if (axis == AXIS_Y) _incoming.y = value;
      else _incoming.z = value;
      _incomingDirty = true;
      break;

    case VECTOR_REFLECT:
      if (axis == AXIS_X) _reflect.x = value;
      else if (axis == AXIS_Y) _reflect.y = value;
      else _reflect.z = value;
      _reflectDirty = true;
      break;

    case VECTOR_CALIBRATION:
      if (axis == AXIS_X) _calibrationRaw.x = value;
      else if (axis == AXIS_Y) _calibrationRaw.y = value;
      else _calibrationRaw.z = value;
      _calibrationDirty = true;
      break;
  }

  markDirty();
}

void MirrorController::update() {
  if (!_dirty) {
    return;
  }
  if (millis() - _dirtyMs < DEBOUNCE_MS) {
    return;
  }
  _dirty = false;

  bool calibrationChanged = false;
  if (_calibrationDirty) {
    _calibrationDirty = false;

    Vec3 n = normalizedOrZero(_calibrationRaw);
    if (vectorNorm(n) > 0.999f) {
      float delta = fabsf(n.x - _calibration.x) + fabsf(n.y - _calibration.y) + fabsf(n.z - _calibration.z);
      if (delta > 1e-6f) {
        _calibration = n;
        saveCalibration();
        calibrationChanged = true;
        Serial.printf(
          "[SERVO] Calibration updated: (%.4f, %.4f, %.4f) at pan=tilt=2048\n",
          _calibration.x, _calibration.y, _calibration.z
        );
      }
    } else {
      _calibrationRaw = _calibration;  // drop the invalid write
      Serial.println(F("[SERVO] Invalid calibration vector, keeping previous value"));
    }
  }

  bool incomingChanged = _incomingDirty;
  bool reflectChanged = _reflectDirty;
  _incomingDirty = false;
  _reflectDirty = false;

  if (!incomingChanged && !reflectChanged && !calibrationChanged) {
    return;
  }

  Vec3 incomingN = normalizedOrZero(_incoming);
  Vec3 reflectN = normalizedOrZero(_reflect);

  if (vectorNorm(incomingN) < 0.999f || vectorNorm(reflectN) < 0.999f) {
    Serial.println(F("[MIRROR] Waiting for valid incoming and reflect unit vectors"));
    return;
  }

  // The mirror normal is the half-rotation (angle bisector) between the
  // mirror->sun vector and the mirror->target vector.
  Vec3 sum = {incomingN.x + reflectN.x, incomingN.y + reflectN.y, incomingN.z + reflectN.z};
  Vec3 normal = normalizedOrZero(sum);
  if (vectorNorm(normal) < 0.999f) {
    Serial.println(F("[MIRROR] Incoming and reflect vectors are opposite; no unique mirror normal"));
    return;
  }

  int16_t pan = SERVO_MID;
  int16_t tilt = SERVO_MID;
  if (!computeServoPositions(normal, pan, tilt)) {
    Serial.println(F("[MIRROR] Could not compute servo positions (bad calibration)"));
    return;
  }

  moveServos(pan, tilt);

  Serial.printf(
    "[MIRROR] incoming=(%.3f,%.3f,%.3f) reflect=(%.3f,%.3f,%.3f) normal=(%.3f,%.3f,%.3f) -> pan=%d tilt=%d\n",
    incomingN.x, incomingN.y, incomingN.z,
    reflectN.x, reflectN.y, reflectN.z,
    normal.x, normal.y, normal.z,
    pan, tilt
  );
}

void MirrorController::homeServos() {
  Serial.println(F("[SERVO] Homing both servos to 2048"));
  moveServos(SERVO_MID, SERVO_MID);
}

void MirrorController::markDirty() {
  _dirty = true;
  _dirtyMs = millis();
}

void MirrorController::moveServos(int16_t pan, int16_t tilt) {
  _pan = pan;
  _tilt = tilt;

  if (!_servosOk) {
    Serial.printf("[SERVO] Target pan=%d tilt=%d (servo link not ready)\n", pan, tilt);
    return;
  }

  uint8_t ids[2] = {SERVO_TILT_ID, SERVO_PAN_ID};
  int16_t positions[2] = {tilt, pan};
  uint16_t speeds[2] = {SERVO_SPEED, SERVO_SPEED};
  uint8_t accs[2] = {SERVO_ACC, SERVO_ACC};

  _servo.SyncWritePosEx(ids, 2, positions, speeds, accs);
}

bool MirrorController::computeServoPositions(const Vec3 &normal, int16_t &pan, int16_t &tilt) const {
  Vec3 cal = normalizedOrZero(_calibration);
  if (vectorNorm(cal) < 0.999f) {
    return false;
  }

  const float stepsPerRad = (float)SERVO_RANGE / (2.0f * (float)M_PI);

  float phi0 = asinf(clampF(cal.z, -1.0f, 1.0f));
  float theta0 = atan2f(cal.y, cal.x);

  float phi = asinf(clampF(normal.z, -1.0f, 1.0f));
  float theta = atan2f(normal.y, normal.x);

  float dTheta = wrapPi(theta - theta0);
  float dPhi = phi - phi0;

  int32_t panSteps = (int32_t)SERVO_MID + (int32_t)lroundf((float)PAN_DIRECTION * dTheta * stepsPerRad);
  int32_t tiltSteps = (int32_t)SERVO_MID + (int32_t)lroundf((float)TILT_DIRECTION * dPhi * stepsPerRad);

  panSteps = wrapInt(panSteps, SERVO_RANGE);
  tiltSteps = constrain((int32_t)tiltSteps, (int32_t)0, (int32_t)(SERVO_RANGE - 1));

  pan = (int16_t)panSteps;
  tilt = (int16_t)tiltSteps;
  return true;
}

Vec3 MirrorController::normalizedOrZero(const Vec3 &v) {
  float len = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
  if (len < 1e-6f) {
    return {0.0f, 0.0f, 0.0f};
  }
  return {v.x / len, v.y / len, v.z / len};
}

float MirrorController::vectorNorm(const Vec3 &v) {
  return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

float MirrorController::wrapPi(float angle) {
  while (angle > (float)M_PI) {
    angle -= 2.0f * (float)M_PI;
  }
  while (angle < -(float)M_PI) {
    angle += 2.0f * (float)M_PI;
  }
  return angle;
}

float MirrorController::clampF(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

int32_t MirrorController::wrapInt(int32_t v, int32_t mod) {
  v %= mod;
  if (v < 0) {
    v += mod;
  }
  return v;
}

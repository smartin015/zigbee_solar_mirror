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

// Per-servo calibration offsets are relative to the 2048 mid position and
// are clamped to one half-turn so the calibration pose stays in 0..4095.
static const int16_t OFFSET_MIN = -2048;
static const int16_t OFFSET_MAX = 2047;

// Time to wait after the last Zigbee write before recomputing, so the three
// X/Y/Z attribute writes arrive as one logical update.
static const uint32_t DEBOUNCE_MS = 80;

static const char *PREFS_NAMESPACE = "solar_mirror";
static const char *PREFS_PAN_OFFSET = "pan_off";
static const char *PREFS_TILT_OFFSET = "tilt_off";

MirrorController::MirrorController()
  : _incoming{0.0f, 0.0f, 0.0f},
    _reflect{0.0f, 0.0f, 0.0f},
    _panOffset(0),
    _tiltOffset(0),
    _incomingDirty(false),
    _reflectDirty(false),
    _offsetsDirty(false),
    _targetModeDirty(false),
    _targetMode(MODE_HALF_ANGLE),
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

  loadOffsets();
  Serial.printf(
    "[SERVO] Calibration pose (+X normal): pan=%d tilt=%d (offsets pan=%d tilt=%d)\n",
    SERVO_MID + _panOffset, SERVO_MID + _tiltOffset, _panOffset, _tiltOffset
  );

  homeServos();
}

void MirrorController::loadOffsets() {
  Preferences prefs;
  if (!prefs.begin(PREFS_NAMESPACE, true)) {
    _panOffset = 0;
    _tiltOffset = 0;
    Serial.println(F("[SERVO] NVS unavailable, using zero servo offsets"));
    return;
  }

  _panOffset = (int16_t)constrain((int32_t)prefs.getShort(PREFS_PAN_OFFSET, 0), (int32_t)OFFSET_MIN, (int32_t)OFFSET_MAX);
  _tiltOffset = (int16_t)constrain((int32_t)prefs.getShort(PREFS_TILT_OFFSET, 0), (int32_t)OFFSET_MIN, (int32_t)OFFSET_MAX);
  prefs.end();

  if (_panOffset == 0 && _tiltOffset == 0) {
    Serial.println(F("[SERVO] No servo offsets stored, using zero offsets"));
  }
}

void MirrorController::saveOffsets() {
  Preferences prefs;
  if (!prefs.begin(PREFS_NAMESPACE, false)) {
    Serial.println(F("[SERVO] Could not open NVS to save servo offsets"));
    return;
  }
  prefs.putShort(PREFS_PAN_OFFSET, _panOffset);
  prefs.putShort(PREFS_TILT_OFFSET, _tiltOffset);
  prefs.end();
}

void MirrorController::clearOffsets() {
  Preferences prefs;
  if (prefs.begin(PREFS_NAMESPACE, false)) {
    prefs.clear();
    prefs.end();
  }
  _panOffset = 0;
  _tiltOffset = 0;
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
  }

  markDirty();
}

void MirrorController::setPanOffset(float value) {
  int16_t v = (int16_t)lroundf(value);
  if (v < OFFSET_MIN) v = OFFSET_MIN;
  if (v > OFFSET_MAX) v = OFFSET_MAX;

  if (v == _panOffset) {
    return;
  }

  _panOffset = v;
  _offsetsDirty = true;
  markDirty();
}

void MirrorController::setTiltOffset(float value) {
  int16_t v = (int16_t)lroundf(value);
  if (v < OFFSET_MIN) v = OFFSET_MIN;
  if (v > OFFSET_MAX) v = OFFSET_MAX;

  if (v == _tiltOffset) {
    return;
  }

  _tiltOffset = v;
  _offsetsDirty = true;
  markDirty();
}

void MirrorController::setTargetMode(uint8_t mode) {
  if (mode > MODE_CALIBRATION) {
    mode = MODE_CALIBRATION;
  }
  if (mode == _targetMode) {
    return;
  }

  _targetMode = mode;
  _targetModeDirty = true;
  markDirty();

  const char *names[] = {"half_angle", "incoming", "reflect", "calibration"};
  Serial.printf("[MIRROR] Target mode set to %u (%s)\n", _targetMode, names[_targetMode]);
}

void MirrorController::update() {
  if (!_dirty) {
    return;
  }
  if (millis() - _dirtyMs < DEBOUNCE_MS) {
    return;
  }
  _dirty = false;

  bool offsetsChanged = false;
  if (_offsetsDirty) {
    _offsetsDirty = false;
    saveOffsets();
    offsetsChanged = true;
    Serial.printf("[SERVO] Servo offsets updated: pan=%d tilt=%d\n", _panOffset, _tiltOffset);
  }

  bool incomingChanged = _incomingDirty;
  bool reflectChanged = _reflectDirty;
  bool modeChanged = _targetModeDirty;
  _incomingDirty = false;
  _reflectDirty = false;
  _targetModeDirty = false;

  if (!incomingChanged && !reflectChanged && !offsetsChanged && !modeChanged) {
    return;
  }

  // Calibration mode moves to the fixed +X calibration pose. Because this
  // branch is reached whenever offsets change, the servo positions live
  // update as new offset counts are written.
  if (_targetMode == MODE_CALIBRATION) {
    int16_t pan = SERVO_MID;
    int16_t tilt = SERVO_MID;
    calibrationPose(pan, tilt);
    Serial.printf("[MIRROR] Calibration mode: +X pose -> pan=%d tilt=%d\n", pan, tilt);
    moveServos(pan, tilt);
    return;
  }

  Vec3 incomingN = normalizedOrZero(_incoming);
  Vec3 reflectN = normalizedOrZero(_reflect);

  Vec3 normal = {0.0f, 0.0f, 0.0f};

  if (_targetMode == MODE_INCOMING) {
    if (vectorNorm(incomingN) < 0.999f) {
      Serial.println(F("[MIRROR] Waiting for a valid incoming unit vector"));
      return;
    }
    normal = incomingN;
  } else if (_targetMode == MODE_REFLECT) {
    if (vectorNorm(reflectN) < 0.999f) {
      Serial.println(F("[MIRROR] Waiting for a valid reflect unit vector"));
      return;
    }
    normal = reflectN;
  } else {  // MODE_HALF_ANGLE
    if (vectorNorm(incomingN) < 0.999f || vectorNorm(reflectN) < 0.999f) {
      Serial.println(F("[MIRROR] Waiting for valid incoming and reflect unit vectors"));
      return;
    }

    // The mirror normal is the half-rotation (angle bisector) between the
    // mirror->sun vector and the mirror->target vector.
    Vec3 sum = {incomingN.x + reflectN.x, incomingN.y + reflectN.y, incomingN.z + reflectN.z};
    normal = normalizedOrZero(sum);
    if (vectorNorm(normal) < 0.999f) {
      Serial.println(F("[MIRROR] Incoming and reflect vectors are opposite; no unique mirror normal"));
      return;
    }
  }

  int16_t pan = SERVO_MID;
  int16_t tilt = SERVO_MID;
  if (!computeServoPositions(normal, pan, tilt)) {
    Serial.println(F("[MIRROR] Could not compute servo positions"));
    return;
  }

  moveServos(pan, tilt);

  const char *modeName = _targetMode == MODE_INCOMING ? "incoming" : (_targetMode == MODE_REFLECT ? "reflect" : "half_angle");
  Serial.printf(
    "[MIRROR] mode=%s incoming=(%.3f,%.3f,%.3f) reflect=(%.3f,%.3f,%.3f) normal=(%.3f,%.3f,%.3f) -> pan=%d tilt=%d\n",
    modeName,
    incomingN.x, incomingN.y, incomingN.z,
    reflectN.x, reflectN.y, reflectN.z,
    normal.x, normal.y, normal.z,
    pan, tilt
  );
}

void MirrorController::homeServos() {
  int16_t pan = SERVO_MID;
  int16_t tilt = SERVO_MID;
  calibrationPose(pan, tilt);
  Serial.printf("[SERVO] Homing to calibration pose: pan=%d tilt=%d\n", pan, tilt);
  moveServos(pan, tilt);
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
  const float stepsPerRad = (float)SERVO_RANGE / (2.0f * (float)M_PI);

  // The calibration normal is fixed at +X, i.e. theta=0, phi=0.
  float phi = asinf(clampF(normal.z, -1.0f, 1.0f));
  float theta = atan2f(normal.y, normal.x);

  int32_t panSteps = (int32_t)SERVO_MID + _panOffset + (int32_t)lroundf((float)PAN_DIRECTION * theta * stepsPerRad);
  int32_t tiltSteps = (int32_t)SERVO_MID + _tiltOffset + (int32_t)lroundf((float)TILT_DIRECTION * phi * stepsPerRad);

  panSteps = wrapInt(panSteps, SERVO_RANGE);
  tiltSteps = constrain(tiltSteps, (int32_t)0, (int32_t)(SERVO_RANGE - 1));

  pan = (int16_t)panSteps;
  tilt = (int16_t)tiltSteps;
  return true;
}

void MirrorController::calibrationPose(int16_t &pan, int16_t &tilt) const {
  int32_t panSteps = wrapInt((int32_t)SERVO_MID + _panOffset, SERVO_RANGE);
  int32_t tiltSteps = constrain((int32_t)SERVO_MID + _tiltOffset, (int32_t)0, (int32_t)(SERVO_RANGE - 1));

  pan = (int16_t)panSteps;
  tilt = (int16_t)tiltSteps;
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

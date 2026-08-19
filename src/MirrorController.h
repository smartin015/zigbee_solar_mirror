#pragma once

#include <Arduino.h>
#include "ST3215HalfDuplex.h"

// Minimal 3D vector used by the mirror geometry.
struct Vec3 {
  float x;
  float y;
  float z;
};

// Which of the three Zigbee-exposed vectors a component belongs to.
enum VectorKind {
  VECTOR_INCOMING = 0,   // unit vector from the mirror toward the sun
  VECTOR_REFLECT  = 1,   // unit vector from the mirror toward the target
  VECTOR_CALIBRATION = 2 // mirror normal when both servos are at 2048
};

enum Axis {
  AXIS_X = 0,
  AXIS_Y = 1,
  AXIS_Z = 2,
};

// How the incoming/reflect vectors are interpreted when moving the mirror.
enum TargetMode {
  MODE_HALF_ANGLE = 0,  // mirror normal = normalize(incoming + reflect)
  MODE_INCOMING = 1,    // mirror normal = normalize(incoming)
  MODE_REFLECT = 2,     // mirror normal = normalize(reflect)
  MODE_CALIBRATION = 3, // move to the calibration pose (pan=tilt=2048)
};

//
// MirrorController owns the two ST3215 servos and translates the three
// Zigbee vectors into servo positions.
//
// Servo roles (as requested):
//   servo ID 1 -> vertical tilt
//   servo ID 2 -> horizontal pan
//
// Coordinate convention (right-handed, Z = up):
//   A mirror normal n is represented by
//     elevation  phi = asin(n.z)
//     azimuth    theta = atan2(n.y, n.x)
//   The calibration vector n0 is the normal measured with both servos at
//   position 2048, giving (phi0, theta0). Moving a servo by d steps moves
//   that axis by d * (2*pi / 4096) radians from the calibrated pose.
//
class MirrorController {
public:
  MirrorController();

  // Start the half-duplex servo link, enable torque, load calibration from
  // NVS, and drive both servos to their calibration pose (2048).
  void begin(int8_t dataPin, HardwareSerial &serial = Serial1);

  // Load/erase the calibration vector stored in NVS.
  void loadCalibration();
  void saveCalibration();
  void clearCalibration();

  // Called by the Zigbee callbacks whenever a vector component is written.
  void setComponent(VectorKind kind, Axis axis, float value);

  // Select how incoming/reflect are interpreted (TargetMode values above).
  void setTargetMode(uint8_t mode);
  uint8_t targetMode() const { return _targetMode; }

  // Call from loop(). Recomputes and commands the servos after a short
  // debounce so a burst of X/Y/Z writes coalesces into one movement.
  void update();

  // Move both servos back to their calibration pose.
  void homeServos();

  Vec3 incoming() const   { return _incoming; }
  Vec3 reflect() const    { return _reflect; }
  Vec3 calibration() const { return _calibration; }
  bool servosOk() const   { return _servosOk; }
  int16_t lastPan() const  { return _pan; }
  int16_t lastTilt() const { return _tilt; }

private:
  void markDirty();
  void moveServos(int16_t pan, int16_t tilt);
  bool computeServoPositions(const Vec3 &normal, int16_t &pan, int16_t &tilt) const;

  static Vec3 normalizedOrZero(const Vec3 &v);
  static float vectorNorm(const Vec3 &v);
  static float wrapPi(float angle);
  static float clampF(float v, float lo, float hi);
  static int32_t wrapInt(int32_t v, int32_t mod);

  SMS_STS_HalfDuplex _servo;

  Vec3 _incoming;        // raw components written over Zigbee
  Vec3 _reflect;         // raw components written over Zigbee
  Vec3 _calibrationRaw;  // raw calibration components written over Zigbee
  Vec3 _calibration;     // last valid unit calibration vector (persisted)

  bool _incomingDirty;
  bool _reflectDirty;
  bool _calibrationDirty;
  bool _targetModeDirty;

  uint8_t _targetMode;

  bool _servosOk;
  int16_t _pan;
  int16_t _tilt;

  bool _dirty;
  uint32_t _dirtyMs;
};

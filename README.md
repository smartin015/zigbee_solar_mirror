# Zigbee Solar Mirror (XIAO ESP32C6 + ST3215 servos)

A Zigbee **router** device that steers a two-axis mirror so incoming sunlight is
reflected toward a commanded target. The mirror normal is pointed at the
half-angle between the sun vector and the target vector.

This project is derived from two earlier projects:

- `~/xiao_servo_waveshare_ST3215` — ST3215 half-duplex servo driver and wiring.
- `~/zigbee_tcs34725` — Arduino Zigbee library usage on the XIAO ESP32C6.

## Hardware

- Seeed Studio XIAO ESP32C6
- 2x Waveshare ST3215 bus servos (or STS3215-compatible)
- External servo power supply, 6–12.6 V

Wiring is identical to the ST3215 demo project:

| XIAO pin | Servo connector (5264-3A) |
|----------|---------------------------|
| D8 (GPIO19) | pin 1 DATA (both servos, daisy-chained) |
| GND       | pin 3 GND (both servos) |
| —         | pin 2 VCC -> external 6–12.6 V (NOT 3V3) |

Servo IDs must be unique:

| Servo | ID | Axis |
|-------|----|------|
| 1     | 1  | vertical tilt |
| 2     | 2  | horizontal pan |

## Zigbee interface

One endpoint per vector component. All are **writable and readable** analog
outputs. Vector components are `-1.0 .. 1.0` (resolution 0.0001, unitless);
`target_mode` is `0 .. 3` (resolution 1).

| Endpoint | Field |
|----------|-------|
| EP10 | incoming vector X |
| EP11 | incoming vector Y |
| EP12 | incoming vector Z |
| EP13 | reflect vector X |
| EP14 | reflect vector Y |
| EP15 | reflect vector Z |
| EP16 | calibration vector X |
| EP17 | calibration vector Y |
| EP18 | calibration vector Z |
| EP19 | target mode |

### Vector conventions

- `incoming` — unit vector pointing **from the mirror toward the sun**.
- `reflect` — unit vector pointing **from the mirror toward the target**.
- `calibration` — unit mirror normal measured when **both servos are at
  position 2048** (the boot/home pose).

World frame is right-handed with **Z = up**; azimuth is measured in the XY
plane with `theta = atan2(y, x)`, elevation is `phi = asin(z)`.

The mirror normal is the angle bisector:

```text
normal = normalize(incoming + reflect)
```

That is the half-rotation between the sun vector and the target vector, which
is the mirror orientation required by the reflection law.

## Servo mapping

- `pan  = 2048 + PAN_DIRECTION  * round((theta - theta0) * 4096 / 2pi)`
- `tilt = 2048 + TILT_DIRECTION * round((phi   - phi0  ) * 4096 / 2pi)`

where `(theta0, phi0)` are the azimuth/elevation of the calibration vector.
Pan wraps at 0/4095; tilt is clamped to `0..4095`.

If a servo moves the wrong way on the real mount, flip `PAN_DIRECTION` or
`TILT_DIRECTION` in `src/MirrorController.cpp` and reflash.

## Calibration

1. Power on the device. On boot it homes both servos to position 2048.
2. Physically measure (or estimate) the mirror normal unit vector in that
   home pose.
3. Write that vector to EP16/EP17/EP18 (X, Y, Z). The firmware normalizes it,
   stores it in NVS, and uses it as the reference for all subsequent moves.

Until calibration is written, the firmware uses a default of `(1, 0, 0)`
(mirror normal pointing along +X at the home pose).

## Operation

The mirror can be commanded to move to any of the three vectors, or to the
half-angle between the sun and target vectors. The `target_mode` endpoint
(EP19) selects the interpretation:

| `target_mode` | Mirror normal target |
|---------------|----------------------|
| `0` (`half_angle`) | `normalize(incoming + reflect)` — reflection mode (default) |
| `1` (`incoming`)   | `normalize(incoming)` — point the mirror at the sun vector |
| `2` (`reflect`)    | `normalize(reflect)` — point the mirror at the target vector |
| `3` (`calibration`) | move both servos to the calibration pose (`pan=tilt=2048`) |

Write the required vector components first, then write the mode (or change
the mode while the vectors are already set). The firmware waits ~80 ms after
the last write so X/Y/Z bursts coalesce, then commands both servos in a single
ST3215 sync-write frame.

Writing a **new calibration vector** also homes the mirror to the calibration
pose (both servos at 2048), in addition to updating the stored reference.

In `half_angle` mode, if either vector is zero or the two vectors are exactly
opposite, the firmware keeps the previous servo positions and logs a message.

## Zigbee2MQTT

An external converter is provided in `zigbee2mqtt/solar-mirror.mjs`.
Copy it to Zigbee2MQTT's `external_converters/` directory (and enable
`external_js` / `enable_external_js` as required by your Zigbee2MQTT
version).

The converter matches the firmware's Basic-cluster model ID
`XIAO-SolarMirror` and maps the ten analog-output endpoints to these
MQTT properties:

| MQTT property | Endpoint |
|---------------|----------|
| `incoming_x`, `incoming_y`, `incoming_z` | EP10-12 |
| `reflect_x`, `reflect_y`, `reflect_z` | EP13-15 |
| `calibration_x`, `calibration_y`, `calibration_z` | EP16-18 |
| `target_mode` | EP19 |

Vector components are read/write (`access.ALL`) floats in `[-1, 1]`;
`target_mode` is exposed as the enum `half_angle`, `incoming`, `reflect`,
or `calibration`.

Example MQTT commands:

```text
# Reflection mode: mirror normal = half-angle between sun and target
zigbee2mqtt/SOLAR_MIRROR/set/incoming_x 0.0
zigbee2mqtt/SOLAR_MIRROR/set/incoming_y 0.0
zigbee2mqtt/SOLAR_MIRROR/set/incoming_z 1.0
zigbee2mqtt/SOLAR_MIRROR/set/reflect_x  1.0
zigbee2mqtt/SOLAR_MIRROR/set/reflect_y  0.0
zigbee2mqtt/SOLAR_MIRROR/set/reflect_z  0.0
zigbee2mqtt/SOLAR_MIRROR/set/target_mode half_angle

# Point the mirror directly at the incoming (sun) vector
zigbee2mqtt/SOLAR_MIRROR/set/target_mode incoming

# Point the mirror directly at the reflect (target) vector
zigbee2mqtt/SOLAR_MIRROR/set/target_mode reflect

# Home both servos to the calibration pose (pan=tilt=2048)
zigbee2mqtt/SOLAR_MIRROR/set/target_mode calibration
```

## Build and flash

```bash
pio run -t upload
pio device monitor
```

The project uses the same Zigbee partition table and build flags as
`~/zigbee_tcs34725` (`ZIGBEE_MODE_ZCZR=1`, USB CDC serial).

## Factory reset

Hold the BOOT button (GPIO9) for about 3 seconds. The built-in LED blinks
faster as the hold time approaches the threshold. Factory reset:

1. erases the stored calibration vector,
2. erases the Zigbee NVRAM (network/commissioning data), and
3. reboots the device.

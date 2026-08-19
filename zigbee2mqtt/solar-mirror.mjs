/**
 * Zigbee2MQTT external converter for the XIAO ESP32C6 solar mirror router.
 *
 * Firmware model ID: XIAO-SolarMirror (set on every endpoint's Basic cluster).
 * The device exposes ten analog-output endpoints:
 *
 *   EP10-12  incoming vector  (mirror -> sun,    unitless, -1..1)
 *   EP13-15  reflect vector   (mirror -> target, unitless, -1..1)
 *   EP16-18  calibration vector (mirror normal at pan=tilt=2048)
 *   EP19     target_mode (0=half_angle, 1=incoming, 2=reflect, 3=calibration)
 *
 * Each endpoint implements the ZCL Analog Output cluster (genAnalogOutput,
 * 0x000D) with the standard PresentValue attribute (0x0055, single float).
 */

import {getEndpointName} from 'zigbee-herdsman-converters/lib/utils';
import {access, presets} from 'zigbee-herdsman-converters/lib/exposes';

const targetModeValues = ['half_angle', 'incoming', 'reflect', 'calibration'];
const targetModeByValue = Object.fromEntries(targetModeValues.map((value, index) => [index, value]));
const targetModeByKey = Object.fromEntries(targetModeValues.map((value, index) => [value, index]));

/** @type{Record<string, import('zigbee-herdsman-converters/lib/types').Fz.Converter>} */
const fzLocal = {
  solar_mirror_analog_output: {
    cluster: 'genAnalogOutput',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
      if (msg.data.hasOwnProperty('presentValue')) {
        const endpointName = getEndpointName(msg, model, meta);
        if (!endpointName) {
          return;
        }

        if (endpointName === 'target_mode') {
          const raw = msg.data.presentValue;
          return {target_mode: targetModeByValue[raw] ?? raw};
        }

        return {[endpointName]: msg.data.presentValue};
      }
    },
  },
};

/** @type{Record<string, import('zigbee-herdsman-converters/lib/types').Tz.Converter>} */
const tzLocal = {
  solar_mirror_analog_output: {
    key: [
      'incoming_x',
      'incoming_y',
      'incoming_z',
      'reflect_x',
      'reflect_y',
      'reflect_z',
      'calibration_x',
      'calibration_y',
      'calibration_z',
      'target_mode',
    ],
    convertSet: async (entity, key, value, meta) => {
      const number = key === 'target_mode' ? (targetModeByKey[value] ?? Number(value)) : Number(value);
      await entity.write('genAnalogOutput', {presentValue: number});
      return {state: {[key]: value}};
    },
    convertGet: async (entity, key, meta) => {
      await entity.read('genAnalogOutput', ['presentValue']);
    },
  },
};

const vectorEndpoints = {
  incoming_x: 10,
  incoming_y: 11,
  incoming_z: 12,
  reflect_x: 13,
  reflect_y: 14,
  reflect_z: 15,
  calibration_x: 16,
  calibration_y: 17,
  calibration_z: 18,
  target_mode: 19,
};

const vectorDescriptions = {
  incoming_x: 'Incoming vector X (mirror -> sun, unitless)',
  incoming_y: 'Incoming vector Y (mirror -> sun, unitless)',
  incoming_z: 'Incoming vector Z (mirror -> sun, unitless)',
  reflect_x: 'Reflect vector X (mirror -> target, unitless)',
  reflect_y: 'Reflect vector Y (mirror -> target, unitless)',
  reflect_z: 'Reflect vector Z (mirror -> target, unitless)',
  calibration_x: 'Calibration normal X at pan=tilt=2048',
  calibration_y: 'Calibration normal Y at pan=tilt=2048',
  calibration_z: 'Calibration normal Z at pan=tilt=2048',
  target_mode: 'Servo target mode: half_angle, incoming, reflect, or calibration',
};

const exposes = Object.keys(vectorEndpoints).map((name) => {
  if (name === 'target_mode') {
    return presets.enum(name, access.ALL, targetModeValues).withDescription(vectorDescriptions[name]).withEndpoint(name);
  }

  return presets
    .numeric(name, access.ALL)
    .withValueMin(-1)
    .withValueMax(1)
    .withValueStep(0.0001)
    .withDescription(vectorDescriptions[name])
    .withEndpoint(name);
});

/** @type{import('zigbee-herdsman-converters/lib/types').DefinitionWithExtend | import('zigbee-herdsman-converters/lib/types').DefinitionWithExtend[]} */
export default {
  zigbeeModel: ['XIAO-SolarMirror'],
  model: 'XIAO-SolarMirror',
  vendor: 'Seeed Studio',
  description: 'Zigbee solar mirror router (ST3215 pan/tilt heliostat)',
  fromZigbee: [fzLocal.solar_mirror_analog_output],
  toZigbee: [tzLocal.solar_mirror_analog_output],
  exposes,
  configure: async (device, coordinatorEndpoint, definition) => {
    for (const endpointId of Object.values(vectorEndpoints)) {
      const endpoint = device.getEndpoint(endpointId);
      if (endpoint) {
        await endpoint.read('genAnalogOutput', ['presentValue']);
      }
    }
  },
  endpoint: (device) => vectorEndpoints,
};

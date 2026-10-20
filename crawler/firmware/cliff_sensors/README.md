# Crawler Cliff Sensor Firmware

## Overview

The crawler uses four VL53L4CD time-of-flight sensors connected to a Raspberry Pi Pico RP2040 through a TCA9548A I2C multiplexer. The sensors are connected to multiplexer channels 4, 5, 6, and 7.

The Pico performs the first safety classification and sends both the raw measurements and the resulting cliff states to the Raspberry Pi over USB serial. ROS can therefore publish and check the raw data while the time-critical classification remains close to the sensors.

The motor driver must still implement a communication watchdog. If current cliff data is missing, motion must be inhibited. A Pico classification cannot protect the crawler if the USB connection or ROS communication fails.

## Hardware and bus configuration

<table><thead><tr><th>Item</th><th>Configuration</th></tr></thead><tbody><tr><td>Microcontroller</td><td>Raspberry Pi Pico RP2040</td></tr><tr><td>I2C bus</td><td>`Wire1`</td></tr><tr><td>SDA</td><td>GPIO 2</td></tr><tr><td>SCL</td><td>GPIO 3</td></tr><tr><td>I2C frequency</td><td>400 kHz</td></tr><tr><td>Multiplexer</td><td>TCA9548A at address `0x70`</td></tr><tr><td>Sensor channels</td><td>4, 5, 6, 7</td></tr><tr><td>Sensor type</td><td>VL53L4CD</td></tr><tr><td>USB serial speed</td><td>115200 baud</td></tr></tbody></table>

The physical position assigned to each multiplexer channel should be documented when the sensors are mounted, for example front-left, front-right, rear-left, and rear-right.

## Cliff classification

Each sensor has one of three states:

<table><thead><tr><th>Value</th><th>State</th><th>Meaning</th></tr></thead><tbody><tr><td>0</td><td>`SAFE`</td><td>Valid and consistently below the clear threshold</td></tr><tr><td>1</td><td>`CLIFF`</td><td>Valid and consistently above the cliff threshold</td></tr><tr><td>2</td><td>`UNKNOWN`</td><td>Startup, invalid measurement, stale data, or communication error</td></tr></tbody></table>

Only `SAFE` should permit motion in the direction protected by that sensor. Both `CLIFF` and `UNKNOWN` must be treated as unsafe by the motor driver.

## Thresholds and filtering

The firmware initially uses these values:

```
constexpr uint16_t CLIFF_DETECT_MM = 170;
constexpr uint16_t CLIFF_CLEAR_MM = 145;
constexpr uint8_t CLIFF_CONFIRM_SAMPLES = 2;
constexpr uint8_t SAFE_CONFIRM_SAMPLES = 8;

```

These distances are measured from the sensor to the detected surface. They are not the height of the drop.

With a normal sensor-to-floor distance of approximately 100 mm, a 170 mm detection threshold corresponds approximately to a 70 mm increase in measured distance. The values are deliberately initial test values and must be verified on the real crawler at low speed with a physical safety barrier.

Hysteresis prevents rapid state changes near one threshold:

- A valid distance above 170 mm contributes to cliff detection.
- Two consecutive cliff samples change the state to `CLIFF`.
- A valid distance below 145 mm contributes to clearing the condition.
- Eight consecutive safe samples are required to change the state to `SAFE`.
- A distance from 145 mm through 170 mm retains the current state.
- An invalid measurement changes the state immediately to `UNKNOWN`.

This provides fast stopping and a slower, more stable release.

## Non-blocking sensor polling

The original implementation could wait up to 500 ms for each sensor. Four failed sensors could therefore delay one complete cycle by approximately two seconds.

The updated firmware polls `dataReady()` without waiting. A sensor that is not ready is checked again during the next loop iteration, so one faulty sensor does not block the other three sensors.

If a sensor has not supplied a new measurement for 250 ms, it becomes `UNKNOWN` with status 253.

## Serial protocol

The Pico outputs one data record approximately every 50 ms:

```
D,time_ms,d4,s4,c4,d5,s5,c5,d6,s6,c6,d7,s7,c7

```

For each multiplexer channel:

- `dN` is the distance in millimetres.
- `sN` is the VL53L4CD range status or a firmware-defined error status.
- `cN` is the cliff state: 0 = `SAFE`, 1 = `CLIFF`, 2 = `UNKNOWN`.

Example:

```
D,12540,102,0,0,104,0,0,65535,253,2,101,0,0

```

In this example, channels 4, 5, and 7 are safe. Channel 6 has stale data and is therefore unknown.

## Firmware-defined status values

<table><thead><tr><th>Status</th><th>Meaning</th></tr></thead><tbody><tr><td>253</td><td>No current measurement within the stale-data timeout</td></tr><tr><td>254</td><td>TCA9548A multiplexer selection failed</td></tr><tr><td>255</td><td>Sensor was not initialized successfully</td></tr></tbody></table>

VL53L4CD status 0 represents a valid range measurement. Other sensor range-status values are classified as `UNKNOWN`.

## Required ROS and motor-driver behaviour

The ROS cliff-sensor node should:

1. Parse every complete `D` record.
2. Publish raw distances, range statuses, and cliff states.
3. Reject malformed or outdated records.
4. Optionally repeat the threshold check as a plausibility check.
5. Use reliable timestamps or reception times for its watchdog logic.

The motor driver should:

- inhibit forward motion when a front sensor reports `CLIFF` or `UNKNOWN`;
- inhibit reverse motion when a rear sensor reports `CLIFF` or `UNKNOWN`;
- inhibit both directions on one side when both sensors on that side are unsafe;
- output no PWM when current cliff messages stop arriving;
- require fresh `SAFE` states before allowing motion again.

The exact combination logic belongs in the motor driver because it is the final component that controls PWM.

## Initial test procedure

1. Raise the crawler so that the tracks cannot move it unexpectedly.
2. Confirm that all four channels become `SAFE` above a normal floor.
3. Move a test surface away from one sensor and confirm `CLIFF` above 170 mm.
4. Disconnect or cover one sensor and confirm `UNKNOWN`.
5. Verify that the corresponding motor direction is inhibited for both `CLIFF` and `UNKNOWN`.
6. Test at the lowest possible speed near a simulated edge while using a physical barrier or tether.
7. Record the real measurements on level floors, small permitted drops, dark surfaces, and the actual stairs.
8. Adjust the thresholds only after reviewing those measurements.

Do not perform the first tests at an unprotected staircase. Time-of-flight sensors are part of the safety system but should not be the only physical protection during commissioning.

### Code

```c
#include <Arduino.h>
#include <Wire.h>
#include <VL53L4CD.h>

constexpr uint8_t TCA_ADDRESS = 0x70;

constexpr uint8_t SENSOR_CHANNELS[] = { 4, 5, 6, 7 };
constexpr size_t SENSOR_COUNT = sizeof(SENSOR_CHANNELS) / sizeof(SENSOR_CHANNELS[0]);

// Thresholds are absolute sensor-to-surface distances, not drop heights.
constexpr uint16_t CLIFF_DETECT_MM = 170;
constexpr uint16_t CLIFF_CLEAR_MM = 145;

// A cliff is accepted quickly, while clearing it requires stable safe readings.
constexpr uint8_t CLIFF_CONFIRM_SAMPLES = 2;
constexpr uint8_t SAFE_CONFIRM_SAMPLES = 8;

constexpr uint32_t SENSOR_STALE_MS = 250;
constexpr uint32_t OUTPUT_INTERVAL_MS = 50;

constexpr uint8_t STATUS_MUX_ERROR = 254;
constexpr uint8_t STATUS_STALE = 253;
constexpr uint8_t STATUS_NOT_INITIALIZED = 255;
constexpr uint16_t INVALID_DISTANCE_MM = 65535;

enum class CliffState : uint8_t {
  SAFE = 0,
  CLIFF = 1,
  UNKNOWN = 2
};

struct SensorState {
  uint16_t distanceMm = INVALID_DISTANCE_MM;
  uint8_t rangeStatus = STATUS_NOT_INITIALIZED;
  CliffState cliffState = CliffState::UNKNOWN;
  uint8_t cliffSamples = 0;
  uint8_t safeSamples = 0;
  uint32_t lastUpdateMs = 0;
  bool initialized = false;
};

VL53L4CD sensors[SENSOR_COUNT];
SensorState sensorStates[SENSOR_COUNT];
uint32_t lastOutputMs = 0;

bool selectMuxChannel(uint8_t channel) {
  if (channel > 7)
    return false;

  Wire1.beginTransmission(TCA_ADDRESS);
  Wire1.write(1 << channel);

  return Wire1.endTransmission() == 0;
}

bool initSensor(size_t index) {
  const uint8_t channel = SENSOR_CHANNELS[index];

  if (!selectMuxChannel(channel))
    return false;

  sensors[index].setBus(&Wire1);
  sensors[index].setTimeout(100);

  if (!sensors[index].init())
    return false;

  if (!sensors[index].setRangeTiming(50, 0))
    return false;

  sensors[index].startContinuous();
  return true;
}

void setUnknown(size_t index, uint8_t status) {
  SensorState &state = sensorStates[index];
  state.distanceMm = INVALID_DISTANCE_MM;
  state.rangeStatus = status;
  state.cliffState = CliffState::UNKNOWN;
  state.cliffSamples = 0;
  state.safeSamples = 0;
}

void classifyMeasurement(size_t index) {
  SensorState &state = sensorStates[index];

  // VL53L4CD range status 0 means a valid measurement.
  if (state.rangeStatus != 0 || state.distanceMm == INVALID_DISTANCE_MM) {
    state.cliffState = CliffState::UNKNOWN;
    state.cliffSamples = 0;
    state.safeSamples = 0;
    return;
  }

  if (state.distanceMm > CLIFF_DETECT_MM) {
    state.safeSamples = 0;
    if (state.cliffSamples < CLIFF_CONFIRM_SAMPLES)
      state.cliffSamples++;

    if (state.cliffSamples >= CLIFF_CONFIRM_SAMPLES)
      state.cliffState = CliffState::CLIFF;
    return;
  }

  if (state.distanceMm < CLIFF_CLEAR_MM) {
    state.cliffSamples = 0;
    if (state.safeSamples < SAFE_CONFIRM_SAMPLES)
      state.safeSamples++;

    if (state.safeSamples >= SAFE_CONFIRM_SAMPLES)
      state.cliffState = CliffState::SAFE;
    return;
  }

  // Inside the hysteresis band, keep the current classification.
  state.cliffSamples = 0;
  state.safeSamples = 0;
}

void pollSensor(size_t index, uint32_t now) {
  SensorState &state = sensorStates[index];

  if (!state.initialized) {
    setUnknown(index, STATUS_NOT_INITIALIZED);
    return;
  }

  if (!selectMuxChannel(SENSOR_CHANNELS[index])) {
    setUnknown(index, STATUS_MUX_ERROR);
    return;
  }

  // Do not block while waiting for a sensor. Each sensor is polled again on
  // the next loop iteration, so one failed sensor cannot delay all others.
  if (sensors[index].dataReady()) {
    state.distanceMm = sensors[index].read(false);
    state.rangeStatus = sensors[index].ranging_data.range_status;
    state.lastUpdateMs = now;
    classifyMeasurement(index);
  } else if (state.lastUpdateMs == 0 || now - state.lastUpdateMs > SENSOR_STALE_MS) {
    setUnknown(index, STATUS_STALE);
  }
}

void printMeasurements(uint32_t now) {
  Serial.print("D,");
  Serial.print(now);

  for (size_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(",");
    Serial.print(sensorStates[i].distanceMm);
    Serial.print(",");
    Serial.print(sensorStates[i].rangeStatus);
    Serial.print(",");
    Serial.print(static_cast<uint8_t>(sensorStates[i].cliffState));
  }

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin();
  Wire1.setClock(400000);

  Serial.println("# Crawler cliff sensors");
  Serial.println("# State: 0=SAFE, 1=CLIFF, 2=UNKNOWN");
  Serial.println("# Format:");
  Serial.println("# D,time_ms,d4,s4,c4,d5,s5,c5,d6,s6,c6,d7,s7,c7");

  for (size_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print("# Initializing channel ");
    Serial.print(SENSOR_CHANNELS[i]);
    Serial.print(" ... ");

    sensorStates[i].initialized = initSensor(i);
    if (sensorStates[i].initialized)
      Serial.println("OK");
    else
      Serial.println("FAILED");

    delay(50);
  }

  Serial.println("# Initialization complete");
}

void loop() {
  const uint32_t now = millis();

  for (size_t i = 0; i < SENSOR_COUNT; i++)
    pollSensor(i, now);

  if (now - lastOutputMs >= OUTPUT_INTERVAL_MS) {
    lastOutputMs = now;
    printMeasurements(now);
  }

  delay(1);
}

```
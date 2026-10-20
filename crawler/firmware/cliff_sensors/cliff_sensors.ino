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


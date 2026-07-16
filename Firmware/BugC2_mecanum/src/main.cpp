#include <Arduino.h>
#include <ArduinoOTA.h>
#ifdef USE_M5STICKC_PLUS
#include <M5StickCPlus.h>
#else
#include <M5StickC.h>
#endif
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <M5_BugMecanum.h>

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

#ifndef OTA_HOSTNAME
#define OTA_HOSTNAME "bug-mecanum"
#endif

#ifndef OTA_PASSWORD
#define OTA_PASSWORD ""
#endif

#ifndef ESPNOW_CHANNEL
#define ESPNOW_CHANNEL 1
#endif

#ifndef BUGC_I2C_SDA
#define BUGC_I2C_SDA 0
#endif

#ifndef BUGC_I2C_SCL
#define BUGC_I2C_SCL 26
#endif

#ifndef BUGC_I2C_FREQ
#define BUGC_I2C_FREQ 400000
#endif

#ifndef BUGC_SPEED_SCALE
#define BUGC_SPEED_SCALE 70
#endif

#ifndef BUGC_RUDDER_SCALE
#define BUGC_RUDDER_SCALE 45
#endif

#ifndef BUGC_DRIFT_RUDDER_SCALE
#define BUGC_DRIFT_RUDDER_SCALE 30
#endif

#ifndef BUGC_ADC_FULL_SCALE_MV
#define BUGC_ADC_FULL_SCALE_MV 9800
#endif

#ifndef BUGC_INPUT_DEADBAND_PERCENT
#define BUGC_INPUT_DEADBAND_PERCENT 12
#endif

#ifndef BUGC_MOTOR_DEADBAND
#define BUGC_MOTOR_DEADBAND 8
#endif

#ifndef BUGC_DRIFT_TURN_DEG_PER_SEC
#define BUGC_DRIFT_TURN_DEG_PER_SEC 360
#endif

#ifndef BUGC_DRIFT_YAW_SIGN
#define BUGC_DRIFT_YAW_SIGN 1
#endif

#ifndef BUGC_DRIFT_GYRO_DEADBAND_DPS
#define BUGC_DRIFT_GYRO_DEADBAND_DPS 1.5f
#endif

#ifndef BUGC_IMU_I2C_SDA
#define BUGC_IMU_I2C_SDA 21
#endif

#ifndef BUGC_IMU_I2C_SCL
#define BUGC_IMU_I2C_SCL 22
#endif

#ifndef BUGC_IMU_I2C_FREQ
#define BUGC_IMU_I2C_FREQ 400000
#endif

namespace {
constexpr uint8_t kLegacyPacketSize = 25;
constexpr uint8_t kAckPacketSize = 8;
constexpr uint8_t kAckMagic0 = 0x52;  // R
constexpr uint8_t kAckMagic1 = 0x56;  // V
constexpr uint8_t kAckMagic2 = 0x43;  // C
constexpr uint8_t kAckMagic3 = 0x41;  // A
constexpr uint32_t kPacketTimeoutMs = 500;
constexpr uint32_t kLogIntervalMs = 500;
constexpr uint32_t kDisplayIntervalMs = 200;
constexpr uint32_t kOtaWifiRetryIntervalMs = 10000;
constexpr uint32_t kOtaButtonHoldMs = 1500;
constexpr uint32_t kPowerDoubleClickMs = 350;
constexpr uint32_t kLedIntervalMs = 250;
constexpr uint16_t kGyroCalibrationSamples = 100;
constexpr uint8_t kGyroCalibrationDelayMs = 5;

struct ControlState {
  float rudder = 0.0f;
  float throttle = 0.0f;
  float aileron = 0.0f;
  float elevator = 0.0f;
  uint8_t arm = 0;
  uint8_t flip = 0;
  uint8_t rotation = 0;
  uint8_t fire = 0;
  uint8_t proactive = 0;
};

M5_BugMecanum bug;
volatile bool packetUpdated = false;
ControlState latestControl;
uint8_t latestMac[6] = {};
uint8_t ackPacket[kAckPacketSize] = {};
uint32_t lastPacketMs = 0;
uint32_t lastLogMs = 0;
uint32_t lastDisplayMs = 0;
uint32_t lastLedMs = 0;
uint32_t lastOtaWifiAttemptMs = 0;
uint32_t lastPowerClickMs = 0;
uint16_t latestBugAdcRaw = 0;
float latestBugBatteryVoltage = 0.0f;
bool bugReady = false;
bool espNowReady = false;
bool otaModeEnabled = false;
bool otaStarted = false;
bool otaButtonHandled = false;
bool motorCommandValid = false;
bool imuReady = false;
int8_t lastMotorX = 0;
int8_t lastMotorY = 0;
int8_t lastMotorZ = 0;
bool ledBlink = false;
bool driftMode = false;
bool driftButtonLatched = false;
float driftYawRad = 0.0f;
float driftGyroZOffsetDps = 0.0f;
uint32_t lastControlMs = 0;

float readFloatLE(const uint8_t *data) {
  float value;
  memcpy(&value, data, sizeof(value));
  return value;
}

bool checksumOk(const uint8_t *data, int len) {
  if (len != kLegacyPacketSize) {
    return false;
  }

  uint8_t sum = 0;
  for (uint8_t i = 0; i < kLegacyPacketSize - 1; i++) {
    sum += data[i];
  }
  return sum == data[kLegacyPacketSize - 1];
}

void addPeerIfNeeded(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) {
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void sendAck(const uint8_t *mac) {
  addPeerIfNeeded(mac);

  ackPacket[0] = kAckMagic0;
  ackPacket[1] = kAckMagic1;
  ackPacket[2] = kAckMagic2;
  ackPacket[3] = kAckMagic3;
  ackPacket[4] = bugReady ? 1 : 0;
  ackPacket[5] = latestControl.arm;
  ackPacket[6] = latestControl.flip;
  ackPacket[7] = 0;
  for (uint8_t i = 0; i < kAckPacketSize - 1; i++) {
    ackPacket[7] += ackPacket[i];
  }

  esp_now_send(mac, ackPacket, sizeof(ackPacket));
}

void parseLegacyPacket(const uint8_t *data, ControlState &state) {
  state.rudder = readFloatLE(&data[3]);
  state.throttle = readFloatLE(&data[7]);
  state.aileron = readFloatLE(&data[11]);
  state.elevator = readFloatLE(&data[15]);
  state.arm = data[19];
  state.flip = data[20];
  state.rotation = data[21];
  state.fire = data[22];
  state.proactive = data[23];
}

void writeBugSpeed(int8_t x, int8_t y, int8_t z, bool force = false) {
  if (!force && motorCommandValid && x == lastMotorX && y == lastMotorY &&
      z == lastMotorZ) {
    return;
  }

  bug.setMecanumSpeed(x, y, z);
  lastMotorX = x;
  lastMotorY = y;
  lastMotorZ = z;
  motorCommandValid = true;
}

void stopBug(bool force = false) {
  writeBugSpeed(0, 0, 0, force);
}

void resetControlTiming() {
  lastControlMs = 0;
}

bool otaIsConfigured() {
  return strlen(WIFI_SSID) > 0;
}

float applyInputDeadband(float value) {
  const float deadband = constrain(BUGC_INPUT_DEADBAND_PERCENT, 0, 50) / 100.0f;
  return fabsf(value) < deadband ? 0.0f : value;
}

int8_t applyMotorDeadband(int value) {
  const int deadband = constrain(BUGC_MOTOR_DEADBAND, 0, 30);
  return abs(value) < deadband ? 0 : (int8_t)constrain(value, -100, 100);
}

void initImu() {
  Wire1.begin(BUGC_IMU_I2C_SDA, BUGC_IMU_I2C_SCL, BUGC_IMU_I2C_FREQ);
  imuReady = M5.IMU.Init() == 0;
  driftGyroZOffsetDps = 0.0f;
  if (!imuReady) {
    return;
  }

  float sumZ = 0.0f;
  for (uint16_t i = 0; i < kGyroCalibrationSamples; i++) {
    float gyroX = 0.0f;
    float gyroY = 0.0f;
    float gyroZ = 0.0f;
    M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
    sumZ += gyroZ;
    delay(kGyroCalibrationDelayMs);
  }
  driftGyroZOffsetDps = sumZ / (float)kGyroCalibrationSamples;
}

float readDriftGyroZRadPerSec() {
  if (!imuReady) {
    return 0.0f;
  }

  float gyroX = 0.0f;
  float gyroY = 0.0f;
  float gyroZ = 0.0f;
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);

  const float yawSign = BUGC_DRIFT_YAW_SIGN < 0 ? -1.0f : 1.0f;
  const float correctedZ = (gyroZ - driftGyroZOffsetDps) * yawSign;
  const float deadband = max(0.0f, (float)BUGC_DRIFT_GYRO_DEADBAND_DPS);
  if (fabsf(correctedZ) < deadband) {
    return 0.0f;
  }
  return radians(correctedZ);
}

void updateBugBatteryVoltage() {
  if (!bugReady) {
    latestBugBatteryVoltage = 0.0f;
    latestBugAdcRaw = 0;
    return;
  }

  uint16_t raw = 0;
  if (!bug.getRawAdc12Bit(raw)) {
    latestBugBatteryVoltage = 0.0f;
    latestBugAdcRaw = 0;
    return;
  }

  latestBugAdcRaw = raw;
  latestBugBatteryVoltage =
      (float)raw * (float)BUGC_ADC_FULL_SCALE_MV / 4095.0f / 1000.0f;
}

void drawStatus(const ControlState &state, bool timedOut) {
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setCursor(0, 0);
  if (latestBugBatteryVoltage > 0.0f) {
    M5.Lcd.printf("Bat %4.2fV\n", latestBugBatteryVoltage);
  } else {
    M5.Lcd.print("Bat --.--V\n");
  }
  M5.Lcd.printf("I2C %s ESP %s\n", bugReady ? "OK" : "NG",
                espNowReady ? "RX" : "NG");
  M5.Lcd.printf("IMU %s yaw%+4.0f\n", imuReady ? "OK" : "NG",
                degrees(driftYawRad));
  M5.Lcd.printf("Link %s     \n", timedOut ? "WAIT" : "OK");
  M5.Lcd.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", latestMac[0],
                latestMac[1], latestMac[2], latestMac[3], latestMac[4],
                latestMac[5]);
  M5.Lcd.printf("x%+5.2f y%+5.2f\n", state.aileron, state.elevator);
  M5.Lcd.printf("z%+5.2f      \n", state.rudder);
  M5.Lcd.printf("mode %s \n", driftMode ? "drift " : "normal");
  M5.Lcd.printf("m%+4d%+4d%+4d\n", lastMotorX, lastMotorY, lastMotorZ);
}

void updateBugLeds(bool timedOut) {
  const uint32_t now = millis();
  if (now - lastLedMs < kLedIntervalMs) {
    return;
  }

  lastLedMs = now;
  ledBlink = !ledBlink;

  if (!bugReady) {
    bug.setColor(0x200000, 0x200000);
  } else if (timedOut) {
    const uint32_t color = ledBlink ? 0x202000 : 0x000000;
    bug.setColor(color, color);
  } else if (driftMode) {
    bug.setColor(0x000020, 0x000020);
  } else {
    bug.setColor(0x002000, 0x002000);
  }
}

void setupOTA() {
  if (otaStarted || !otaIsConfigured()) {
    return;
  }

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  if (strlen(OTA_PASSWORD) > 0) {
    ArduinoOTA.setPassword(OTA_PASSWORD);
  }

  ArduinoOTA
      .onStart([]() {
        stopBug();
        bug.setColor(0x000020, 0x000020);
        Serial.println("OTA update started");
        M5.Lcd.fillScreen(BLUE);
        M5.Lcd.setCursor(0, 10);
        M5.Lcd.print("OTA start");
      })
      .onEnd([]() {
        stopBug();
        Serial.println("\nOTA update finished");
        M5.Lcd.setCursor(0, 50);
        M5.Lcd.print("Done");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        const unsigned int percent = total == 0 ? 0 : (progress * 100) / total;
        static unsigned int lastPercent = 101;
        if (percent != lastPercent) {
          lastPercent = percent;
          Serial.printf("OTA progress: %u%%\r", percent);
          M5.Lcd.setCursor(0, 50);
          M5.Lcd.printf("%3u%%", percent);
        }
      })
      .onError([](ota_error_t error) {
        stopBug();
        Serial.printf("OTA error[%u]\n", error);
        M5.Lcd.setCursor(0, 80);
        M5.Lcd.printf("Err %u", error);
      });

  ArduinoOTA.begin();
  otaStarted = true;
  Serial.printf("OTA ready: %s.local IP: %s\n", OTA_HOSTNAME,
                WiFi.localIP().toString().c_str());
}

void serviceOTAMode() {
  if (!otaModeEnabled || !otaIsConfigured()) {
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    const uint32_t now = millis();
    if (lastOtaWifiAttemptMs != 0 &&
        now - lastOtaWifiAttemptMs < kOtaWifiRetryIntervalMs) {
      return;
    }
    lastOtaWifiAttemptMs = now;
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("Connecting OTA WiFi: %s\n", WIFI_SSID);
    M5.Lcd.setCursor(0, 50);
    M5.Lcd.print("WiFi...");
    return;
  }

  setupOTA();
  ArduinoOTA.handle();
}

void enableOTAMode() {
  if (otaModeEnabled) {
    return;
  }

  otaModeEnabled = true;
  stopBug();
  bug.setColor(0x000020, 0x000020);
  M5.Lcd.fillScreen(BLUE);
  M5.Lcd.setCursor(0, 10);
  M5.Lcd.print("OTA mode");

  if (!otaIsConfigured()) {
    Serial.println("OTA mode enabled, but WIFI_SSID is empty");
    M5.Lcd.setCursor(0, 50);
    M5.Lcd.print("No WiFi");
    return;
  }

  lastOtaWifiAttemptMs = 0;
  serviceOTAMode();
}

void checkOtaButton() {
  if (!M5.BtnA.isPressed()) {
    otaButtonHandled = false;
    return;
  }

  if (!otaButtonHandled && M5.BtnA.pressedFor(kOtaButtonHoldMs)) {
    otaButtonHandled = true;
    enableOTAMode();
  }
}

void powerOffNow() {
  stopBug(true);
  bug.setColor(0x000000, 0x000000);
  Serial.println("Power off by BtnA double click");

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.print("Power off");
  delay(200);
  M5.Axp.PowerOff();

  while (true) {
    delay(1000);
  }
}

void checkPowerButtonDoubleClick() {
  if (!M5.BtnA.wasPressed()) {
    return;
  }

  const uint32_t now = millis();
  if (lastPowerClickMs != 0 && now - lastPowerClickMs <= kPowerDoubleClickMs) {
    powerOffNow();
  }
  lastPowerClickMs = now;
}

void logStatus(const ControlState &state, bool timedOut) {
  Serial.printf("I2C:%s ESP-NOW ch:%d %s from "
                "%02X:%02X:%02X:%02X:%02X:%02X "
                "bat:%.2fV raw:%u x:%+.2f y:%+.2f z:%+.2f arm:%u flip:%u\n",
                bugReady ? "OK" : "NG", ESPNOW_CHANNEL,
                timedOut ? "timeout" : "rx", latestMac[0], latestMac[1],
                latestMac[2], latestMac[3], latestMac[4], latestMac[5],
                latestBugBatteryVoltage, latestBugAdcRaw,
                state.aileron, state.elevator, state.rudder, state.arm,
                state.flip);
}

void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (!checksumOk(data, len)) {
    return;
  }

  ControlState parsed;
  parseLegacyPacket(data, parsed);
  memcpy(latestMac, mac, sizeof(latestMac));
  latestControl = parsed;
  lastPacketMs = millis();
  packetUpdated = true;
  sendAck(mac);
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    espNowReady = false;
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
  espNowReady = true;
}

void applyControl(const ControlState &state) {
  float x = applyInputDeadband(constrain(state.aileron, -1.0f, 1.0f));
  float y = applyInputDeadband(constrain(-state.elevator, -1.0f, 1.0f));
  const float rudder = applyInputDeadband(constrain(state.rudder, -1.0f, 1.0f));
  const int scale = constrain(BUGC_SPEED_SCALE, 1, 100);
  const bool driftButtonPressed = state.fire != 0;
  const uint32_t now = millis();

  if (driftButtonPressed && !driftButtonLatched) {
    driftButtonLatched = true;
    driftMode = !driftMode;
    driftYawRad = 0.0f;
    lastControlMs = now;
    motorCommandValid = false;
  } else if (!driftButtonPressed) {
    driftButtonLatched = false;
  }

  const int rudderScale =
      driftMode ? constrain(BUGC_DRIFT_RUDDER_SCALE, 0, 100)
                : constrain(BUGC_RUDDER_SCALE, 0, 100);
  const float z = rudder * rudderScale / 100.0f;

  if (driftMode) {
    float dt = 0.0f;
    if (lastControlMs != 0) {
      dt = (now - lastControlMs) / 1000.0f;
      dt = constrain(dt, 0.0f, 0.2f);
    }
    lastControlMs = now;

    driftYawRad += readDriftGyroZRadPerSec() * dt;
    if (driftYawRad > PI) driftYawRad -= TWO_PI;
    if (driftYawRad < -PI) driftYawRad += TWO_PI;

    const float cosYaw = cosf(driftYawRad);
    const float sinYaw = sinf(driftYawRad);
    const float rotatedX = x * cosYaw - y * sinYaw;
    const float rotatedY = x * sinYaw + y * cosYaw;
    x = rotatedX;
    y = rotatedY;
  } else {
    lastControlMs = now;
  }

  writeBugSpeed(applyMotorDeadband(lroundf(x * scale)),
                applyMotorDeadband(lroundf(y * scale)),
                applyMotorDeadband(lroundf(z * scale)));
}

}  // namespace

void setup() {
  M5.begin();
  M5.update();
  M5.Lcd.setRotation(0);
  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextSize(2);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.print("BugC2 boot");

  Serial.begin(115200);
  delay(500);
  Serial.println("Bug mecanum StickC receiver");

  initImu();
  Serial.printf("IMU: %s z-offset=%.2f dps\n", imuReady ? "OK" : "NG",
                driftGyroZOffsetDps);

  bugReady = bug.begin(&Wire, BUGC_I2C_SDA, BUGC_I2C_SCL, BUGC_ADDRESS,
                       BUGC_I2C_FREQ);
  stopBug(true);
  bug.setColor(bugReady ? 0x002000 : 0x200000,
               bugReady ? 0x002000 : 0x200000);
  Serial.printf("BugC I2C SDA=%d SCL=%d: %s\n", BUGC_I2C_SDA, BUGC_I2C_SCL,
                bugReady ? "OK" : "NG");

  setupEspNow();
  Serial.printf("ESP-NOW receive channel %d\n", ESPNOW_CHANNEL);
  logStatus(latestControl, true);
}

void loop() {
  const uint32_t now = millis();
  const bool timedOut = now - lastPacketMs > kPacketTimeoutMs;

  M5.update();
  checkPowerButtonDoubleClick();
  checkOtaButton();
  if (otaModeEnabled) {
    serviceOTAMode();
    delay(1);
    return;
  }

  if (!bugReady) {
    stopBug();
    resetControlTiming();
  } else if (timedOut) {
    stopBug();
    resetControlTiming();
  } else if (packetUpdated) {
    packetUpdated = false;
    applyControl(latestControl);
  }

  updateBugLeds(timedOut);

  if (now - lastLogMs >= kLogIntervalMs) {
    lastLogMs = now;
    logStatus(latestControl, timedOut);
  }

  if (now - lastDisplayMs >= kDisplayIntervalMs) {
    lastDisplayMs = now;
    updateBugBatteryVoltage();
    drawStatus(latestControl, timedOut);
  }

  delay(1);
}

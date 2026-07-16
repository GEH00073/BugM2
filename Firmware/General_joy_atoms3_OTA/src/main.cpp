#include <ArduinoOTA.h>
#include <M5AtomS3.h>
#include <Toio.h>
#include <WiFi.h>
#include <atoms3joy.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <FS.h>
#include <SPIFFS.h>

#include "buzzer.h"

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

#ifndef OTA_HOSTNAME
#define OTA_HOSTNAME "general-joy"
#endif

#ifndef OTA_PASSWORD
#define OTA_PASSWORD ""
#endif

#ifndef ESPNOW_CHANNEL
#define ESPNOW_CHANNEL 1
#endif

namespace {
constexpr uint16_t kStickCenter = 2048;
constexpr float kStickHalfRange = 2048.0f;
constexpr uint8_t kPacketSize = 25;
constexpr uint8_t kAckPacketSize = 8;
constexpr uint8_t kRoller485PeerInfoSize = 11;
constexpr uint8_t kAckMagic0 = 0x52;  // R
constexpr uint8_t kAckMagic1 = 0x56;  // V
constexpr uint8_t kAckMagic2 = 0x43;  // C
constexpr uint8_t kAckMagic3 = 0x41;  // A
constexpr uint8_t kRoller485PeerMagic0 = 0xaa;
constexpr uint8_t kRoller485PeerMagic1 = 0x55;
constexpr uint8_t kRoller485PeerMagic2 = 0x16;
constexpr uint8_t kRoller485PeerMagic3 = 0x88;
constexpr uint32_t kOtaWifiRetryIntervalMs = 10000;
constexpr uint32_t kOtaButtonHoldMs = 1500;
constexpr uint32_t kPairingButtonHoldMs = 800;
constexpr uint32_t kEspNowSendIntervalMs = 50;
constexpr uint32_t kPeerStatusTimeoutMs = 1000;
constexpr uint32_t kDisplayIntervalMs = 100;
constexpr uint32_t kMenuMoveIntervalMs = 250;
constexpr uint32_t kPairingProbeIntervalMs = 100;
constexpr uint32_t kPairingChannelScanIntervalMs = 40;
constexpr uint32_t kBatteryAlarmBeepIntervalMs = 10000;
constexpr float kBatteryPresentMinVoltage = 3.0f;
constexpr float kBatteryWarnVoltage = 3.7f;
constexpr float kBatteryAlarmVoltage = 3.5f;
constexpr float kToioDriftTargetYawStepDeg = 1.25f;
constexpr float kToioDriftYawFeedbackDivisorDeg = 180.0f;
constexpr int kDisplayTopY = 2;
constexpr int kDisplayLineHeight = 19;
constexpr uint8_t kMaxToioCubes = 2;
constexpr uint8_t kMenuVisibleRows = 4;
constexpr const char *kToioStartupFrontNameSuffixes[] = {"486", "F8P"};
constexpr char kRoverCPeerFile[] = "/roverc_peer.txt";
constexpr char kBugCPeerFile[] = "/bugc_peer.txt";
constexpr char kMini4wdPeerFile[] = "/mini4wd_peer.txt";
constexpr char kRoller485PeerFile[] = "/roller485_peer.txt";

enum class TargetMode : uint8_t {
  Toio = 0,
  RoverC,
  BugC,
  Mini4WD,
  Roller485,
};

struct JoyState {
  float throttle = 0.0f;
  float aileron = 0.0f;
  float elevator = 0.0f;
  float rudder = 0.0f;
};

struct PeerEntry {
  bool valid = false;
  uint8_t mac[6] = {};
  uint8_t channel = ESPNOW_CHANNEL;
};

TargetMode targetMode = TargetMode::Toio;
TargetMode menuMode = TargetMode::Toio;
Toio toio;
ToioCore *toioCore[kMaxToioCubes] = {};
String toioName[kMaxToioCubes];
size_t toioCount = 0;
float toioYaw[kMaxToioCubes] = {};
bool toioDriftMode = false;
bool toioSwap = true;
float toioStartYaw = 0.0f;
float toioTargetYaw = 0.0f;
float toioDiff = 0.0f;

JoyState joy;
bool otaModeEnabled = false;
bool otaStarted = false;
bool otaButtonHandled = false;
bool espNowReady = false;
bool peerReady = false;
bool sendEspNow = true;
uint32_t espnowVersion = 0;
uint32_t lastOtaWifiAttemptMs = 0;
uint32_t lastBatteryAlarmBeepMs = 0;
uint32_t lastPeerAckMs = 0;
uint32_t lastEspNowSendMs = 0;
uint32_t lastDisplayMs = 0;
uint32_t lastMenuMoveMs = 0;
uint32_t sentCount = 0;
uint32_t sendFailCount = 0;
uint8_t peerMac[6] = {};
uint8_t broadcastAddress[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint8_t senddata[kPacketSize] = {};
esp_now_peer_info_t peerInfo = {};
PeerEntry roverCPeer;
PeerEntry bugCPeer;
PeerEntry mini4wdPeer;
PeerEntry roller485Peer;
bool pairingMode = false;
bool pairingReceived = false;
bool roller485PairingMode = false;
TargetMode pairingTargetMode = TargetMode::RoverC;
uint8_t pairingScanChannel = ESPNOW_CHANNEL;
float roller485Voltage = 0.0f;
float roller485ModeFlag = 0.0f;

const TargetMode kMainModes[] = {TargetMode::Toio, TargetMode::RoverC,
                                 TargetMode::BugC, TargetMode::Mini4WD,
                                 TargetMode::Roller485};
constexpr uint8_t kMainModeCount = sizeof(kMainModes) / sizeof(kMainModes[0]);
const TargetMode kPairingModes[] = {TargetMode::RoverC, TargetMode::BugC,
                                    TargetMode::Mini4WD,
                                    TargetMode::Roller485};
constexpr uint8_t kPairingModeCount =
    sizeof(kPairingModes) / sizeof(kPairingModes[0]);

const char *modeName(TargetMode mode) {
  switch (mode) {
    case TargetMode::Toio:
      return "toio";
    case TargetMode::RoverC:
      return "RoverC";
    case TargetMode::BugC:
      return "BugC";
    case TargetMode::Mini4WD:
      return "MINI4WD";
    case TargetMode::Roller485:
      return "Roller";
  }
  return "?";
}

bool isEspNowMode() {
  return targetMode == TargetMode::RoverC || targetMode == TargetMode::BugC ||
         targetMode == TargetMode::Mini4WD ||
         targetMode == TargetMode::Roller485;
}

bool isAckPairingMode(TargetMode mode) {
  return mode == TargetMode::RoverC || mode == TargetMode::BugC ||
         mode == TargetMode::Mini4WD;
}

bool isRoller485Mode(TargetMode mode) {
  return mode == TargetMode::Roller485;
}

bool isBroadcastMac(const uint8_t mac[6]) {
  for (uint8_t i = 0; i < 6; i++) {
    if (mac[i] != 0xff) {
      return false;
    }
  }
  return true;
}

PeerEntry *peerForMode(TargetMode mode) {
  switch (mode) {
    case TargetMode::RoverC:
      return &roverCPeer;
    case TargetMode::BugC:
      return &bugCPeer;
    case TargetMode::Mini4WD:
      return &mini4wdPeer;
    case TargetMode::Roller485:
      return &roller485Peer;
    default:
      return nullptr;
  }
}

bool savedPeerValid(TargetMode mode) {
  PeerEntry *peer = peerForMode(mode);
  return peer != nullptr && peer->valid;
}

const uint8_t *savedPeerMac(TargetMode mode) {
  if (!savedPeerValid(mode)) {
    return broadcastAddress;
  }
  return peerForMode(mode)->mac;
}

uint8_t savedPeerChannel(TargetMode mode) {
  if (!savedPeerValid(mode)) {
    return ESPNOW_CHANNEL;
  }
  return peerForMode(mode)->channel;
}

const char *peerFile(TargetMode mode) {
  switch (mode) {
    case TargetMode::RoverC:
      return kRoverCPeerFile;
    case TargetMode::BugC:
      return kBugCPeerFile;
    case TargetMode::Mini4WD:
      return kMini4wdPeerFile;
    case TargetMode::Roller485:
      return kRoller485PeerFile;
    default:
      return "";
  }
}

const uint8_t *activeTargetMac() {
  return savedPeerValid(targetMode) ? savedPeerMac(targetMode) : broadcastAddress;
}

uint8_t activeTargetChannel() {
  return savedPeerValid(targetMode) ? savedPeerChannel(targetMode)
                                    : ESPNOW_CHANNEL;
}

float normalizeStick(uint16_t raw, bool invert) {
  float value = (float)((int)raw - (int)kStickCenter) / kStickHalfRange;
  value = constrain(value, -1.0f, 1.0f);
  return invert ? -value : value;
}

float applyDeadband(float value, float deadband) {
  return fabsf(value) < deadband ? 0.0f : value;
}

float normalizeAngle180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float averageYaw(float yawA, float yawB) {
  return yawA + normalizeAngle180(yawB - yawA) * 0.5f;
}

void readJoy() {
  joy_update();
  joy.throttle = normalizeStick(getThrottle(), true) + 0.02f;
  joy.aileron = normalizeStick(getAileron(), true) - 0.04f;
  joy.elevator = normalizeStick(getElevator(), true) + 0.01f;
  joy.rudder = normalizeStick(getRudder(), false) - 0.05f;

  joy.aileron = applyDeadband(joy.aileron, 0.05f);
  joy.elevator = applyDeadband(joy.elevator, 0.05f);
  joy.rudder = applyDeadband(joy.rudder, 0.05f);
}

float getJoyBatteryVoltage() {
#ifdef NEW_ATOM_JOY
  const bool battery0Valid = Battery_voltage[0] >= kBatteryPresentMinVoltage;
  const bool battery1Valid = Battery_voltage[1] >= kBatteryPresentMinVoltage;
  if (battery0Valid && battery1Valid) {
    return min(Battery_voltage[0], Battery_voltage[1]);
  }
  if (battery0Valid) {
    return Battery_voltage[0];
  }
  if (battery1Valid) {
    return Battery_voltage[1];
  }
#else
  if (Battery_voltage[0] >= kBatteryPresentMinVoltage) {
    return Battery_voltage[0];
  }
#endif
  return 0.0f;
}

uint16_t getJoyBatteryColor(float voltage) {
  if (voltage <= 0.0f) {
    return WHITE;
  }
  if (voltage <= kBatteryAlarmVoltage) {
    return RED;
  }
  if (voltage <= kBatteryWarnVoltage) {
    return YELLOW;
  }
  return GREEN;
}

void serviceJoyBatteryAlarm(float voltage) {
  if (voltage <= 0.0f || voltage > kBatteryAlarmVoltage) {
    return;
  }

  const uint32_t now = millis();
  if (lastBatteryAlarmBeepMs == 0 ||
      now - lastBatteryAlarmBeepMs >= kBatteryAlarmBeepIntervalMs) {
    lastBatteryAlarmBeepMs = now;
    beep();
  }
}

void printJoyBatteryVoltage() {
  const float voltage = getJoyBatteryVoltage();
  serviceJoyBatteryAlarm(voltage);

  M5.Lcd.setCursor(0, kDisplayTopY);
  M5.Lcd.setTextColor(getJoyBatteryColor(voltage), BLACK);
  if (voltage > 0.0f) {
    M5.Lcd.printf("BAT %3.1fV   ", voltage);
  } else {
    M5.Lcd.print("BAT --.-V   ");
  }
  M5.Lcd.setTextColor(WHITE, BLACK);
}

void stopAllOutputs() {
  sendEspNow = false;
  for (size_t i = 0; i < toioCount; i++) {
    if (toioCore[i] != nullptr) {
      toioCore[i]->controlMotor(0, 0, 0, 0);
    }
  }
}

bool otaIsConfigured() {
  return strlen(WIFI_SSID) > 0;
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
        stopAllOutputs();
        USBSerial.println("OTA update started. Outputs stopped.");
        M5.Lcd.fillScreen(RED);
        M5.Lcd.setCursor(0, 10);
        M5.Lcd.print("OTA");
      })
      .onEnd([]() {
        USBSerial.println("\nOTA update finished.");
        M5.Lcd.setCursor(0, 50);
        M5.Lcd.print("Done");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        const unsigned int percent = total == 0 ? 0 : (progress * 100) / total;
        static unsigned int lastPercent = 101;
        if (percent != lastPercent) {
          lastPercent = percent;
          USBSerial.printf("OTA progress: %u%%\r", percent);
        }
      })
      .onError([](ota_error_t error) {
        stopAllOutputs();
        USBSerial.printf("OTA error[%u]\n", error);
        M5.Lcd.setCursor(0, 90);
        M5.Lcd.printf("Err %u", error);
      });

  ArduinoOTA.begin();
  otaStarted = true;
  USBSerial.printf("OTA ready: %s.local IP: %s\n", OTA_HOSTNAME,
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
    USBSerial.printf("Connecting OTA WiFi: %s\n", WIFI_SSID);
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
  stopAllOutputs();
  M5.Lcd.fillScreen(BLUE);
  M5.Lcd.setCursor(0, 10);
  M5.Lcd.print("OTA");

  if (!otaIsConfigured()) {
    USBSerial.println("OTA mode enabled, but WIFI_SSID is empty.");
    M5.Lcd.setCursor(0, 50);
    M5.Lcd.print("No WiFi");
    return;
  }

  lastOtaWifiAttemptMs = 0;
  serviceOTAMode();
}

void checkOtaButton() {
  if (!M5.Btn.isPressed()) {
    otaButtonHandled = false;
    return;
  }

  if (!otaButtonHandled && M5.Btn.pressedFor(kOtaButtonHoldMs)) {
    otaButtonHandled = true;
    enableOTAMode();
  }
}

void copyFloatToPacket(uint8_t offset, float value) {
  uint8_t *raw = reinterpret_cast<uint8_t *>(&value);
  for (uint8_t i = 0; i < sizeof(float); i++) {
    senddata[offset + i] = raw[i];
  }
}

bool parsePeerLine(const String &line, PeerEntry &peer) {
  String trimmed = line;
  trimmed.trim();
  if (trimmed.length() == 0) {
    return false;
  }

  unsigned int ch = 0;
  unsigned int m[6] = {};
  if (sscanf(trimmed.c_str(), "%u %02x:%02x:%02x:%02x:%02x:%02x", &ch,
             &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) != 7 &&
      sscanf(trimmed.c_str(), "%u,%02x,%02x,%02x,%02x,%02x,%02x", &ch,
             &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) != 7) {
    return false;
  }

  peer.channel = (uint8_t)ch;
  for (uint8_t i = 0; i < 6; i++) {
    peer.mac[i] = (uint8_t)m[i];
  }
  peer.valid = !isBroadcastMac(peer.mac);
  return peer.valid;
}

bool loadPeerFile(const char *path, PeerEntry &peer) {
  peer = PeerEntry();
  File file = SPIFFS.open(path, FILE_READ);
  if (!file) {
    return false;
  }

  while (file.available()) {
    PeerEntry parsed;
    if (parsePeerLine(file.readStringUntil('\n'), parsed)) {
      peer = parsed;
      file.close();
      return true;
    }
  }
  file.close();
  return false;
}

bool savePeerFile(const char *path, const PeerEntry &peer) {
  File file = SPIFFS.open(path, FILE_WRITE);
  if (!file) {
    return false;
  }

  if (peer.valid) {
    file.printf("%u %02X:%02X:%02X:%02X:%02X:%02X\n", peer.channel,
                peer.mac[0], peer.mac[1], peer.mac[2], peer.mac[3],
                peer.mac[4], peer.mac[5]);
  }
  file.close();
  return true;
}

void loadEspNowPeers() {
  loadPeerFile(kRoverCPeerFile, roverCPeer);
  loadPeerFile(kBugCPeerFile, bugCPeer);
  loadPeerFile(kMini4wdPeerFile, mini4wdPeer);
  loadPeerFile(kRoller485PeerFile, roller485Peer);
}

void savePairingPeer(TargetMode mode, const uint8_t mac[6], uint8_t channel) {
  PeerEntry *peer = peerForMode(mode);
  if (peer == nullptr) {
    return;
  }

  peer->valid = true;
  peer->channel = channel;
  memcpy(peer->mac, mac, 6);
  savePeerFile(peerFile(mode), *peer);
}

bool ensureEspNowPeer(const uint8_t mac[6], uint8_t channel) {
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = channel;
  peerInfo.encrypt = false;

  if (esp_now_is_peer_exist(mac)) {
    return esp_now_mod_peer(&peerInfo) == ESP_OK;
  }
  return esp_now_add_peer(&peerInfo) == ESP_OK;
}

void setEspNowChannel(uint8_t channel) {
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  ensureEspNowPeer(broadcastAddress, channel);
}

void buildEspNowPacket() {
  const uint8_t *targetMac = pairingMode ? broadcastAddress : activeTargetMac();
  senddata[0] = targetMac[3];
  senddata[1] = targetMac[4];
  senddata[2] = targetMac[5];

  if (isRoller485Mode(targetMode)) {
    copyFloatToPacket(3, normalizeStick(getRudder(), false) - 0.047f);
    copyFloatToPacket(7, normalizeStick(getThrottle(), true));
    copyFloatToPacket(11, normalizeStick(getAileron(), false));
    copyFloatToPacket(15, normalizeStick(getElevator(), false) + 0.012f);
  } else {
    copyFloatToPacket(3, joy.rudder);
    copyFloatToPacket(7, joy.throttle);
    copyFloatToPacket(11, joy.aileron);
    copyFloatToPacket(15, joy.elevator);
  }

  senddata[19] = getArmButton();
  senddata[20] = getFlipButton();
  senddata[21] = getModeButton();
  senddata[22] = getOptionButton();
  senddata[23] = 0;

  senddata[24] = 0;
  for (uint8_t i = 0; i < 24; i++) {
    senddata[24] += senddata[i];
  }
}

void sendEspNowPacket() {
  if (!sendEspNow) {
    return;
  }
  if (!savedPeerValid(targetMode)) {
    return;
  }

  buildEspNowPacket();
  const uint8_t *targetMac = activeTargetMac();
  const uint8_t channel = activeTargetChannel();
  ensureEspNowPeer(targetMac, channel);
  const esp_err_t result = esp_now_send(targetMac, senddata, sizeof(senddata));
  if (result == ESP_OK) {
    sentCount++;
  } else {
    sendFailCount++;
  }
}

bool ackChecksumOk(const uint8_t *data, int len) {
  if (len != kAckPacketSize) {
    return false;
  }

  uint8_t sum = 0;
  for (uint8_t i = 0; i < kAckPacketSize - 1; i++) {
    sum += data[i];
  }
  return sum == data[kAckPacketSize - 1];
}

void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (roller485PairingMode && len >= kRoller485PeerInfoSize &&
      data[7] == kRoller485PeerMagic0 && data[8] == kRoller485PeerMagic1 &&
      data[9] == kRoller485PeerMagic2 && data[10] == kRoller485PeerMagic3) {
    savePairingPeer(TargetMode::Roller485, &data[1], data[0]);
    memcpy(peerMac, &data[1], sizeof(peerMac));
    lastPeerAckMs = millis();
    peerReady = true;
    pairingReceived = true;
    return;
  }

  if (isRoller485Mode(targetMode)) {
    if (len >= (int)sizeof(float)) {
      memcpy(&roller485Voltage, data, sizeof(float));
      if (len >= (int)(sizeof(float) * 2)) {
        memcpy(&roller485ModeFlag, data + sizeof(float), sizeof(float));
      }
      memcpy(peerMac, mac, sizeof(peerMac));
      lastPeerAckMs = millis();
      peerReady = true;
    }
    return;
  }

  if (!ackChecksumOk(data, len)) {
    return;
  }
  if (data[0] != kAckMagic0 || data[1] != kAckMagic1 ||
      data[2] != kAckMagic2 || data[3] != kAckMagic3) {
    return;
  }

  peerReady = data[4] != 0;
  memcpy(peerMac, mac, sizeof(peerMac));
  lastPeerAckMs = millis();
  if (pairingMode && isAckPairingMode(pairingTargetMode) && peerReady) {
    savePairingPeer(pairingTargetMode, mac, pairingScanChannel);
    pairingReceived = true;
  }
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(activeTargetChannel(), WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    USBSerial.println("ESP-NOW init failed.");
    M5.Lcd.fillScreen(RED);
    M5.Lcd.setCursor(0, 10);
    M5.Lcd.print("ESPNOW NG");
    delay(2000);
    ESP.restart();
  }

  if (!ensureEspNowPeer(broadcastAddress, activeTargetChannel())) {
    USBSerial.println("Failed to add broadcast peer.");
  }
  if (savedPeerValid(targetMode)) {
    ensureEspNowPeer(activeTargetMac(), activeTargetChannel());
    memcpy(peerMac, activeTargetMac(), sizeof(peerMac));
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_get_version(&espnowVersion);
  espNowReady = true;
  USBSerial.printf("ESP-NOW ready ch=%d version=%lu\n", activeTargetChannel(),
                   espnowVersion);
}

bool toioPairReady() {
  return toioCount >= 2 && toioCore[0] != nullptr && toioCore[1] != nullptr;
}

size_t toioFrontIndex() {
  return toioSwap ? 0 : 1;
}

size_t toioRearIndex() {
  return toioSwap ? 1 : 0;
}

bool toioNameMatchesSuffix(size_t index, const char *suffix) {
  return index < toioCount && suffix != nullptr && suffix[0] != '\0' &&
         (toioName[index] == suffix || toioName[index].endsWith(suffix));
}

void updateStartupToioFront() {
  for (const char *suffix : kToioStartupFrontNameSuffixes) {
    for (size_t i = 0; i < toioCount; i++) {
      if (toioNameMatchesSuffix(i, suffix)) {
        toioSwap = (i == 0);
        USBSerial.printf("Startup front toio: %u %s\n", (unsigned)i,
                         toioName[i].c_str());
        return;
      }
    }
  }
}

void setupToio() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.print("scan toio");
  USBSerial.println("Scanning toio core cubes...");

  std::vector<ToioCore *> list = toio.scan(3);
  toioCount = min((size_t)kMaxToioCubes, list.size());

  for (size_t i = 0; i < toioCount; i++) {
    toioCore[i] = list.at(i);
    toioName[i] = toioCore[i]->getName().c_str();
    USBSerial.printf("%u: %s (%s)\n", (unsigned)i, toioName[i].c_str(),
                     toioCore[i]->getAddress().c_str());

    M5.Lcd.setCursor(0, 40 + i * 20);
    M5.Lcd.printf("%u %s", (unsigned)i, toioName[i].c_str());
    if (!toioCore[i]->connect()) {
      USBSerial.printf("%u: Failed BLE connection.\n", (unsigned)i);
      continue;
    }
    toioCore[i]->setConnectionInterval(8, 8);
    toioCore[i]->setIDnotificationSettings(1, 0x00);
    toioCore[i]->setIDmissedNotificationSettings(10);
    toioCore[i]->setPostureAngleDetectionSettings(
        1, 0x00, AngleTypeHighPrecisionEuller);
    toioCore[i]->onMotion(nullptr, nullptr, [i](ToioCorePostureAngle angle) {
      toioYaw[i] = angle.eulerf.yaw;
    });

    const uint j = i + 1;
    toioCore[i]->turnOnLed((j & 0x01) ? 64 : 0, (j & 0x02) ? 64 : 0,
                           (j & 0x04) ? 64 : 0);
  }

  updateStartupToioFront();

  M5.Lcd.setCursor(0, 90);
  M5.Lcd.printf("%u toio", (unsigned)toioCount);
  delay(500);
}

void serviceToio() {
  toio.loop();

  if (!toioPairReady()) {
    return;
  }

  float vx = joy.aileron / 1.7f;
  float vy = -joy.elevator / 2.0f;
  float omega = -joy.rudder / 4.0f;

  if (toioDriftMode) {
    const float angleRad = toioTargetYaw * PI / 180.0f;
    const float x1 = vx * cos(angleRad) - vy * sin(angleRad);
    const float y1 = vx * sin(angleRad) + vy * cos(angleRad);
    vx = x1;
    vy = y1;

    toioTargetYaw += joy.rudder * kToioDriftTargetYawStepDeg;
    while (toioTargetYaw >= 360.0f) toioTargetYaw -= 360.0f;
    while (toioTargetYaw < 0.0f) toioTargetYaw += 360.0f;
    toioDiff = normalizeAngle180(toioYaw[toioFrontIndex()] - toioStartYaw -
                                 toioTargetYaw);
    omega = constrain(toioDiff / kToioDriftYawFeedbackDivisorDeg, -1.0f, 1.0f);
  }

  float w1 = vy - vx - omega;
  float w2 = vy + vx + omega;
  float w3 = vy + vx - omega;
  float w4 = vy - vx + omega;

  const float maxW = max(max(fabsf(w1), fabsf(w2)), max(fabsf(w3), fabsf(w4)));
  if (maxW > 1.0f) {
    w1 /= maxW;
    w2 /= maxW;
    w3 /= maxW;
    w4 /= maxW;
  }

  const int m1 = (int)(w1 * 115);
  const int m2 = (int)(w2 * 115);
  const int m3 = (int)(w3 * 115);
  const int m4 = (int)(w4 * 115);

  const int leftIndex = toioFrontIndex();
  const int rightIndex = toioRearIndex();
  toioCore[leftIndex]->controlMotor(m2 < 0 ? 1 : 0, abs(m2),
                                    m1 < 0 ? 1 : 0, abs(m1));
  toioCore[rightIndex]->controlMotor(m3 < 0 ? 0 : 1, abs(m3),
                                     m4 < 0 ? 0 : 1, abs(m4));

  static bool optionButtonLatched = false;
  static bool throttleLowLatched = false;
  const bool optionButtonPressed = getOptionButton() != 0;
  if (optionButtonPressed && !optionButtonLatched) {
    optionButtonLatched = true;
    toioDriftMode = !toioDriftMode;
    toioStartYaw = toioYaw[toioFrontIndex()];
    toioTargetYaw = 0.0f;
    beep();
  } else if (!optionButtonPressed) {
    optionButtonLatched = false;
  }

  if (joy.throttle < -0.9f && !throttleLowLatched) {
    throttleLowLatched = true;
    toioSwap = !toioSwap;
    beep();
  } else if (joy.throttle >= -0.5f) {
    throttleLowLatched = false;
  }
}

void configureJoyMapping() {
  THROTTLE = LEFTY;
  AILERON = RIGHTX;
  ELEVATOR = RIGHTY;
  RUDDER = LEFTX;
  ARM_BUTTON = LEFT_STICK_BUTTON;
  FLIP_BUTTON = RIGHT_STICK_BUTTON;
  MODE_BUTTON = RIGHT_BUTTON;
  OPTION_BUTTON = LEFT_BUTTON;
}

uint8_t modeIndex(TargetMode mode, const TargetMode modes[], uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    if (modes[i] == mode) {
      return i;
    }
  }
  return 0;
}

void drawModeList(const char *title, const TargetMode modes[], uint8_t count,
                  uint8_t selected) {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.print(title);

  uint8_t first = 0;
  if (selected >= kMenuVisibleRows) {
    first = selected - kMenuVisibleRows + 1;
  }
  for (uint8_t row = 0; row < kMenuVisibleRows && first + row < count; row++) {
    const uint8_t index = first + row;
    M5.Lcd.setCursor(0, 22 + row * 22);
    M5.Lcd.printf("%c %s", index == selected ? '>' : ' ',
                  modeName(modes[index]));
  }
  M5.Lcd.setCursor(0, 110);
  M5.Lcd.print("OK:Btn");
}

TargetMode selectPairingTarget() {
  uint8_t selected = 0;
  lastMenuMoveMs = 0;
  drawModeList("Pair", kPairingModes, kPairingModeCount, selected);

  while (true) {
    M5.update();
    joy_update();

    const uint16_t x = getElevator();
    const uint32_t now = millis();
    if (now - lastMenuMoveMs >= kMenuMoveIntervalMs) {
      if (x < 1200) {
        selected = (selected + kPairingModeCount - 1) % kPairingModeCount;
        lastMenuMoveMs = now;
        drawModeList("Pair", kPairingModes, kPairingModeCount, selected);
      } else if (x > 2900) {
        selected = (selected + 1) % kPairingModeCount;
        lastMenuMoveMs = now;
        drawModeList("Pair", kPairingModes, kPairingModeCount, selected);
      }
    }

    if (M5.Btn.wasPressed() || getArmButton() || getFlipButton()) {
      beep();
      delay(250);
      return kPairingModes[selected];
    }
    delay(10);
  }
}

void enterPairingMode(TargetMode mode) {
  targetMode = mode;
  pairingTargetMode = mode;
  pairingMode = true;
  roller485PairingMode = isRoller485Mode(mode);
  pairingReceived = false;
  pairingScanChannel = 1;
  setupEspNow();
  setEspNowChannel(pairingScanChannel);
  sendEspNow = true;

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 5);
  M5.Lcd.printf("%s pair", modeName(mode));
  M5.Lcd.setCursor(0, 30);
  M5.Lcd.print("waiting...");

  uint32_t lastChannelScanMs = 0;
  while (!pairingReceived) {
    M5.update();
    joy_update();
    readJoy();

    const uint32_t now = millis();
    if (now - lastChannelScanMs >= kPairingChannelScanIntervalMs) {
      lastChannelScanMs = now;
      pairingScanChannel++;
      if (pairingScanChannel >= 15) {
        pairingScanChannel = 1;
      }
      setEspNowChannel(pairingScanChannel);
      M5.Lcd.setCursor(0, 55);
      M5.Lcd.printf("ch %02d   ", pairingScanChannel);
    }

    if (isAckPairingMode(mode) &&
        now - lastEspNowSendMs >= kPairingProbeIntervalMs) {
      lastEspNowSendMs = now;
      buildEspNowPacket();
      ensureEspNowPeer(broadcastAddress, pairingScanChannel);
      esp_now_send(broadcastAddress, senddata, sizeof(senddata));
    }
    delay(1);
  }

  M5.Lcd.fillScreen(GREEN);
  M5.Lcd.setCursor(0, 5);
  M5.Lcd.print("paired");
  M5.Lcd.setCursor(0, 30);
  M5.Lcd.printf("ch%d", pairingScanChannel);
  M5.Lcd.setCursor(0, 55);
  M5.Lcd.printf("%02X:%02X:%02X", peerMac[3], peerMac[4], peerMac[5]);
  beep();
  delay(1000);
  ESP.restart();
}

bool enterStartupPairingModeIfRequested() {
  if (!M5.Btn.isPressed()) {
    return false;
  }

  const uint32_t startMs = millis();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.print("Hold pair");
  while (M5.Btn.isPressed()) {
    M5.update();
    if (millis() - startMs >= kPairingButtonHoldMs) {
      configureJoyMapping();
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setCursor(0, 20);
      M5.Lcd.print("Release Btn");
      while (M5.Btn.isPressed()) {
        M5.update();
        delay(10);
      }
      delay(120);
      M5.update();

      enterPairingMode(selectPairingTarget());
      return true;
    }
    delay(10);
  }
  return false;
}

void drawMenu() {
  drawModeList("Select", kMainModes, kMainModeCount,
               modeIndex(menuMode, kMainModes, kMainModeCount));
}

TargetMode selectModeMenu() {
  configureJoyMapping();
  uint8_t selected = modeIndex(menuMode, kMainModes, kMainModeCount);

  drawMenu();
  while (true) {
    M5.update();
    joy_update();

    const uint16_t x = getElevator();
    const uint32_t now = millis();
    if (now - lastMenuMoveMs >= kMenuMoveIntervalMs) {
      if (x < 1200) {
        selected = (selected + kMainModeCount - 1) % kMainModeCount;
        menuMode = kMainModes[selected];
        lastMenuMoveMs = now;
        drawMenu();
      } else if (x > 2900) {
        selected = (selected + 1) % kMainModeCount;
        menuMode = kMainModes[selected];
        lastMenuMoveMs = now;
        drawMenu();
      }
    }

    if (M5.Btn.wasPressed() || getArmButton() || getFlipButton()) {
      beep();
      delay(250);
      return menuMode;
    }
    delay(10);
  }
}

void drawStatus() {
  if (isRoller485Mode(targetMode)) {
    const bool peerSeen = millis() - lastPeerAckMs <= kPeerStatusTimeoutMs;
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setCursor(8, 5);
    M5.Lcd.printf("V1 %4.1fV     ", roller485Voltage);
    M5.Lcd.setCursor(8, 21);
    M5.Lcd.printf("V2 %s      ", peerSeen && peerReady ? "OK" : "WAIT");
    M5.Lcd.setCursor(8, 37);
    M5.Lcd.printf("DT %02X:%02X:%02X   ", peerMac[3], peerMac[4],
                  peerMac[5]);
    M5.Lcd.setCursor(8, 53);
    M5.Lcd.printf("MD %s      ", roller485ModeFlag > 0.5f ? "angle" : "normal");
    return;
  }

  printJoyBatteryVoltage();
  M5.Lcd.setCursor(0, kDisplayTopY + 1 * kDisplayLineHeight);
  M5.Lcd.printf("%s       \n", modeName(targetMode));

  if (targetMode == TargetMode::Toio) {
    M5.Lcd.printf("toio %u/2   \n", (unsigned)toioCount);
    M5.Lcd.printf("%s swap%s \n", toioDriftMode ? "drift" : "normal",
                  toioSwap ? "A" : "B");
    M5.Lcd.printf("diff%5.1f \n", toioDiff);
  } else {
    const bool peerSeen = millis() - lastPeerAckMs <= kPeerStatusTimeoutMs;
    if (!savedPeerValid(targetMode)) {
      M5.Lcd.print("peer PAIR  \n");
    } else {
      M5.Lcd.printf("peer %s   \n", peerSeen && peerReady ? "OK" : "WAIT");
    }
    M5.Lcd.printf("esp %s c%02d\n", sendEspNow ? "send" : "stop",
                  activeTargetChannel());
    M5.Lcd.printf("%02X:%02X:%02X \n", peerMac[3], peerMac[4], peerMac[5]);
  }

  M5.Lcd.printf("x%+5.2f \n", joy.aileron);
  M5.Lcd.printf("y%+5.2f \n", joy.elevator);
  M5.Lcd.printf("z%+5.2f \n", joy.rudder);
}
}  // namespace

void setup() {
  M5.begin();
  Serial2.begin(115200, SERIAL_8N1, 2, 1);
  Wire1.begin(38, 39, 400 * 1000);
  M5.update();
  setup_pwm_buzzer();
  M5.Lcd.setRotation(2);
  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(4, 2);
  M5.Lcd.fillScreen(BLACK);
  SPIFFS.begin(true);
  loadEspNowPeers();

  joy_update();
  if (enterStartupPairingModeIfRequested()) {
    return;
  }
  targetMode = selectModeMenu();
  if (isEspNowMode() && !savedPeerValid(targetMode)) {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 20);
    M5.Lcd.printf("%s PAIR", modeName(targetMode));
    delay(700);
  }
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.printf("%s mode", modeName(targetMode));
  delay(400);

  if (targetMode == TargetMode::Toio) {
    setupToio();
  } else {
    setupEspNow();
    sendEspNow = true;
  }

  start_tone();
}

void loop() {
  M5.update();
  checkOtaButton();
  if (otaModeEnabled) {
    serviceOTAMode();
    delay(1);
    return;
  }

  readJoy();

  if (targetMode == TargetMode::Toio) {
    serviceToio();
  } else {
    if (!isRoller485Mode(targetMode) && joy.throttle > 0.9f) {
      sendEspNow = !sendEspNow;
      beep();
      delay(500);
    }

    const uint32_t now = millis();
    if (now - lastEspNowSendMs >= kEspNowSendIntervalMs) {
      lastEspNowSendMs = now;
      sendEspNowPacket();
    }
  }

  const uint32_t now = millis();
  if (now - lastDisplayMs >= kDisplayIntervalMs) {
    lastDisplayMs = now;
    drawStatus();
  }

  delay(1);
}

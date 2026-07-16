#include "M5_BugMecanum.h"

#ifndef BUGC_MOTOR0_SIGN
#define BUGC_MOTOR0_SIGN 1
#endif

#ifndef BUGC_MOTOR1_SIGN
#define BUGC_MOTOR1_SIGN 1
#endif

#ifndef BUGC_MOTOR2_SIGN
#define BUGC_MOTOR2_SIGN 1
#endif

#ifndef BUGC_MOTOR3_SIGN
#define BUGC_MOTOR3_SIGN 1
#endif

#ifndef BUGC_MOTOR_DEADBAND
#define BUGC_MOTOR_DEADBAND 8
#endif

namespace {
int8_t clampSpeed(int value) {
  return (int8_t)constrain(value, -100, 100);
}

int8_t applyDeadband(int value) {
  const int deadband = constrain(BUGC_MOTOR_DEADBAND, 0, 30);
  return abs(value) < deadband ? 0 : clampSpeed(value);
}

int8_t applyMotorSign(int8_t speed, int sign) {
  return applyDeadband(speed * (sign < 0 ? -1 : 1));
}
}  // namespace

bool M5_BugMecanum::begin(TwoWire *wire, uint8_t sda, uint8_t scl,
                          uint8_t addr, uint32_t freq) {
  _wire = wire;
  _addr = addr;
  _sda = sda;
  _scl = scl;

  _wire->begin((int)_sda, (int)_scl, freq);
  delay(10);

  _wire->beginTransmission(_addr);
  return _wire->endTransmission() == 0;
}

void M5_BugMecanum::writeBytes(uint8_t reg, const uint8_t *buffer,
                               uint8_t length) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  for (uint8_t i = 0; i < length; i++) {
    _wire->write(buffer[i]);
  }
  _wire->endTransmission();
}

bool M5_BugMecanum::readBytes(uint8_t reg, uint8_t *buffer, uint8_t length) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) {
    return false;
  }

  if (_wire->requestFrom(_addr, length) != length) {
    return false;
  }

  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = _wire->read();
  }
  return true;
}

void M5_BugMecanum::setMotor(uint8_t pos, int8_t speed) {
  const int8_t clamped = clampSpeed(speed);
  writeBytes(pos, (const uint8_t *)&clamped, 1);
}

void M5_BugMecanum::setAllMotors(int8_t motor0, int8_t motor1, int8_t motor2,
                                 int8_t motor3) {
  int8_t buffer[4] = {
      applyMotorSign(motor0, BUGC_MOTOR0_SIGN),
      applyMotorSign(motor1, BUGC_MOTOR1_SIGN),
      applyMotorSign(motor2, BUGC_MOTOR2_SIGN),
      applyMotorSign(motor3, BUGC_MOTOR3_SIGN),
  };
  writeBytes(0x00, (const uint8_t *)buffer, sizeof(buffer));
}

void M5_BugMecanum::setMecanumSpeed(int8_t x, int8_t y, int8_t z) {
  if (z != 0) {
    x = (int8_t)(x * (100 - abs(z)) / 100);
    y = (int8_t)(y * (100 - abs(z)) / 100);
  }

  const int motor0 = y + x - z;
  const int motor1 = y - x + z;
  const int motor2 = y - x - z;
  const int motor3 = y + x + z;

  setAllMotors(clampSpeed(motor0), clampSpeed(motor1), clampSpeed(motor2),
               clampSpeed(motor3));
}

void M5_BugMecanum::setColor(uint32_t left, uint32_t right) {
  uint8_t color[4] = {
      0,
      (uint8_t)((left & 0xff0000) >> 16),
      (uint8_t)((left & 0x00ff00) >> 8),
      (uint8_t)(left & 0x0000ff),
  };
  writeBytes(0x10, color, sizeof(color));

  color[0] = 1;
  color[1] = (uint8_t)((right & 0xff0000) >> 16);
  color[2] = (uint8_t)((right & 0x00ff00) >> 8);
  color[3] = (uint8_t)(right & 0x0000ff);
  writeBytes(0x10, color, sizeof(color));
}

bool M5_BugMecanum::getRawAdc12Bit(uint16_t &raw) {
  uint8_t buffer[2] = {};
  if (!readBytes(0x30, buffer, sizeof(buffer))) {
    return false;
  }

  raw = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
  return true;
}

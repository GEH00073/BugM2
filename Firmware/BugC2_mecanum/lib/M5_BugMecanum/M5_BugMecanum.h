#ifndef M5_BUG_MECANUM_H
#define M5_BUG_MECANUM_H

#include <Arduino.h>
#include <Wire.h>

#define BUGC_ADDRESS 0x38

class M5_BugMecanum {
 public:
  bool begin(TwoWire *wire = &Wire, uint8_t sda = 39, uint8_t scl = 38,
             uint8_t addr = BUGC_ADDRESS, uint32_t freq = 400000);
  void setMotor(uint8_t pos, int8_t speed);
  void setAllMotors(int8_t motor0, int8_t motor1, int8_t motor2,
                    int8_t motor3);
  void setMecanumSpeed(int8_t x, int8_t y, int8_t z);
  void setColor(uint32_t left, uint32_t right);
  bool getRawAdc12Bit(uint16_t &raw);

 private:
  void writeBytes(uint8_t reg, const uint8_t *buffer, uint8_t length);
  bool readBytes(uint8_t reg, uint8_t *buffer, uint8_t length);

  uint8_t _addr = BUGC_ADDRESS;
  TwoWire *_wire = &Wire;
  uint8_t _sda = 39;
  uint8_t _scl = 38;
};

#endif

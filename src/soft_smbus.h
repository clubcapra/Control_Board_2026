#ifndef PDU_SOFT_SMBUS_H_
#define PDU_SOFT_SMBUS_H_

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

#include <LM5066H1.h>

namespace pdu {
namespace soft_smbus {

class Master final : public LM5066H1Bus {
 public:
  Master(uint32_t sda_pin, uint32_t scl_pin);

  void begin() override;
  void setClock(uint32_t clockHz) override;
  void beginTransmission(uint8_t address) override;
  size_t write(uint8_t value) override;
  uint8_t endTransmission(bool sendStop = true) override;
  size_t requestFrom(uint8_t address, uint8_t quantity) override;
  int available() override;
  int read() override;

 private:
  static constexpr uint8_t kBufferSize = 40U;
  static constexpr uint32_t kClockStretchTimeoutUs = 25000UL;

  void sdaLow();
  void sdaRelease();
  void sclLow();
  bool sclReleaseAndWait();
  void delayHalfPeriod() const;

  bool startCondition();
  void stopCondition();
  bool writeByteOnBus(uint8_t value);
  uint8_t readByteFromBus(bool ack);
  void writeBit(bool high);
  bool readBit();

  uint32_t sda_pin_;
  uint32_t scl_pin_;
  uint32_t half_period_us_;
  uint8_t tx_address_;
  uint8_t tx_buffer_[kBufferSize];
  uint8_t tx_len_;
  uint8_t rx_buffer_[kBufferSize];
  uint8_t rx_len_;
  uint8_t rx_pos_;
};

}  /* namespace soft_smbus */
}  /* namespace pdu */

#endif /* PDU_SOFT_SMBUS_H_ */

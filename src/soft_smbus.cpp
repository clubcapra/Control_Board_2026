#include "soft_smbus.h"

namespace pdu {
namespace soft_smbus {

Master::Master(uint32_t sda_pin, uint32_t scl_pin)
    : sda_pin_(sda_pin),
      scl_pin_(scl_pin),
      half_period_us_(5U),
      tx_address_(0U),
      tx_buffer_{},
      tx_len_(0U),
      rx_buffer_{},
      rx_len_(0U),
      rx_pos_(0U) {}

void Master::begin() {
  sdaRelease();
  sclReleaseAndWait();
}

void Master::setClock(uint32_t clockHz) {
  if (clockHz == 0U) {
    clockHz = 100000UL;
  }
  const uint32_t half = 500000UL / clockHz;
  half_period_us_ = (half < 5UL) ? 5UL : half;
}

void Master::beginTransmission(uint8_t address) {
  tx_address_ = address;
  tx_len_ = 0U;
}

size_t Master::write(uint8_t value) {
  if (tx_len_ >= kBufferSize) {
    return 0U;
  }
  tx_buffer_[tx_len_] = value;
  ++tx_len_;
  return 1U;
}

uint8_t Master::endTransmission(bool sendStop) {
  if (!startCondition()) {
    stopCondition();
    return 4U;
  }
  if (!writeByteOnBus(static_cast<uint8_t>((tx_address_ << 1) | 0U))) {
    stopCondition();
    return 2U;
  }
  for (uint8_t i = 0U; i < tx_len_; ++i) {
    if (!writeByteOnBus(tx_buffer_[i])) {
      stopCondition();
      return 3U;
    }
  }
  if (sendStop) {
    stopCondition();
  }
  return 0U;
}

size_t Master::requestFrom(uint8_t address, uint8_t quantity) {
  if (quantity > kBufferSize) {
    quantity = kBufferSize;
  }

  rx_len_ = 0U;
  rx_pos_ = 0U;

  if (!startCondition()) {
    stopCondition();
    return 0U;
  }
  if (!writeByteOnBus(static_cast<uint8_t>((address << 1) | 1U))) {
    stopCondition();
    return 0U;
  }

  for (uint8_t i = 0U; i < quantity; ++i) {
    const bool ack = (i + 1U) < quantity;
    rx_buffer_[rx_len_] = readByteFromBus(ack);
    ++rx_len_;
  }
  stopCondition();
  return rx_len_;
}

int Master::available() {
  return static_cast<int>(rx_len_ - rx_pos_);
}

int Master::read() {
  if (rx_pos_ >= rx_len_) {
    return -1;
  }
  const uint8_t value = rx_buffer_[rx_pos_];
  ++rx_pos_;
  return static_cast<int>(value);
}

void Master::sdaLow() {
  digitalWrite(sda_pin_, LOW);
  pinMode(sda_pin_, OUTPUT);
}

void Master::sdaRelease() {
  pinMode(sda_pin_, INPUT_PULLUP);
}

void Master::sclLow() {
  digitalWrite(scl_pin_, LOW);
  pinMode(scl_pin_, OUTPUT);
}

bool Master::sclReleaseAndWait() {
  pinMode(scl_pin_, INPUT_PULLUP);
  const uint32_t start = micros();
  while (digitalRead(scl_pin_) == LOW) {
    if ((micros() - start) >= kClockStretchTimeoutUs) {
      return false;
    }
  }
  return true;
}

void Master::delayHalfPeriod() const {
  delayMicroseconds(half_period_us_);
}

bool Master::startCondition() {
  sdaRelease();
  if (!sclReleaseAndWait()) {
    return false;
  }
  delayHalfPeriod();
  sdaLow();
  delayHalfPeriod();
  sclLow();
  delayHalfPeriod();
  return true;
}

void Master::stopCondition() {
  sdaLow();
  delayHalfPeriod();
  (void)sclReleaseAndWait();
  delayHalfPeriod();
  sdaRelease();
  delayHalfPeriod();
}

bool Master::writeByteOnBus(uint8_t value) {
  for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U) {
    writeBit((value & mask) != 0U);
  }
  return !readBit();
}

uint8_t Master::readByteFromBus(bool ack) {
  uint8_t value = 0U;
  for (uint8_t i = 0U; i < 8U; ++i) {
    value <<= 1U;
    if (readBit()) {
      value |= 1U;
    }
  }
  writeBit(!ack);
  return value;
}

void Master::writeBit(bool high) {
  if (high) {
    sdaRelease();
  } else {
    sdaLow();
  }
  delayHalfPeriod();
  (void)sclReleaseAndWait();
  delayHalfPeriod();
  sclLow();
  delayHalfPeriod();
}

bool Master::readBit() {
  sdaRelease();
  delayHalfPeriod();
  (void)sclReleaseAndWait();
  delayHalfPeriod();
  const bool high = (digitalRead(sda_pin_) == HIGH);
  sclLow();
  delayHalfPeriod();
  return high;
}

}  /* namespace soft_smbus */
}  /* namespace pdu */

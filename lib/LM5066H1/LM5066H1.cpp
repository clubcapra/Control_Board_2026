#include "LM5066H1.h"

#include <math.h>

namespace {
constexpr uint8_t kBlockWidth = 0xFFU;

const LM5066H1::RegisterDescriptor kLm5066H1Registers[] = {
    {0x01U, "OPERATION", LM5066H1::RegisterAccess::kReadWrite, 1U},
    {0x03U, "CLEAR_FAULTS", LM5066H1::RegisterAccess::kSendByte, 0U},
    {0x10U, "WRITE_PROTECT", LM5066H1::RegisterAccess::kReadWrite, 1U},
    {0x12U, "RESTORE_FACTORY_DEFAULTS", LM5066H1::RegisterAccess::kSendByte, 0U},
    {0x15U, "STORE_USER_ALL", LM5066H1::RegisterAccess::kSendByte, 0U},
    {0x16U, "RESTORE_USER_ALL", LM5066H1::RegisterAccess::kSendByte, 0U},
    {0x19U, "CAPABILITY", LM5066H1::RegisterAccess::kReadOnly, 1U},
    {0x43U, "VOUT_UV_WARN_LIMIT", LM5066H1::RegisterAccess::kReadWrite, 2U},
    {0x4FU, "OT_FAULT_LIMIT", LM5066H1::RegisterAccess::kReadWrite, 2U},
    {0x51U, "OT_WARN_LIMIT", LM5066H1::RegisterAccess::kReadWrite, 2U},
    {0x57U, "VIN_OV_WARN_LIMIT", LM5066H1::RegisterAccess::kReadWrite, 2U},
    {0x58U, "VIN_UV_WARN_LIMIT", LM5066H1::RegisterAccess::kReadWrite, 2U},
    {0x5DU, "IIN_OC_WARN_LIMIT", LM5066H1::RegisterAccess::kReadWrite, 2U},
    {0x78U, "STATUS_BYTE", LM5066H1::RegisterAccess::kReadOnly, 1U},
    {0x79U, "STATUS_WORD", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0x7AU, "STATUS_VOUT", LM5066H1::RegisterAccess::kReadOnly, 1U},
    {0x7CU, "STATUS_INPUT", LM5066H1::RegisterAccess::kReadOnly, 1U},
    {0x7DU, "STATUS_TEMPERATURE", LM5066H1::RegisterAccess::kReadOnly, 1U},
    {0x7EU, "STATUS_CML", LM5066H1::RegisterAccess::kReadOnly, 1U},
    {0x7FU, "STATUS_OTHER", LM5066H1::RegisterAccess::kReadOnly, 1U},
    {0x80U, "STATUS_MFR_SPECIFIC", LM5066H1::RegisterAccess::kReadOnly, 1U},
    {0x86U, "READ_EIN", LM5066H1::RegisterAccess::kBlockRead, kBlockWidth},
    {0x88U, "READ_VIN", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0x89U, "READ_IIN", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0x8BU, "READ_VOUT", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0x8CU, "READ_IOUT", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0x8DU, "READ_TEMPERATURE_1", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0x96U, "READ_POUT", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0x97U, "READ_PIN", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0x98U, "PMBUS_REVISION", LM5066H1::RegisterAccess::kReadOnly, 1U},
    {0x99U, "MFR_ID", LM5066H1::RegisterAccess::kBlockRead, kBlockWidth},
    {0x9AU, "MFR_MODEL", LM5066H1::RegisterAccess::kBlockRead, kBlockWidth},
    {0x9BU, "MFR_REVISION", LM5066H1::RegisterAccess::kBlockRead, kBlockWidth},
    {0xA0U, "MFR_READ_VIN_MIN", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0xA1U, "MFR_READ_VIN_PEAK", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0xA2U, "MFR_READ_IIN_PEAK", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0xA3U, "MFR_READ_PIN_PEAK", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0xA4U, "MFR_READ_VOUT_MIN", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0xBCU, "MFR_USER_DATA", LM5066H1::RegisterAccess::kReadWrite, 1U},
    {0xC7U, "MFR_READ_AVG_TEMPERATURE", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0xC8U, "MFR_READ_PEAK_TEMPERATURE", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0xC9U, "MFR_READ_SAMPLE_BUFFER", LM5066H1::RegisterAccess::kBlockRead, kBlockWidth},
    {0xCAU, "MFR_POWER_CYCLE", LM5066H1::RegisterAccess::kSendByte, 0U},
    {0xCCU, "DEVICE_SETUP", LM5066H1::RegisterAccess::kReadWrite, 1U},
    {0xCDU, "DEVICE_SETUP_4", LM5066H1::RegisterAccess::kReadWrite, 1U},
    {0xCEU, "DEVICE_SETUP_5", LM5066H1::RegisterAccess::kReadWrite, 1U},
    {0xD0U, "MFR_READ_VAUX", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0xD1U, "MFR_READ_IIN", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0xD2U, "MFR_READ_PIN", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0xD3U, "MFR_IIN_OC_WARN_LIMIT", LM5066H1::RegisterAccess::kReadWrite, 2U},
    {0xD4U, "MFR_PIN_OP_WARN_LIMIT", LM5066H1::RegisterAccess::kReadWrite, 2U},
    {0xD5U, "MFR_READ_PIN_PEAK", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0xD6U, "MFR_CLEAR_PIN_PEAK", LM5066H1::RegisterAccess::kSendByte, 0U},
    {0xD7U, "MFR_GATE_MASK", LM5066H1::RegisterAccess::kReadWrite, 1U},
    {0xD8U, "MFR_ALERT_MASK", LM5066H1::RegisterAccess::kReadWrite, 2U},
    {0xD9U, "MFR_READ_AVG_VAUX", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0xDAU, "MFR_BLOCK_READ", LM5066H1::RegisterAccess::kBlockRead, kBlockWidth},
    {0xDBU, "SAMPLES_FOR_AVG", LM5066H1::RegisterAccess::kReadWrite, 1U},
    {0xDCU, "MFR_READ_AVG_VIN", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0xDDU, "MFR_READ_AVG_VOUT", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0xDEU, "MFR_READ_AVG_IIN", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0xDFU, "MFR_READ_AVG_PIN", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0xE0U, "BLACK_BOX_CLEAR", LM5066H1::RegisterAccess::kSendByte, 0U},
    {0xE1U, "DIAGNOSTIC_WORD", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0xE2U, "AVG_BLOCK_READ", LM5066H1::RegisterAccess::kBlockRead, kBlockWidth},
    {0xE3U, "BLACK_BOX_ERASE", LM5066H1::RegisterAccess::kSendByte, 0U},
    {0xE4U, "BLACK_BOX_CONFIG", LM5066H1::RegisterAccess::kReadWrite, 1U},
    {0xE5U, "OC_BLANKING_TIMERS", LM5066H1::RegisterAccess::kReadWrite, 1U},
    {0xE7U, "RETRY_INSERTION_DELAY", LM5066H1::RegisterAccess::kReadWrite, 1U},
    {0xE8U, "WATCHDOG_PL_TIMER", LM5066H1::RegisterAccess::kReadWrite, 1U},
    {0xE9U, "PK_MIN_AVG", LM5066H1::RegisterAccess::kReadWrite, 1U},
    {0xEAU, "P2T_TIMER", LM5066H1::RegisterAccess::kReadWrite, 1U},
    {0xEBU, "FETCH_BLACK_BOX_EEPROM", LM5066H1::RegisterAccess::kSendByte, 0U},
    {0xECU, "READ_BLACK_BOX_RAM", LM5066H1::RegisterAccess::kBlockRead, kBlockWidth},
    {0xEDU, "ADC_CONFIG_1", LM5066H1::RegisterAccess::kReadWrite, 1U},
    {0xEEU, "ADC_CONFIG_2", LM5066H1::RegisterAccess::kReadWrite, 1U},
    {0xEFU, "DEVICE_SETUP_2", LM5066H1::RegisterAccess::kReadWrite, 1U},
    {0xF0U, "DEVICE_SETUP_3", LM5066H1::RegisterAccess::kReadWrite, 1U},
    {0xF2U, "IIN_OFFSET_CALIBRATION", LM5066H1::RegisterAccess::kReadWrite, 1U},
    {0xF3U, "STATUS_MFR_SPECIFIC_2", LM5066H1::RegisterAccess::kReadOnly, 2U},
    {0xF4U, "READ_BLACK_BOX_EEPROM", LM5066H1::RegisterAccess::kBlockRead, kBlockWidth},
    {0xF6U, "BLACK_BOX_TIMER", LM5066H1::RegisterAccess::kReadOnly, 1U},
    {0xF7U, "PMBUS_ADDRESS", LM5066H1::RegisterAccess::kReadWrite, 1U},
    {0xF8U, "OC_WARN_LIMIT", LM5066H1::RegisterAccess::kReadWrite, 2U},
};
}  // namespace

LM5066H1::LM5066H1(uint8_t address, TwoWire& wire)
    : _address(address),
      _wireBus(wire),
      _bus(&_wireBus),
      _senseResistorMilliOhms(1.0),
      _currentLimitSetting(CurrentLimitSetting::kHigh50mV),
      _currentLimitMilliVolts(50.0),
      _adcFullScale2x(false),
      _ntcPullupOhms(10000.0),
      _ntcNominalOhms(10000.0),
      _ntcNominalTempC(25.0),
      _ntcBeta(3950.0),
      _ntcSupplyVolts(3.3),
      _pecEnabled(false) {}

LM5066H1::LM5066H1(uint8_t address, LM5066H1Bus& bus)
    : _address(address),
      _wireBus(Wire),
      _bus(&bus),
      _senseResistorMilliOhms(1.0),
      _currentLimitSetting(CurrentLimitSetting::kHigh50mV),
      _currentLimitMilliVolts(50.0),
      _adcFullScale2x(false),
      _ntcPullupOhms(10000.0),
      _ntcNominalOhms(10000.0),
      _ntcNominalTempC(25.0),
      _ntcBeta(3950.0),
      _ntcSupplyVolts(3.3),
      _pecEnabled(false) {}

bool LM5066H1::begin(int sdaPin, int sclPin, uint32_t clockHz) {
#if defined(ARDUINO_ARCH_STM32) || defined(STM32F1xx) || defined(STM32F1)
  if (_bus == &_wireBus) {
    if (sdaPin >= 0) {
      _wireBus.setSDA(static_cast<uint32_t>(sdaPin));
    }
    if (sclPin >= 0) {
      _wireBus.setSCL(static_cast<uint32_t>(sclPin));
    }
  }
  _bus->begin();
#else
  (void)sdaPin;
  (void)sclPin;
  _bus->begin();
#endif
  _bus->setClock(clockHz);
  return isPresent();
}

bool LM5066H1::beginAttached(uint32_t clockHz) {
  _bus->setClock(clockHz);
  return isPresent();
}

bool LM5066H1::isPresent() {
  _bus->beginTransmission(_address);
  return _bus->endTransmission() == 0;
}

void LM5066H1::recoverBus() { _bus->recover(); }

void LM5066H1::setAddress(uint8_t address) {
  _address = address;
}

uint8_t LM5066H1::address() const {
  return _address;
}

void LM5066H1::setSenseResistorMilliOhms(double rs_mohm) {
  if (rs_mohm > 0.0) {
    _senseResistorMilliOhms = rs_mohm;
  }
}

double LM5066H1::senseResistorMilliOhms() const {
  return _senseResistorMilliOhms;
}

void LM5066H1::setCurrentLimitSetting(CurrentLimitSetting setting) {
  _currentLimitSetting = setting;
  if (setting == CurrentLimitSetting::kLow26mV) {
    _currentLimitMilliVolts = 25.0;
  } else {
    _currentLimitMilliVolts = 50.0;
  }
}

LM5066H1::CurrentLimitSetting LM5066H1::currentLimitSetting() const {
  return _currentLimitSetting;
}

void LM5066H1::setCurrentLimitMilliVolts(double millivolts) {
  if (millivolts > 0.0) {
    _currentLimitMilliVolts = millivolts;
  }
}

double LM5066H1::currentLimitMilliVolts() const {
  return _currentLimitMilliVolts;
}

void LM5066H1::setAdcFullScale2x(bool enable) {
  _adcFullScale2x = enable;
}

bool LM5066H1::adcFullScale2x() const {
  return _adcFullScale2x;
}

void LM5066H1::setNtcConfig(const NtcConfig& config) {
  if (config.pullupOhms > 0.0) {
    _ntcPullupOhms = config.pullupOhms;
  }
  if (config.nominalOhms > 0.0) {
    _ntcNominalOhms = config.nominalOhms;
  }
  _ntcNominalTempC = config.nominalTempC;
  if (config.beta > 0.0) {
    _ntcBeta = config.beta;
  }
  if (config.supplyVolts > 0.0) {
    _ntcSupplyVolts = config.supplyVolts;
  }
}

LM5066H1::NtcConfig LM5066H1::ntcConfig() const {
  NtcConfig config;
  config.pullupOhms = _ntcPullupOhms;
  config.nominalOhms = _ntcNominalOhms;
  config.nominalTempC = _ntcNominalTempC;
  config.beta = _ntcBeta;
  config.supplyVolts = _ntcSupplyVolts;
  return config;
}

bool LM5066H1::writeByte(uint8_t reg, uint8_t value) {
  _bus->beginTransmission(_address);
  _bus->write(reg);
  _bus->write(value);
  return _bus->endTransmission() == 0;
}

bool LM5066H1::writeWord(uint8_t reg, uint16_t value) {
  _bus->beginTransmission(_address);
  _bus->write(reg);
  _bus->write(static_cast<uint8_t>(value & 0xFF));
  _bus->write(static_cast<uint8_t>((value >> 8) & 0xFF));
  return _bus->endTransmission() == 0;
}

bool LM5066H1::writeBlock(uint8_t reg, const uint8_t* data, size_t len) {
  _bus->beginTransmission(_address);
  _bus->write(reg);
  for (size_t i = 0; i < len; ++i) {
    _bus->write(data[i]);
  }
  return _bus->endTransmission() == 0;
}

bool LM5066H1::writeBlockCommand(uint8_t cmd, const uint8_t* data,
                                 size_t len) {
  if ((data == nullptr && len != 0U) || len > 32U) {
    return false;
  }

  _bus->beginTransmission(_address);
  _bus->write(cmd);
  _bus->write(static_cast<uint8_t>(len));
  for (size_t i = 0; i < len; ++i) {
    _bus->write(data[i]);
  }
  return _bus->endTransmission() == 0;
}

uint8_t LM5066H1::pecCrc8(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0U; i < 8U; ++i) {
    if ((crc & 0x80U) != 0U) {
      crc = static_cast<uint8_t>((crc << 1) ^ 0x07U);
    } else {
      crc = static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

void LM5066H1::setPecEnabled(bool enable) { _pecEnabled = enable; }
bool LM5066H1::pecEnabled() const { return _pecEnabled; }

bool LM5066H1::readByte(uint8_t reg, uint8_t& value) {
  _bus->beginTransmission(_address);
  _bus->write(reg);
  if (_bus->endTransmission(false) != 0) {
    return false;
  }
  const uint8_t want = _pecEnabled ? 2U : 1U;
  if (_bus->requestFrom(_address, want) != want) {
    return false;
  }
  const uint8_t data = static_cast<uint8_t>(_bus->read());
  if (_pecEnabled) {
    const uint8_t pec = static_cast<uint8_t>(_bus->read());
    /* PEC spans the whole Read-Byte transaction in wire order:
     * [addr|W], command, [addr|R], data. */
    uint8_t crc = 0U;
    crc = pecCrc8(crc, static_cast<uint8_t>(_address << 1));
    crc = pecCrc8(crc, reg);
    crc = pecCrc8(crc, static_cast<uint8_t>((_address << 1) | 1U));
    crc = pecCrc8(crc, data);
    if (crc != pec) {
      return false;
    }
  }
  value = data;
  return true;
}

bool LM5066H1::readWord(uint8_t reg, uint16_t& value) {
  _bus->beginTransmission(_address);
  _bus->write(reg);
  if (_bus->endTransmission(false) != 0) {
    return false;
  }
  const uint8_t want = _pecEnabled ? 3U : 2U;
  if (_bus->requestFrom(_address, want) != want) {
    return false;
  }
  const uint8_t low  = static_cast<uint8_t>(_bus->read());
  const uint8_t high = static_cast<uint8_t>(_bus->read());
  if (_pecEnabled) {
    const uint8_t pec = static_cast<uint8_t>(_bus->read());
    /* PEC spans the whole Read-Word transaction in wire order:
     * [addr|W], command, [addr|R], data_low, data_high. */
    uint8_t crc = 0U;
    crc = pecCrc8(crc, static_cast<uint8_t>(_address << 1));
    crc = pecCrc8(crc, reg);
    crc = pecCrc8(crc, static_cast<uint8_t>((_address << 1) | 1U));
    crc = pecCrc8(crc, low);
    crc = pecCrc8(crc, high);
    if (crc != pec) {
      return false;
    }
  }
  value = static_cast<uint16_t>(high << 8) | low;
  return true;
}

bool LM5066H1::readBlock(uint8_t reg, uint8_t* data, size_t len) {
  _bus->beginTransmission(_address);
  _bus->write(reg);
  if (_bus->endTransmission(false) != 0) {
    return false;
  }
  if (_bus->requestFrom(_address, static_cast<uint8_t>(len)) != len) {
    return false;
  }
  for (size_t i = 0; i < len; ++i) {
    data[i] = _bus->read();
  }
  return true;
}

bool LM5066H1::readBlockCommand(uint8_t cmd, uint8_t* data, size_t maxLen,
                                size_t& outLen) {
  outLen = 0;
  if (data == nullptr || maxLen == 0) {
    return false;
  }

  _bus->beginTransmission(_address);
  _bus->write(cmd);
  if (_bus->endTransmission(false) != 0) {
    return false;
  }
  size_t request = maxLen + 1;
  size_t received =
      _bus->requestFrom(_address, static_cast<uint8_t>(request));
  if (received < 1) {
    return false;
  }
  uint8_t count = _bus->read();
  bool tooLong = count > maxLen;
  size_t toCopy = tooLong ? maxLen : count;
  for (size_t i = 0; i < toCopy && _bus->available(); ++i) {
    data[i] = _bus->read();
    outLen++;
  }
  while (_bus->available()) {
    _bus->read();
  }
  return !tooLong && outLen == count;
}

bool LM5066H1::sendCommand(uint8_t cmd) {
  return sendByte(cmd);
}

bool LM5066H1::readRegisterRaw(uint8_t cmd, uint8_t* data, size_t maxLen,
                               size_t& outLen) {
  outLen = 0U;
  if (data == nullptr || maxLen == 0U) {
    return false;
  }

  const RegisterDescriptor* desc = findRegisterDescriptor(cmd);
  if (desc != nullptr) {
    if (desc->width == 1U) {
      uint8_t raw = 0U;
      if (!readByte(cmd, raw) || maxLen < 1U) {
        return false;
      }
      data[0] = raw;
      outLen = 1U;
      return true;
    }
    if (desc->width == 2U) {
      uint16_t raw = 0U;
      if (!readWord(cmd, raw) || maxLen < 2U) {
        return false;
      }
      data[0] = static_cast<uint8_t>(raw & 0xFFU);
      data[1] = static_cast<uint8_t>((raw >> 8U) & 0xFFU);
      outLen = 2U;
      return true;
    }
    if (desc->width == kBlockWidth) {
      return readBlockCommand(cmd, data, maxLen, outLen);
    }
    return false;
  }

  return bridgeRead(cmd, data, maxLen, outLen);
}

bool LM5066H1::writeRegisterRaw(uint8_t cmd, const uint8_t* data, size_t len) {
  const RegisterDescriptor* desc = findRegisterDescriptor(cmd);
  if (desc != nullptr) {
    switch (desc->access) {
      case RegisterAccess::kReadOnly:
      case RegisterAccess::kBlockRead:
        return false;
      case RegisterAccess::kSendByte:
        return len == 0U && sendByte(cmd);
      case RegisterAccess::kWriteOnly:
      case RegisterAccess::kReadWrite:
        if (desc->width == 1U) {
          return data != nullptr && len == 1U && writeByte(cmd, data[0]);
        }
        if (desc->width == 2U) {
          if (data == nullptr || len != 2U) {
            return false;
          }
          const uint16_t raw =
              static_cast<uint16_t>(data[0]) |
              static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
          return writeWord(cmd, raw);
        }
        return false;
      case RegisterAccess::kBlockReadWrite:
        return writeBlockCommand(cmd, data, len);
      default:
        return false;
    }
  }

  return bridgeWrite(cmd, data, len);
}

bool LM5066H1::writeOperationRaw(uint8_t value) {
  return writeByte(kCmdOperation, value);
}

bool LM5066H1::readOperationRaw(uint8_t& value) {
  return readByte(kCmdOperation, value);
}

bool LM5066H1::readOperation(bool& on) {
  uint8_t value = 0;
  if (!readOperationRaw(value)) {
    return false;
  }
  on = (value & kOperationOn) != 0;
  return true;
}

bool LM5066H1::setOperation(bool on) {
  return writeOperationRaw(on ? kOperationOn : kOperationOff);
}

bool LM5066H1::clearFaults() {
  return sendByte(kCmdClearFaults);
}

bool LM5066H1::setWriteProtect(uint8_t value) {
  return writeByte(kCmdWriteProtect, value);
}

bool LM5066H1::readWriteProtect(uint8_t& value) {
  return readByte(kCmdWriteProtect, value);
}

bool LM5066H1::restoreFactoryDefaults() {
  return sendByte(kCmdRestoreFactoryDefaults);
}

bool LM5066H1::storeUserAll() {
  return sendByte(kCmdStoreUserAll);
}

bool LM5066H1::restoreUserAll() {
  return sendByte(kCmdRestoreUserAll);
}

bool LM5066H1::powerCycle() {
  return sendByte(kCmdPowerCycle);
}

bool LM5066H1::readCapability(uint8_t& value) {
  return readByte(kCmdCapability, value);
}

bool LM5066H1::readPmbusRevision(uint8_t& value) {
  return readByte(kCmdPmbusRevision, value);
}

bool LM5066H1::readStatusByte(uint8_t& value) {
  return readByte(kCmdStatusByte, value);
}

bool LM5066H1::readStatusByte(StatusByteBits& status) {
  uint8_t raw = 0;
  if (!readStatusByte(raw)) {
    return false;
  }
  status.off = (raw >> 6) & 0x1;
  status.vinUvFault = (raw >> 3) & 0x1;
  status.temperature = (raw >> 2) & 0x1;
  status.cml = (raw >> 1) & 0x1;
  status.other = (raw & 0x1) != 0;
  return true;
}

bool LM5066H1::readStatusWord(uint16_t& value) {
  return readWord(kCmdStatusWord, value);
}

bool LM5066H1::readStatusWord(StatusWordBits& status) {
  uint16_t raw = 0;
  if (!readStatusWord(raw)) {
    return false;
  }
  status.outStatus = (raw >> 15) & 0x1;
  status.inputStatus = (raw >> 13) & 0x1;
  status.fetFail = (raw >> 12) & 0x1;
  status.pgoodb = (raw >> 11) & 0x1;
  status.cbFault = (raw >> 9) & 0x1;
  status.unknown = (raw >> 8) & 0x1;

  const uint8_t lowByte = static_cast<uint8_t>(raw & 0xFF);
  status.byte.off = (lowByte >> 6) & 0x1;
  status.byte.vinUvFault = (lowByte >> 3) & 0x1;
  status.byte.temperature = (lowByte >> 2) & 0x1;
  status.byte.cml = (lowByte >> 1) & 0x1;
  status.byte.other = (lowByte & 0x1) != 0;
  return true;
}

bool LM5066H1::readStatusVout(uint8_t& value) {
  return readByte(kCmdStatusVout, value);
}

bool LM5066H1::readStatusVout(StatusVoutBits& status) {
  uint8_t raw = 0;
  if (!readStatusVout(raw)) {
    return false;
  }
  status.voutUvWarn = (raw >> 5) & 0x1;
  return true;
}

bool LM5066H1::readStatusInput(uint8_t& value) {
  return readByte(kCmdStatusInput, value);
}

bool LM5066H1::readStatusInput(StatusInputBits& status) {
  uint8_t raw = 0;
  if (!readStatusInput(raw)) {
    return false;
  }
  status.vinOvFault = (raw >> 7) & 0x1;
  status.vinOvWarn = (raw >> 6) & 0x1;
  status.vinUvWarn = (raw >> 5) & 0x1;
  status.vinUvFault = (raw >> 4) & 0x1;
  status.ocFault = (raw >> 2) & 0x1;
  status.ocWarn = (raw >> 1) & 0x1;
  status.inOpWarn = (raw & 0x1) != 0;
  return true;
}

bool LM5066H1::readStatusTemperature(uint8_t& value) {
  return readByte(kCmdStatusTemperature, value);
}

bool LM5066H1::readStatusTemperature(StatusTemperatureBits& status) {
  uint8_t raw = 0;
  if (!readStatusTemperature(raw)) {
    return false;
  }
  status.otFault = (raw >> 7) & 0x1;
  status.otWarn = (raw >> 6) & 0x1;
  return true;
}

bool LM5066H1::readStatusCml(uint8_t& value) {
  return readByte(kCmdStatusCml, value);
}

bool LM5066H1::readStatusCml(StatusCmlBits& status) {
  uint8_t raw = 0;
  if (!readStatusCml(raw)) {
    return false;
  }
  status.invCmd = (raw >> 7) & 0x1;
  status.invData = (raw >> 6) & 0x1;
  status.invPec = (raw >> 5) & 0x1;
  status.memoryFault = (raw >> 4) & 0x1;
  status.noneOfAbove = (raw >> 1) & 0x1;
  return true;
}

bool LM5066H1::readStatusOther(uint8_t& value) {
  return readByte(kCmdStatusOther, value);
}

bool LM5066H1::readStatusOther(StatusOtherBits& status) {
  uint8_t raw = 0;
  if (!readStatusOther(raw)) {
    return false;
  }
  status.cbFault = (raw >> 5) & 0x1;
  status.firstSmba = (raw & 0x1) != 0;
  return true;
}

bool LM5066H1::readStatusMfrSpecific(uint8_t& value) {
  return readByte(kCmdStatusMfrSpecific, value);
}

bool LM5066H1::readStatusMfrSpecific(StatusMfrSpecificBits& status) {
  uint8_t raw = 0;
  if (!readStatusMfrSpecific(raw)) {
    return false;
  }
  status.cbFault = (raw >> 7) & 0x1;
  status.fetFail = (raw >> 6) & 0x1;
  status.defaultsLoaded = (raw >> 4) & 0x1;
  status.bbRamFull = (raw >> 3) & 0x1;
  status.fetFaultGate2 = (raw >> 2) & 0x1;
  status.fetFaultGate1 = (raw >> 1) & 0x1;
  status.fetFaultDrain = (raw & 0x1) != 0;
  return true;
}

bool LM5066H1::readStatusMfrSpecific2(uint16_t& value) {
  return readWord(kCmdStatusMfrSpecific2, value);
}

bool LM5066H1::readStatusMfrSpecific2(StatusMfrSpecific2Bits& status) {
  uint16_t raw = 0;
  if (!readStatusMfrSpecific2(raw)) {
    return false;
  }
  status.watchdogFault = (raw >> 12) & 0x1;
  status.scFault = (raw >> 11) & 0x1;
  status.einOfWarn = (raw >> 9) & 0x1;
  status.vinTran = (raw >> 8) & 0x1;
  status.eeProg = (raw >> 6) & 0x1;
  status.avgDone = (raw >> 5) & 0x1;
  status.retryRec = (raw >> 3) & 0x1;
  status.powerCycleRec = (raw >> 2) & 0x1;
  status.initDone = (raw >> 1) & 0x1;
  return true;
}

bool LM5066H1::readDiagnosticWord(uint16_t& value) {
  return readWord(kCmdDiagnosticWord, value);
}

bool LM5066H1::readDiagnosticWord(DiagnosticWordBits& status) {
  uint16_t raw = 0;
  if (!readDiagnosticWord(raw)) {
    return false;
  }
  status.voutUndervoltageWarn = (raw >> 15) & 0x1;
  status.iinOpWarn = (raw >> 14) & 0x1;
  status.vinUndervoltageWarn = (raw >> 13) & 0x1;
  status.vinOvervoltageWarn = (raw >> 12) & 0x1;
  status.powerGood = (raw >> 11) & 0x1;
  status.overTemperatureWarn = (raw >> 10) & 0x1;
  status.timerLatchedOff = (raw >> 9) & 0x1;
  status.fetFail = (raw >> 8) & 0x1;
  status.configPreset = (raw >> 7) & 0x1;
  status.deviceOff = (raw >> 6) & 0x1;
  status.vinUndervoltageFault = (raw >> 5) & 0x1;
  status.vinOvervoltageFault = (raw >> 4) & 0x1;
  status.iinOcPfetOpFault = (raw >> 3) & 0x1;
  status.overTemperatureFault = (raw >> 2) & 0x1;
  status.cmlFault = (raw >> 1) & 0x1;
  status.circuitBreakerFault = (raw & 0x1) != 0;
  return true;
}

bool LM5066H1::readEinRaw(uint8_t* data, size_t len, size_t& outLen) {
  return readBlockCommand(kCmdReadEin, data, len, outLen);
}

bool LM5066H1::readEin(EinData& data) {
  uint8_t buffer[6] = {};
  size_t outLen = 0;
  if (!readEinRaw(buffer, sizeof(buffer), outLen)) {
    return false;
  }
  if (outLen < sizeof(buffer)) {
    return false;
  }
  data.accumulator = static_cast<int16_t>(
      static_cast<uint16_t>(buffer[1] << 8) | buffer[0]);
  data.rolloverCount = buffer[2];
  data.sampleCount =
      static_cast<uint32_t>(buffer[3]) |
      (static_cast<uint32_t>(buffer[4]) << 8) |
      (static_cast<uint32_t>(buffer[5]) << 16);
  return true;
}

bool LM5066H1::readVinRaw(uint16_t& value) {
  return readWord(kCmdReadVin, value);
}

bool LM5066H1::readVoutRaw(uint16_t& value) {
  return readWord(kCmdReadVout, value);
}

bool LM5066H1::readVauxRaw(uint16_t& value) {
  return readWord(kCmdReadVaux, value);
}

bool LM5066H1::readVauxAvgRaw(uint16_t& value) {
  return readWord(kCmdReadVauxAvg, value);
}

bool LM5066H1::readTemperatureRaw(int16_t& value) {
  uint16_t raw = 0;
  if (!readWord(kCmdReadTemperature, raw)) {
    return false;
  }
  value = static_cast<int16_t>(raw);
  return true;
}

bool LM5066H1::readTempAvgRaw(uint16_t& value) {
  return readWord(kCmdReadTempAvg, value);
}

bool LM5066H1::readTempPeakRaw(uint16_t& value) {
  return readWord(kCmdReadTempPeak, value);
}

bool LM5066H1::readIinRaw(uint16_t& value) {
  return readWord(kCmdReadIinStd, value);
}

bool LM5066H1::readIinStdRaw(uint16_t& value) {
  return readIinRaw(value);
}

bool LM5066H1::readIinMfrRaw(uint16_t& value) {
  return readWord(kCmdReadIin, value);
}

bool LM5066H1::readIoutRaw(uint16_t& value) {
  return readWord(kCmdReadIout, value);
}

bool LM5066H1::readPinRaw(uint16_t& value) {
  return readWord(kCmdReadPinStd, value);
}

bool LM5066H1::readPinStdRaw(uint16_t& value) {
  return readPinRaw(value);
}

bool LM5066H1::readPinMfrRaw(uint16_t& value) {
  return readWord(kCmdReadPin, value);
}

bool LM5066H1::readPoutRaw(uint16_t& value) {
  return readWord(kCmdReadPout, value);
}

bool LM5066H1::readVinMinRaw(uint16_t& value) {
  return readWord(kCmdReadVinMin, value);
}

bool LM5066H1::readVinPeakRaw(uint16_t& value) {
  return readWord(kCmdReadVinPeak, value);
}

bool LM5066H1::readIinPeakRaw(uint16_t& value) {
  return readWord(kCmdReadIinPeak, value);
}

bool LM5066H1::readPinPeakRaw(uint16_t& value) {
  return readWord(kCmdReadPinPeakStd, value);
}

bool LM5066H1::readPinPeakStdRaw(uint16_t& value) {
  return readPinPeakRaw(value);
}

bool LM5066H1::readPinPeakMfrRaw(uint16_t& value) {
  return readWord(kCmdReadPinPeak, value);
}

bool LM5066H1::readVoutMinRaw(uint16_t& value) {
  return readWord(kCmdReadVoutMin, value);
}

bool LM5066H1::clearPinPeak() {
  return sendByte(kCmdClearPinPeak);
}

bool LM5066H1::readAvgVinRaw(uint16_t& value) {
  return readWord(kCmdReadAvgVin, value);
}

bool LM5066H1::readAvgVoutRaw(uint16_t& value) {
  return readWord(kCmdReadAvgVout, value);
}

bool LM5066H1::readAvgIinRaw(uint16_t& value) {
  return readWord(kCmdReadAvgIin, value);
}

bool LM5066H1::readAvgPinRaw(uint16_t& value) {
  return readWord(kCmdReadAvgPin, value);
}

bool LM5066H1::readIinStd(double& amps) {
  return readIin(amps);
}

bool LM5066H1::readIout(double& amps) {
  uint16_t raw = 0;
  if (!readIoutRaw(raw)) {
    return false;
  }
  amps = directToValue(raw, coeffIout());
  return true;
}

bool LM5066H1::readVin(double& volts) {
  uint16_t raw = 0;
  if (!readVinRaw(raw)) {
    return false;
  }
  volts = directToValue(raw, coeffVin());
  return true;
}

bool LM5066H1::readVout(double& volts) {
  uint16_t raw = 0;
  if (!readVoutRaw(raw)) {
    return false;
  }
  volts = directToValue(raw, coeffVout());
  return true;
}

bool LM5066H1::readVaux(double& volts) {
  uint16_t raw = 0;
  if (!readVauxRaw(raw)) {
    return false;
  }
  volts = directToValue(raw, coeffVaux());
  return true;
}

bool LM5066H1::readVauxAvg(double& volts) {
  uint16_t raw = 0;
  if (!readVauxAvgRaw(raw)) {
    return false;
  }
  volts = directToValue(raw, coeffVaux());
  return true;
}

bool LM5066H1::readNtcTemperatureC(double& celsius) {
  double v = 0.0;
  if (!readVaux(v)) {
    return false;
  }
  if (v <= 0.0 || v >= _ntcSupplyVolts) {
    return false;
  }

  double rNtc = (v * _ntcPullupOhms) / (_ntcSupplyVolts - v);
  if (rNtc <= 0.0 || _ntcNominalOhms <= 0.0 || _ntcBeta <= 0.0) {
    return false;
  }

  double t0 = _ntcNominalTempC + 273.15;
  double invT = (1.0 / t0) + (1.0 / _ntcBeta) * log(rNtc / _ntcNominalOhms);
  if (invT <= 0.0) {
    return false;
  }
  celsius = (1.0 / invT) - 273.15;
  return true;
}

bool LM5066H1::readTemperatureC(double& celsius) {
  int16_t raw = 0;
  if (!readTemperatureRaw(raw)) {
    return false;
  }
  celsius = directToValue(raw, coeffTemperature());
  return true;
}

bool LM5066H1::readTempAvgC(double& celsius) {
  uint16_t raw = 0;
  if (!readTempAvgRaw(raw)) {
    return false;
  }
  celsius = directToValue(static_cast<int16_t>(raw), coeffTemperatureAvg());
  return true;
}

bool LM5066H1::readTempPeakC(double& celsius) {
  uint16_t raw = 0;
  if (!readTempPeakRaw(raw)) {
    return false;
  }
  celsius = directToValue(static_cast<int16_t>(raw), coeffTemperature());
  return true;
}

bool LM5066H1::readIin(double& amps) {
  uint16_t raw = 0;
  if (!readIinRaw(raw)) {
    return false;
  }
  amps = directToValue(raw, coeffIin());
  return true;
}

bool LM5066H1::readPin(double& watts) {
  uint16_t raw = 0;
  if (!readPinRaw(raw)) {
    return false;
  }
  watts = directToValue(raw, coeffPin());
  return true;
}

bool LM5066H1::readPinStd(double& watts) {
  return readPin(watts);
}

bool LM5066H1::readPout(double& watts) {
  uint16_t raw = 0;
  if (!readPoutRaw(raw)) {
    return false;
  }
  watts = directToValue(raw, coeffPout());
  return true;
}

bool LM5066H1::readVinMin(double& volts) {
  uint16_t raw = 0;
  if (!readVinMinRaw(raw)) {
    return false;
  }
  volts = directToValue(raw, coeffVin());
  return true;
}

bool LM5066H1::readVinPeak(double& volts) {
  uint16_t raw = 0;
  if (!readVinPeakRaw(raw)) {
    return false;
  }
  volts = directToValue(raw, coeffVin());
  return true;
}

bool LM5066H1::readIinPeak(double& amps) {
  uint16_t raw = 0;
  if (!readIinPeakRaw(raw)) {
    return false;
  }
  amps = directToValue(raw, coeffIin());
  return true;
}

bool LM5066H1::readPinPeak(double& watts) {
  uint16_t raw = 0;
  if (!readPinPeakRaw(raw)) {
    return false;
  }
  watts = directToValue(raw, coeffPin());
  return true;
}

bool LM5066H1::readPinPeakStd(double& watts) {
  return readPinPeak(watts);
}

bool LM5066H1::readVoutMin(double& volts) {
  uint16_t raw = 0;
  if (!readVoutMinRaw(raw)) {
    return false;
  }
  volts = directToValue(raw, coeffVout());
  return true;
}

bool LM5066H1::readAvgVin(double& volts) {
  uint16_t raw = 0;
  if (!readAvgVinRaw(raw)) {
    return false;
  }
  volts = directToValue(raw, coeffVinAvg());
  return true;
}

bool LM5066H1::readAvgVout(double& volts) {
  uint16_t raw = 0;
  if (!readAvgVoutRaw(raw)) {
    return false;
  }
  volts = directToValue(raw, coeffVoutAvg());
  return true;
}

bool LM5066H1::readAvgIin(double& amps) {
  uint16_t raw = 0;
  if (!readAvgIinRaw(raw)) {
    return false;
  }
  amps = directToValue(raw, coeffIinAvg());
  return true;
}

bool LM5066H1::readAvgPin(double& watts) {
  uint16_t raw = 0;
  if (!readAvgPinRaw(raw)) {
    return false;
  }
  watts = directToValue(raw, coeffPinAvg());
  return true;
}

bool LM5066H1::writeIinOcWarnLimitRaw(uint16_t value) {
  return writeWord(kCmdIinOcWarnLimitStd, value);
}

bool LM5066H1::writeIinOcWarnLimitStdRaw(uint16_t value) {
  return writeIinOcWarnLimitRaw(value);
}

bool LM5066H1::writeIinOcWarnLimitMfrRaw(uint16_t value) {
  return writeWord(kCmdIinOcWarnLimit, value);
}

bool LM5066H1::writeOcWarnLimitRaw(uint16_t value) {
  return writeWord(kCmdOcWarnLimit, value);
}

bool LM5066H1::writeVinOvWarnLimitRaw(uint16_t value) {
  return writeWord(kCmdVinOvWarnLimit, value);
}

bool LM5066H1::writeVinUvWarnLimitRaw(uint16_t value) {
  return writeWord(kCmdVinUvWarnLimit, value);
}

bool LM5066H1::writeVoutUvWarnLimitRaw(uint16_t value) {
  return writeWord(kCmdVoutUvWarnLimit, value);
}

bool LM5066H1::writeOtWarnLimitRaw(uint16_t value) {
  return writeWord(kCmdOtWarnLimit, value);
}

bool LM5066H1::writeOtFaultLimitRaw(uint16_t value) {
  return writeWord(kCmdOtFaultLimit, value);
}

bool LM5066H1::writePinOpWarnLimitRaw(uint16_t value) {
  return writeWord(kCmdPinOpWarnLimit, value);
}

bool LM5066H1::readIinOcWarnLimitRaw(uint16_t& value) {
  return readWord(kCmdIinOcWarnLimitStd, value);
}

bool LM5066H1::readIinOcWarnLimitStdRaw(uint16_t& value) {
  return readIinOcWarnLimitRaw(value);
}

bool LM5066H1::readIinOcWarnLimitMfrRaw(uint16_t& value) {
  return readWord(kCmdIinOcWarnLimit, value);
}

bool LM5066H1::readOcWarnLimitRaw(uint16_t& value) {
  return readWord(kCmdOcWarnLimit, value);
}

bool LM5066H1::readVinOvWarnLimitRaw(uint16_t& value) {
  return readWord(kCmdVinOvWarnLimit, value);
}

bool LM5066H1::readVinUvWarnLimitRaw(uint16_t& value) {
  return readWord(kCmdVinUvWarnLimit, value);
}

bool LM5066H1::readVoutUvWarnLimitRaw(uint16_t& value) {
  return readWord(kCmdVoutUvWarnLimit, value);
}

bool LM5066H1::readOtWarnLimitRaw(uint16_t& value) {
  return readWord(kCmdOtWarnLimit, value);
}

bool LM5066H1::readOtFaultLimitRaw(uint16_t& value) {
  return readWord(kCmdOtFaultLimit, value);
}

bool LM5066H1::readPinOpWarnLimitRaw(uint16_t& value) {
  return readWord(kCmdPinOpWarnLimit, value);
}

bool LM5066H1::setIinOcWarnLimitStd(double amps) {
  uint16_t raw = 0;
  if (!valueToDirect(amps, coeffIin(), raw, true)) {
    return false;
  }
  return writeIinOcWarnLimitStdRaw(raw);
}

bool LM5066H1::setOcWarnLimit(double amps) {
  uint16_t raw = 0;
  if (!valueToDirect(amps, coeffIin(), raw, true)) {
    return false;
  }
  return writeOcWarnLimitRaw(raw);
}

bool LM5066H1::setVinOvWarnLimit(double volts) {
  uint16_t raw = 0;
  if (!valueToDirect(volts, coeffVin(), raw, true)) {
    return false;
  }
  return writeVinOvWarnLimitRaw(raw);
}

bool LM5066H1::setVinUvWarnLimit(double volts) {
  uint16_t raw = 0;
  if (!valueToDirect(volts, coeffVin(), raw, true)) {
    return false;
  }
  return writeVinUvWarnLimitRaw(raw);
}

bool LM5066H1::setVoutUvWarnLimit(double volts) {
  uint16_t raw = 0;
  if (!valueToDirect(volts, coeffVout(), raw, true)) {
    return false;
  }
  return writeVoutUvWarnLimitRaw(raw);
}

bool LM5066H1::setOtWarnLimit(double celsius) {
  uint16_t raw = 0;
  if (!valueToDirect(celsius, coeffTemperature(), raw, true)) {
    return false;
  }
  return writeOtWarnLimitRaw(raw);
}

bool LM5066H1::setOtFaultLimit(double celsius) {
  uint16_t raw = 0;
  if (!valueToDirect(celsius, coeffTemperature(), raw, true)) {
    return false;
  }
  return writeOtFaultLimitRaw(raw);
}

bool LM5066H1::setIinOcWarnLimit(double amps) {
  uint16_t raw = 0;
  if (!valueToDirect(amps, coeffIin(), raw, true)) {
    return false;
  }
  return writeIinOcWarnLimitRaw(raw);
}

bool LM5066H1::setIinOcWarnLimitMfr(double amps) {
  uint16_t raw = 0;
  if (!valueToDirect(amps, coeffIin(), raw, true)) {
    return false;
  }
  return writeIinOcWarnLimitMfrRaw(raw);
}

bool LM5066H1::setPinOpWarnLimit(double watts) {
  uint16_t raw = 0;
  if (!valueToDirect(watts, coeffPin(), raw, true)) {
    return false;
  }
  return writePinOpWarnLimitRaw(raw);
}

bool LM5066H1::readIinOcWarnLimitStd(double& amps) {
  uint16_t raw = 0;
  if (!readIinOcWarnLimitStdRaw(raw)) {
    return false;
  }
  amps = directToValue(raw, coeffIin());
  return true;
}

bool LM5066H1::readOcWarnLimit(double& amps) {
  uint16_t raw = 0;
  if (!readOcWarnLimitRaw(raw)) {
    return false;
  }
  amps = directToValue(raw, coeffIin());
  return true;
}

bool LM5066H1::readVinOvWarnLimit(double& volts) {
  uint16_t raw = 0;
  if (!readVinOvWarnLimitRaw(raw)) {
    return false;
  }
  volts = directToValue(raw, coeffVin());
  return true;
}

bool LM5066H1::readVinUvWarnLimit(double& volts) {
  uint16_t raw = 0;
  if (!readVinUvWarnLimitRaw(raw)) {
    return false;
  }
  volts = directToValue(raw, coeffVin());
  return true;
}

bool LM5066H1::readVoutUvWarnLimit(double& volts) {
  uint16_t raw = 0;
  if (!readVoutUvWarnLimitRaw(raw)) {
    return false;
  }
  volts = directToValue(raw, coeffVout());
  return true;
}

bool LM5066H1::readOtWarnLimit(double& celsius) {
  uint16_t raw = 0;
  if (!readOtWarnLimitRaw(raw)) {
    return false;
  }
  celsius = directToValue(static_cast<int16_t>(raw), coeffTemperature());
  return true;
}

bool LM5066H1::readOtFaultLimit(double& celsius) {
  uint16_t raw = 0;
  if (!readOtFaultLimitRaw(raw)) {
    return false;
  }
  celsius = directToValue(static_cast<int16_t>(raw), coeffTemperature());
  return true;
}

bool LM5066H1::readIinOcWarnLimit(double& amps) {
  uint16_t raw = 0;
  if (!readIinOcWarnLimitRaw(raw)) {
    return false;
  }
  amps = directToValue(raw, coeffIin());
  return true;
}

bool LM5066H1::readIinOcWarnLimitMfr(double& amps) {
  uint16_t raw = 0;
  if (!readIinOcWarnLimitMfrRaw(raw)) {
    return false;
  }
  amps = directToValue(raw, coeffIin());
  return true;
}

bool LM5066H1::readPinOpWarnLimit(double& watts) {
  uint16_t raw = 0;
  if (!readPinOpWarnLimitRaw(raw)) {
    return false;
  }
  watts = directToValue(raw, coeffPin());
  return true;
}

bool LM5066H1::setAlertMask(uint16_t mask) {
  return writeWord(kCmdAlertMask, mask);
}

bool LM5066H1::setAlertMask(const AlertMaskBits& mask) {
  uint16_t raw = 0;
  if (mask.voutUndervoltageWarn) raw |= (1u << 15);
  if (mask.iinLimitWarn) raw |= (1u << 14);
  if (mask.vinUndervoltageWarn) raw |= (1u << 13);
  if (mask.vinOvervoltageWarn) raw |= (1u << 12);
  if (mask.powerGood) raw |= (1u << 11);
  if (mask.overtempWarn) raw |= (1u << 10);
  if (mask.watchdogFault) raw |= (1u << 9);
  if (mask.overpowerLimitWarn) raw |= (1u << 8);
  if (mask.scpFault) raw |= (1u << 7);
  if (mask.fetFailFault) raw |= (1u << 6);
  if (mask.vinUndervoltageFault) raw |= (1u << 5);
  if (mask.vinOvervoltageFault) raw |= (1u << 4);
  if (mask.iinPfetFault) raw |= (1u << 3);
  if (mask.overtempFault) raw |= (1u << 2);
  if (mask.cmlFault) raw |= (1u << 1);
  if (mask.circuitBreakerFault) raw |= (1u << 0);
  return setAlertMask(raw);
}

bool LM5066H1::readAlertMask(uint16_t& mask) {
  return readWord(kCmdAlertMask, mask);
}

bool LM5066H1::readAlertMask(AlertMaskBits& mask) {
  uint16_t raw = 0;
  if (!readAlertMask(raw)) {
    return false;
  }
  mask.voutUndervoltageWarn = (raw >> 15) & 0x1;
  mask.iinLimitWarn = (raw >> 14) & 0x1;
  mask.vinUndervoltageWarn = (raw >> 13) & 0x1;
  mask.vinOvervoltageWarn = (raw >> 12) & 0x1;
  mask.powerGood = (raw >> 11) & 0x1;
  mask.overtempWarn = (raw >> 10) & 0x1;
  mask.watchdogFault = (raw >> 9) & 0x1;
  mask.overpowerLimitWarn = (raw >> 8) & 0x1;
  mask.scpFault = (raw >> 7) & 0x1;
  mask.fetFailFault = (raw >> 6) & 0x1;
  mask.vinUndervoltageFault = (raw >> 5) & 0x1;
  mask.vinOvervoltageFault = (raw >> 4) & 0x1;
  mask.iinPfetFault = (raw >> 3) & 0x1;
  mask.overtempFault = (raw >> 2) & 0x1;
  mask.cmlFault = (raw >> 1) & 0x1;
  mask.circuitBreakerFault = (raw & 0x1) != 0;
  return true;
}

bool LM5066H1::setGateMask(uint8_t mask) {
  return writeByte(kCmdGateMask, mask);
}

bool LM5066H1::setGateMask(const GateMaskBits& mask) {
  uint8_t raw = 0;
  if (mask.scpFault) raw |= (1u << 7);
  if (mask.fetFail) raw |= (1u << 6);
  if (mask.vinUvFault) raw |= (1u << 5);
  if (mask.vinOvFault) raw |= (1u << 4);
  if (mask.iinPfetFault) raw |= (1u << 3);
  if (mask.overtempFault) raw |= (1u << 2);
  if (mask.watchdogFault) raw |= (1u << 1);
  if (mask.circuitBreakerFault) raw |= (1u << 0);
  return setGateMask(raw);
}

bool LM5066H1::readGateMask(uint8_t& mask) {
  return readByte(kCmdGateMask, mask);
}

bool LM5066H1::readGateMask(GateMaskBits& mask) {
  uint8_t raw = 0;
  if (!readGateMask(raw)) {
    return false;
  }
  mask.scpFault = (raw >> 7) & 0x1;
  mask.fetFail = (raw >> 6) & 0x1;
  mask.vinUvFault = (raw >> 5) & 0x1;
  mask.vinOvFault = (raw >> 4) & 0x1;
  mask.iinPfetFault = (raw >> 3) & 0x1;
  mask.overtempFault = (raw >> 2) & 0x1;
  mask.watchdogFault = (raw >> 1) & 0x1;
  mask.circuitBreakerFault = (raw & 0x1) != 0;
  return true;
}

bool LM5066H1::setDeviceSetup1(uint8_t value) {
  return writeByte(kCmdDeviceSetup1, value);
}

bool LM5066H1::readDeviceSetup1(uint8_t& value) {
  return readByte(kCmdDeviceSetup1, value);
}

bool LM5066H1::setDeviceSetup1(const DeviceSetup1Bits& config) {
  uint8_t raw = (config.retrySetting & 0x7) << 5;
  if (config.currentLimitSetting) raw |= (1u << 4);
  if (config.cbClRatio) raw |= (1u << 3);
  if (config.currentLimitConfig) raw |= (1u << 2);
  if (config.permanentWriteDisable) raw |= (1u << 0);
  return setDeviceSetup1(raw);
}

bool LM5066H1::readDeviceSetup1(DeviceSetup1Bits& config) {
  uint8_t raw = 0;
  if (!readDeviceSetup1(raw)) {
    return false;
  }
  config.retrySetting = (raw >> 5) & 0x7;
  config.currentLimitSetting = (raw >> 4) & 0x1;
  config.cbClRatio = (raw >> 3) & 0x1;
  config.currentLimitConfig = (raw >> 2) & 0x1;
  config.permanentWriteDisable = (raw & 0x1) != 0;
  return true;
}

bool LM5066H1::setDeviceSetup2(uint8_t value) {
  return writeByte(kCmdDeviceSetup2, value);
}

bool LM5066H1::readDeviceSetup2(uint8_t& value) {
  return readByte(kCmdDeviceSetup2, value);
}

bool LM5066H1::setDeviceSetup2(const DeviceSetup2Bits& config) {
  uint8_t raw = (config.cbClRatio2 & 0x3) << 6;
  raw |= (config.currentLimitSetting2 & 0x7) << 3;
  if (config.fastRecoveryCb) raw |= (1u << 1);
  if (config.vinTranEnable) raw |= (1u << 0);
  return setDeviceSetup2(raw);
}

bool LM5066H1::readDeviceSetup2(DeviceSetup2Bits& config) {
  uint8_t raw = 0;
  if (!readDeviceSetup2(raw)) {
    return false;
  }
  config.cbClRatio2 = (raw >> 6) & 0x3;
  config.currentLimitSetting2 = (raw >> 3) & 0x7;
  config.fastRecoveryCb = (raw >> 1) & 0x1;
  config.vinTranEnable = (raw & 0x1) != 0;
  return true;
}

bool LM5066H1::setDeviceSetup3(uint8_t value) {
  return writeByte(kCmdDeviceSetup3, value);
}

bool LM5066H1::readDeviceSetup3(uint8_t& value) {
  return readByte(kCmdDeviceSetup3, value);
}

bool LM5066H1::setDeviceSetup3(const DeviceSetup3Bits& config) {
  uint8_t raw = 0;
  if (config.currentLimitingDisable) raw |= (1u << 7);
  if (config.powerLimitProfile) raw |= (1u << 6);
  raw |= (config.foldbackCurrentLimit & 0x3) << 4;
  raw |= (config.ocBlanking2Threshold & 0x3) << 2;
  raw |= (config.ocBlanking1Threshold & 0x3);
  return setDeviceSetup3(raw);
}

bool LM5066H1::readDeviceSetup3(DeviceSetup3Bits& config) {
  uint8_t raw = 0;
  if (!readDeviceSetup3(raw)) {
    return false;
  }
  config.currentLimitingDisable = (raw >> 7) & 0x1;
  config.powerLimitProfile = (raw >> 6) & 0x1;
  config.foldbackCurrentLimit = (raw >> 4) & 0x3;
  config.ocBlanking2Threshold = (raw >> 2) & 0x3;
  config.ocBlanking1Threshold = (raw & 0x3);
  return true;
}

bool LM5066H1::setDeviceSetup4(uint8_t value) {
  return writeByte(kCmdDeviceSetup4, value);
}

bool LM5066H1::readDeviceSetup4(uint8_t& value) {
  return readByte(kCmdDeviceSetup4, value);
}

bool LM5066H1::setDeviceSetup4(const DeviceSetup4Bits& config) {
  uint8_t raw = 0;
  if (config.immediateRetry) raw |= (1u << 7);
  if (config.regulationTimerDischarge) raw |= (1u << 6);
  if (config.syncPinFunction) raw |= (1u << 5);
  if (config.sftSrt) raw |= (1u << 4);
  raw |= (config.regulationTimerSetting & 0x3) << 2;
  raw |= (config.powerLimitBlankingVds & 0x3);
  return setDeviceSetup4(raw);
}

bool LM5066H1::readDeviceSetup4(DeviceSetup4Bits& config) {
  uint8_t raw = 0;
  if (!readDeviceSetup4(raw)) {
    return false;
  }
  config.immediateRetry = (raw >> 7) & 0x1;
  config.regulationTimerDischarge = (raw >> 6) & 0x1;
  config.syncPinFunction = (raw >> 5) & 0x1;
  config.sftSrt = (raw >> 4) & 0x1;
  config.regulationTimerSetting = (raw >> 2) & 0x3;
  config.powerLimitBlankingVds = (raw & 0x3);
  return true;
}

bool LM5066H1::setDeviceSetup5(uint8_t value) {
  return writeByte(kCmdDeviceSetup5, value);
}

bool LM5066H1::readDeviceSetup5(uint8_t& value) {
  return readByte(kCmdDeviceSetup5, value);
}

bool LM5066H1::setDeviceSetup5(const DeviceSetup5Bits& config) {
  uint8_t raw = 0;
  if (config.gate1OvercurrentPullDown) raw |= (1u << 4);
  if (config.gate2OvercurrentPullDown) raw |= (1u << 3);
  if (config.gate12OvertempPullDown) raw |= (1u << 2);
  if (config.gate12OvloPullDown) raw |= (1u << 1);
  if (config.gate12UvloPullDown) raw |= (1u << 0);
  return setDeviceSetup5(raw);
}

bool LM5066H1::readDeviceSetup5(DeviceSetup5Bits& config) {
  uint8_t raw = 0;
  if (!readDeviceSetup5(raw)) {
    return false;
  }
  config.gate1OvercurrentPullDown = (raw >> 4) & 0x1;
  config.gate2OvercurrentPullDown = (raw >> 3) & 0x1;
  config.gate12OvertempPullDown = (raw >> 2) & 0x1;
  config.gate12OvloPullDown = (raw >> 1) & 0x1;
  config.gate12UvloPullDown = (raw & 0x1) != 0;
  return true;
}

bool LM5066H1::setOcBlankingTimers(uint8_t value) {
  return writeByte(kCmdOcBlankingTimers, value);
}

bool LM5066H1::readOcBlankingTimers(uint8_t& value) {
  return readByte(kCmdOcBlankingTimers, value);
}

bool LM5066H1::setOcBlankingTimers(const OcBlankingTimersBits& config) {
  uint8_t raw = (config.blanking2 & 0xF) << 4;
  raw |= (config.blanking1 & 0xF);
  return setOcBlankingTimers(raw);
}

bool LM5066H1::readOcBlankingTimers(OcBlankingTimersBits& config) {
  uint8_t raw = 0;
  if (!readOcBlankingTimers(raw)) {
    return false;
  }
  config.blanking2 = (raw >> 4) & 0xF;
  config.blanking1 = (raw & 0xF);
  return true;
}

bool LM5066H1::setDelayConfig(uint8_t value) {
  return writeByte(kCmdDelayConfig, value);
}

bool LM5066H1::readDelayConfig(uint8_t& value) {
  return readByte(kCmdDelayConfig, value);
}

bool LM5066H1::setDelayConfig(const DelayConfigBits& config) {
  uint8_t raw = (config.retryDelay & 0xF) << 4;
  raw |= (config.insertionDelay & 0xF);
  return setDelayConfig(raw);
}

bool LM5066H1::readDelayConfig(DelayConfigBits& config) {
  uint8_t raw = 0;
  if (!readDelayConfig(raw)) {
    return false;
  }
  config.retryDelay = (raw >> 4) & 0xF;
  config.insertionDelay = (raw & 0xF);
  return true;
}

bool LM5066H1::setWdPlbTimer(uint8_t value) {
  return writeByte(kCmdWdPlbTimer, value);
}

bool LM5066H1::readWdPlbTimer(uint8_t& value) {
  return readByte(kCmdWdPlbTimer, value);
}

bool LM5066H1::setWdPlbTimer(const WdPlbTimerBits& config) {
  uint8_t raw = (config.watchdogTimer & 0xF) << 4;
  raw |= (config.powerLimitBlanking & 0xF);
  return setWdPlbTimer(raw);
}

bool LM5066H1::readWdPlbTimer(WdPlbTimerBits& config) {
  uint8_t raw = 0;
  if (!readWdPlbTimer(raw)) {
    return false;
  }
  config.watchdogTimer = (raw >> 4) & 0xF;
  config.powerLimitBlanking = (raw & 0xF);
  return true;
}

bool LM5066H1::setPkMinAvg(uint8_t value) {
  return writeByte(kCmdPkMinAvg, value);
}

bool LM5066H1::readPkMinAvg(uint8_t& value) {
  return readByte(kCmdPkMinAvg, value);
}

bool LM5066H1::setPkMinAvg(const PkMinAvgBits& config) {
  uint8_t raw = 0;
  if (config.resetPeak) raw |= (1u << 7);
  if (config.resetAvg) raw |= (1u << 6);
  if (config.resetMin) raw |= (1u << 5);
  return setPkMinAvg(raw);
}

bool LM5066H1::readPkMinAvg(PkMinAvgBits& config) {
  uint8_t raw = 0;
  if (!readPkMinAvg(raw)) {
    return false;
  }
  config.resetPeak = (raw >> 7) & 0x1;
  config.resetAvg = (raw >> 6) & 0x1;
  config.resetMin = (raw >> 5) & 0x1;
  return true;
}

bool LM5066H1::setP2tTimer(uint8_t value) {
  return writeByte(kCmdP2tTimer, value);
}

bool LM5066H1::readP2tTimer(uint8_t& value) {
  return readByte(kCmdP2tTimer, value);
}

bool LM5066H1::setP2tTimer(const P2tTimerBits& config) {
  uint8_t raw = 0;
  if (config.absTempMode) raw |= (1u << 4);
  raw |= (config.maxDuration & 0xF);
  return setP2tTimer(raw);
}

bool LM5066H1::readP2tTimer(P2tTimerBits& config) {
  uint8_t raw = 0;
  if (!readP2tTimer(raw)) {
    return false;
  }
  config.absTempMode = (raw >> 4) & 0x1;
  config.maxDuration = (raw & 0xF);
  return true;
}

bool LM5066H1::setBbConfig(uint8_t value) {
  return writeByte(kCmdBbConfig, value);
}

bool LM5066H1::readBbConfig(uint8_t& value) {
  return readByte(kCmdBbConfig, value);
}

bool LM5066H1::setBbConfig(const BbConfigBits& config) {
  uint8_t raw = 0;
  if (config.fetOffWr) raw |= (1u << 7);
  if (config.fltWr) raw |= (1u << 6);
  if (config.alertWr) raw |= (1u << 5);
  raw |= (config.bbTick & 0x3);
  return setBbConfig(raw);
}

bool LM5066H1::readBbConfig(BbConfigBits& config) {
  uint8_t raw = 0;
  if (!readBbConfig(raw)) {
    return false;
  }
  config.fetOffWr = (raw >> 7) & 0x1;
  config.fltWr = (raw >> 6) & 0x1;
  config.alertWr = (raw >> 5) & 0x1;
  config.bbTick = (raw & 0x3);
  return true;
}

bool LM5066H1::readBbTimer(uint8_t& value) {
  return readByte(kCmdBbTimer, value);
}

bool LM5066H1::readBbTimer(BbTimerBits& config) {
  uint8_t raw = 0;
  if (!readBbTimer(raw)) {
    return false;
  }
  config.bbPtr = (raw >> 5) & 0x7;
  config.timerExpired = (raw >> 4) & 0x1;
  config.bbTick = (raw & 0xF);
  return true;
}

bool LM5066H1::setAdcConfig1(uint8_t value) {
  return writeByte(kCmdAdcConfig1, value);
}

bool LM5066H1::readAdcConfig1(uint8_t& value) {
  return readByte(kCmdAdcConfig1, value);
}

bool LM5066H1::setAdcConfig1(const AdcConfig1Bits& config) {
  uint8_t raw = 0;
  if (config.eoc) raw |= (1u << 7);
  if (config.convst) raw |= (1u << 6);
  raw |= (config.mode & 0x3) << 4;
  raw |= (config.convChSel & 0xF);
  return setAdcConfig1(raw);
}

bool LM5066H1::readAdcConfig1(AdcConfig1Bits& config) {
  uint8_t raw = 0;
  if (!readAdcConfig1(raw)) {
    return false;
  }
  config.eoc = (raw >> 7) & 0x1;
  config.convst = (raw >> 6) & 0x1;
  config.mode = (raw >> 4) & 0x3;
  config.convChSel = (raw & 0xF);
  return true;
}

bool LM5066H1::setAdcConfig2(uint8_t value) {
  bool ok = writeByte(kCmdAdcConfig2, value);
  _adcFullScale2x = (value >> 6) & 0x1;
  return ok;
}

bool LM5066H1::readAdcConfig2(uint8_t& value) {
  bool ok = readByte(kCmdAdcConfig2, value);
  if (ok) {
    _adcFullScale2x = (value >> 6) & 0x1;
  }
  return ok;
}

bool LM5066H1::setAdcConfig2(const AdcConfig2Bits& config) {
  uint8_t raw = 0;
  if (config.fullScale) raw |= (1u << 6);
  raw |= (config.bufChSel & 0x7) << 3;
  raw |= (config.decRate & 0x7);
  return setAdcConfig2(raw);
}

bool LM5066H1::readAdcConfig2(AdcConfig2Bits& config) {
  uint8_t raw = 0;
  if (!readAdcConfig2(raw)) {
    return false;
  }
  config.fullScale = (raw >> 6) & 0x1;
  config.bufChSel = (raw >> 3) & 0x7;
  config.decRate = (raw & 0x7);
  return true;
}

bool LM5066H1::setIinOffsetCalibration(uint8_t value) {
  return writeByte(kCmdIinOffsetCalibration, value);
}

bool LM5066H1::readIinOffsetCalibration(uint8_t& value) {
  return readByte(kCmdIinOffsetCalibration, value);
}

bool LM5066H1::setIinOffsetCalibration(const IinOffsetCalibrationBits& config) {
  uint8_t raw = 0;
  if (config.negative) raw |= (1u << 7);
  raw |= (config.offsetFactor & 0x7F);
  return setIinOffsetCalibration(raw);
}

bool LM5066H1::readIinOffsetCalibration(IinOffsetCalibrationBits& config) {
  uint8_t raw = 0;
  if (!readIinOffsetCalibration(raw)) {
    return false;
  }
  config.negative = (raw >> 7) & 0x1;
  config.offsetFactor = (raw & 0x7F);
  return true;
}

bool LM5066H1::setPmbusAddress(uint8_t value) {
  return writeByte(kCmdPmbusAddr, value);
}

bool LM5066H1::readPmbusAddress(uint8_t& value) {
  return readByte(kCmdPmbusAddr, value);
}

bool LM5066H1::setUserData(uint8_t value) {
  return writeByte(kCmdUserData, value);
}

bool LM5066H1::readUserData(uint8_t& value) {
  return readByte(kCmdUserData, value);
}

bool LM5066H1::setSamplesForAvg(uint8_t value) {
  return writeByte(kCmdSamplesForAvg, value);
}

bool LM5066H1::readSamplesForAvg(uint8_t& value) {
  return readByte(kCmdSamplesForAvg, value);
}

bool LM5066H1::readBlockRead(uint8_t* data, size_t len, size_t& outLen) {
  return readBlockCommand(kCmdBlockRead, data, len, outLen);
}

bool LM5066H1::readBlockRead(BlockReadData& data) {
  uint8_t buffer[12] = {};
  size_t outLen = 0;
  if (!readBlockRead(buffer, sizeof(buffer), outLen)) {
    return false;
  }
  if (outLen < sizeof(buffer)) {
    return false;
  }
  data.diagnosticWord = static_cast<uint16_t>(buffer[1] << 8) | buffer[0];
  data.readIin = static_cast<uint16_t>(buffer[3] << 8) | buffer[2];
  data.readVout = static_cast<uint16_t>(buffer[5] << 8) | buffer[4];
  data.readVin = static_cast<uint16_t>(buffer[7] << 8) | buffer[6];
  data.readPin = static_cast<uint16_t>(buffer[9] << 8) | buffer[8];
  data.readTemperature =
      static_cast<uint16_t>(buffer[11] << 8) | buffer[10];
  return true;
}

bool LM5066H1::readAvgBlockRead(uint8_t* data, size_t len, size_t& outLen) {
  return readBlockCommand(kCmdAvgBlockRead, data, len, outLen);
}

bool LM5066H1::readAvgBlockRead(AvgBlockReadData& data) {
  uint8_t buffer[12] = {};
  size_t outLen = 0;
  if (!readAvgBlockRead(buffer, sizeof(buffer), outLen)) {
    return false;
  }
  if (outLen < sizeof(buffer)) {
    return false;
  }
  data.diagnosticWord = static_cast<uint16_t>(buffer[1] << 8) | buffer[0];
  data.avgIin = static_cast<uint16_t>(buffer[3] << 8) | buffer[2];
  data.avgVout = static_cast<uint16_t>(buffer[5] << 8) | buffer[4];
  data.avgVin = static_cast<uint16_t>(buffer[7] << 8) | buffer[6];
  data.avgPin = static_cast<uint16_t>(buffer[9] << 8) | buffer[8];
  data.temperature = static_cast<uint16_t>(buffer[11] << 8) | buffer[10];
  return true;
}

bool LM5066H1::bbClear() {
  return sendByte(kCmdBbClear);
}

bool LM5066H1::bbErase() {
  return sendByte(kCmdBbErase);
}

bool LM5066H1::fetchBbEeprom() {
  return sendByte(kCmdFetchBbEeprom);
}

bool LM5066H1::readBlackBoxRead(uint8_t* data, size_t len, size_t& outLen) {
  return readBlockCommand(kCmdReadBbRam, data, len, outLen);
}

bool LM5066H1::readSampleBuffer(uint8_t* data, size_t len, size_t& outLen) {
  return readBlockCommand(kCmdReadSampleBuf, data, len, outLen);
}

bool LM5066H1::readBbRam(uint8_t* data, size_t len, size_t& outLen) {
  return readBlockCommand(kCmdReadBbRam, data, len, outLen);
}

bool LM5066H1::readBbRam(BbRamEntry& entry) {
  uint8_t buffer[7] = {};
  size_t outLen = 0;
  if (!readBbRam(buffer, sizeof(buffer), outLen)) {
    return false;
  }
  if (outLen < sizeof(buffer)) {
    return false;
  }
  uint8_t raw = buffer[0];
  entry.eventId = (raw >> 5) & 0x7;
  entry.timerExpired = (raw >> 4) & 0x1;
  entry.tick = raw & 0xF;
  return true;
}

bool LM5066H1::readBbEeprom(uint8_t* data, size_t len, size_t& outLen) {
  return readBlockCommand(kCmdReadBbEeprom, data, len, outLen);
}

bool LM5066H1::readMfrId(char* out, size_t maxLen) {
  uint8_t buffer[16] = {};
  size_t outLen = 0;
  if (!readBlockCommand(kCmdMfrId, buffer, sizeof(buffer), outLen)) {
    return false;
  }
  size_t toCopy = (outLen < maxLen - 1) ? outLen : maxLen - 1;
  memcpy(out, buffer, toCopy);
  out[toCopy] = '\0';
  return true;
}

bool LM5066H1::readMfrModel(char* out, size_t maxLen) {
  uint8_t buffer[16] = {};
  size_t outLen = 0;
  if (!readBlockCommand(kCmdMfrModel, buffer, sizeof(buffer), outLen)) {
    return false;
  }
  size_t toCopy = (outLen < maxLen - 1) ? outLen : maxLen - 1;
  memcpy(out, buffer, toCopy);
  out[toCopy] = '\0';
  return true;
}

bool LM5066H1::readMfrRevision(char* out, size_t maxLen) {
  uint8_t buffer[16] = {};
  size_t outLen = 0;
  if (!readBlockCommand(kCmdMfrRevision, buffer, sizeof(buffer), outLen)) {
    return false;
  }
  size_t toCopy = (outLen < maxLen - 1) ? outLen : maxLen - 1;
  memcpy(out, buffer, toCopy);
  out[toCopy] = '\0';
  return true;
}

bool LM5066H1::bridgeRead(uint8_t cmd, uint8_t* data, size_t len, size_t& outLen) {
  outLen = 0U;
  if (data == nullptr || len == 0U || len > 32U) {
    return false;
  }

  _bus->beginTransmission(_address);
  _bus->write(cmd);
  if (_bus->endTransmission(false) != 0) {
    return false;
  }

  const uint8_t wanted = static_cast<uint8_t>(len);
  const size_t received = _bus->requestFrom(_address, wanted);
  while (_bus->available() && outLen < len) {
    data[outLen] = static_cast<uint8_t>(_bus->read());
    ++outLen;
  }
  return received == wanted && outLen == len;
}

bool LM5066H1::bridgeWrite(uint8_t cmd, const uint8_t* data, size_t len) {
  if (len > 32U) {
    return false;
  }

  _bus->beginTransmission(_address);
  _bus->write(cmd);
  for (size_t i = 0U; i < len; ++i) {
    _bus->write(data[i]);
  }
  return _bus->endTransmission(true) == 0;
}

size_t LM5066H1::registerDescriptorCount() {
  return sizeof(kLm5066H1Registers) / sizeof(kLm5066H1Registers[0]);
}

const LM5066H1::RegisterDescriptor& LM5066H1::registerDescriptor(
    size_t index) {
  const size_t count = registerDescriptorCount();
  if (index >= count) {
    index = 0U;
  }
  return kLm5066H1Registers[index];
}

const LM5066H1::RegisterDescriptor* LM5066H1::findRegisterDescriptor(
    uint8_t cmd) {
  for (size_t i = 0U; i < registerDescriptorCount(); ++i) {
    if (kLm5066H1Registers[i].code == cmd) {
      return &kLm5066H1Registers[i];
    }
  }
  return nullptr;
}

const char* LM5066H1::commandName(uint8_t cmd) {
  const RegisterDescriptor* desc = findRegisterDescriptor(cmd);
  return (desc != nullptr) ? desc->name : "UNKNOWN";
}

double LM5066H1::ocBlankingTimerMs(uint8_t nibble, bool longRange) {
  static const double kTimer1[] = {
      0.0, 0.019, 0.095, 0.475, 0.7125, 0.95, 1.9, 3.8,
      7.6, 9.5, 14.25, 19.0, 38.0, 57.0, 76.0, 95.0};
  static const double kTimer2[] = {
      0.0, 0.038, 0.057, 0.095, 0.19, 0.285, 0.38, 0.57,
      0.76, 0.95, 1.9, 2.85, 3.8, 4.75, 9.5, 95.0};
  nibble &= 0xF;
  return longRange ? kTimer2[nibble] : kTimer1[nibble];
}

double LM5066H1::ocBlankingTimer2Ms(uint8_t nibble) {
  return ocBlankingTimerMs(nibble, true);
}

double LM5066H1::ocBlankingTimer1Ms(uint8_t nibble) {
  return ocBlankingTimerMs(nibble, false);
}

double LM5066H1::retryDelaySeconds(uint8_t nibble) {
  static const double kRetry[] = {0.0095, 0.019, 0.0475, 0.095, 0.19, 0.285,
                                  0.475, 0.7125, 0.95, 1.9, 2.85, 4.75,
                                  9.5, 19.0, 47.5, 95.0};
  return kRetry[nibble & 0xF];
}

double LM5066H1::insertionDelayMs(uint8_t nibble) {
  static const double kIns[] = {0.95, 1.9, 4.75, 6.65, 9.5, 19.0, 28.5, 38.0,
                                47.5, 66.5, 85.5, 95.0, 285.0, 475.0, 665.0,
                                950.0};
  return kIns[nibble & 0xF];
}

double LM5066H1::watchdogTimerMs(uint8_t nibble) {
  static const double kWd[] = {9.5, 19.0, 28.5, 38.0, 47.5, 95.0, 142.5, 190.0,
                               237.5, 475.0, 712.5, 950.0, 1900.0, 2850.0,
                               4750.0, 9500.0};
  return kWd[nibble & 0xF];
}

double LM5066H1::powerLimitBlankingMs(uint8_t nibble) {
  static const double kPlb[] = {0.0, 0.038, 0.057, 0.095, 0.19, 0.285, 0.38,
                                0.475, 0.7125, 0.95, 1.9, 4.75, 9.5, 19.0,
                                47.5, 95.0};
  return kPlb[nibble & 0xF];
}

double LM5066H1::p2tMaxDurationUs(uint8_t nibble) {
  static const double kP2t[] = {95, 190, 285, 380, 475, 665, 760, 855,
                                950, 1900, 4750, 6650, 9500, 19000, 47500,
                                95000};
  return kP2t[nibble & 0xF];
}

double LM5066H1::bbTickMicroseconds(uint8_t bbTick) {
  switch (bbTick & 0x3) {
    case 0:
      return 10.0;
    case 1:
      return 200.0;
    case 2:
      return 800.0;
    case 3:
      return 3200.0;
    default:
      return 10.0;
  }
}

uint16_t LM5066H1::samplesForAvgCount(uint8_t exponent) {
  if (exponent > 12) {
    exponent = 12;
  }
  return static_cast<uint16_t>(1u << exponent);
}

double LM5066H1::samplesForAvgPeriodMs(uint8_t exponent) {
  return 0.02 * static_cast<double>(samplesForAvgCount(exponent));
}

const char* LM5066H1::bbRamEventName(uint8_t eventId) {
  switch (eventId & 0x7) {
    case 0:
      return "NONE";
    case 1:
      return "IN_OP_WARN";
    case 2:
      return "VIN_TRAN";
    case 3:
      return "OC_DET";
    case 4:
      return "OT_WARN";
    case 5:
      return "OC_WARN";
    case 6:
      return "VIN_OV_WARN";
    case 7:
      return "VIN_UV_WARN";
    default:
      return "UNKNOWN";
  }
}

LM5066H1::DirectCoefficients LM5066H1::coeffVin() const {
  return {4596.0, 255.0, -2};
}

LM5066H1::DirectCoefficients LM5066H1::coeffVinAvg() const {
  return {4596.0, 233.0, -2};
}

LM5066H1::DirectCoefficients LM5066H1::coeffVout() const {
  return {4596.0, 455.0, -2};
}

LM5066H1::DirectCoefficients LM5066H1::coeffVoutAvg() const {
  return {4596.0, 417.0, -2};
}

LM5066H1::DirectCoefficients LM5066H1::coeffVaux() const {
  return {13788.0, 23.0, -1};
}

LM5066H1::DirectCoefficients LM5066H1::coeffTemperature() const {
  return {100.0, 26437.0, -2};
}

LM5066H1::DirectCoefficients LM5066H1::coeffTemperatureAvg() const {
  return {100.0, 26437.0, -2};
}

LM5066H1::DirectCoefficients LM5066H1::coeffIin() const {
  static const CoeffRow kIin1x[] = {
      {100, 3791.7, 10.25, -1},
      {125, 30333.3, 395.57, -2},
      {150, 25277.8, -209.0, -2},
      {175, 21666.8, 4.23, -2},
      {200, 18958.3, -246.36, -2},
      {225, 16851.9, -54.93, -2},
      {250, 15166.7, 63.89, -2},
      {500, 7583.3, 237.03, -2},
  };
  static const CoeffRow kIin2x[] = {
      {100, 18958.3, 461.02, -2},
      {125, 15166.7, 621.66, -2},
      {150, 12638.9, 121.11, -2},
      {175, 10833.3, 250.83, -2},
      {200, 9479.2, 58.24, -2},
      {225, 8425.9, 193.76, -2},
      {250, 7583.3, 281.86, -2},
      {500, 3791.7, 414.51, -2},
  };
  return coeffFromTable(kIin1x, kIin2x, sizeof(kIin1x) / sizeof(kIin1x[0]));
}

LM5066H1::DirectCoefficients LM5066H1::coeffIinAvg() const {
  static const CoeffRow kIinAvg1x[] = {
      {100, 3791.7, 18.78, -1},
      {125, 30333.3, 441.95, -2},
      {150, 25277.8, -150.06, -2},
      {175, 21666.7, 27.64, -2},
      {200, 18958.3, -240.23, -2},
      {225, 16851.9, -51.31, -2},
      {250, 15166.7, 58.36, -2},
      {500, 7583.3, 220.65, -2},
  };
  static const CoeffRow kIinAvg2x[] = {
      {100, 18958.3, 457.6, -2},
      {125, 15166.7, 614.42, -2},
      {150, 12638.9, 103.87, -2},
      {175, 10833.3, 259.76, -2},
      {200, 9479.2, 53.34, -2},
      {225, 8425.9, 178.01, -2},
      {250, 7583.3, 333.99, -2},
      {500, 3791.7, 401.74, -2},
  };
  return coeffFromTable(kIinAvg1x, kIinAvg2x,
                        sizeof(kIinAvg1x) / sizeof(kIinAvg1x[0]));
}

LM5066H1::DirectCoefficients LM5066H1::coeffIout() const {
  return coeffIin();
}

LM5066H1::DirectCoefficients LM5066H1::coeffPin() const {
  static const CoeffRow kPin1x[] = {
      {100, 4255.5, 6690.0, -3},
      {125, 3404.4, 8003.0, -3},
      {150, 28370.1, 4699.0, -4},
      {175, 24317.2, 5764.0, -4},
      {200, 21277.6, 4422.0, -4},
      {225, 18913.4, 5378.0, -4},
      {250, 17022.1, 6005.0, -4},
      {500, 8511.0, 6868.0, -4},
  };
  static const CoeffRow kPin2x[] = {
      {100, 22979.8, 6692.0, -4},
      {125, 18383.8, 7998.0, -4},
      {150, 15319.9, 4711.0, -4},
      {175, 13131.3, 5779.0, -4},
      {200, 11489.9, 4418.0, -4},
      {225, 10213.2, 5387.0, -4},
      {250, 9191.9, 6026.0, -4},
      {500, 4596.0, 6871.0, -4},
  };
  return coeffFromTable(kPin1x, kPin2x, sizeof(kPin1x) / sizeof(kPin1x[0]));
}

LM5066H1::DirectCoefficients LM5066H1::coeffPinAvg() const {
  static const CoeffRow kPinAvg1x[] = {
      {100, 4255.5, 6829.0, -3},
      {125, 3404.4, 8133.0, -3},
      {150, 28370.1, 4788.0, -4},
      {175, 24317.2, 5791.0, -4},
      {200, 21277.6, 4418.0, -4},
      {225, 18913.4, 5363.0, -4},
      {250, 17022.1, 5942.0, -4},
      {500, 8511.0, 6672.0, -4},
  };
  static const CoeffRow kPinAvg2x[] = {
      {100, 22979.8, 6834.0, -4},
      {125, 18383.8, 8124.0, -4},
      {150, 15319.9, 4799.0, -4},
      {175, 13131.3, 5796.0, -4},
      {200, 11489.9, 4410.0, -4},
      {225, 10213.2, 5367.0, -4},
      {250, 9191.9, 5934.0, -4},
      {500, 4596.0, 6685.0, -4},
  };
  return coeffFromTable(kPinAvg1x, kPinAvg2x,
                        sizeof(kPinAvg1x) / sizeof(kPinAvg1x[0]));
}

LM5066H1::DirectCoefficients LM5066H1::coeffPout() const {
  static const CoeffRow kPout1x[] = {
      {100, 4255.5, 6690.0, -3},
      {125, 3404.4, 8003.0, -3},
      {150, 28370.1, 4699.0, -4},
      {175, 24317.2, 5764.0, -4},
      {200, 21277.6, 4422.0, -4},
      {225, 18913.4, 5378.0, -4},
      {250, 17022.1, 6005.0, -4},
      {500, 8511.0, 6868.0, -4},
  };
  static const CoeffRow kPout2x[] = {
      {100, 22979.8, 6692.0, -4},
      {125, 18383.8, 7998.0, -4},
      {150, 15319.9, 4711.0, -4},
      {175, 13131.3, 5779.0, -4},
      {200, 11489.9, 4418.0, -4},
      {225, 10213.2, 5387.0, -4},
      {250, 9191.9, 6026.0, -4},
      {500, 4596.0, 6871.0, -4},
  };
  return coeffFromTable(kPout1x, kPout2x, sizeof(kPout1x) / sizeof(kPout1x[0]));
}

LM5066H1::DirectCoefficients LM5066H1::coeffEin() const {
  static const CoeffRow kEin1x[] = {
      {100, 16623.1, 6675.0, -6},
      {125, 13298.5, 8032.0, -6},
      {150, 11082.1, 4669.0, -6},
      {175, 9498.9, 5784.0, -6},
      {200, 8311.6, 4458.0, -6},
      {225, 7388.1, 5356.0, -6},
      {250, 6649.2, 6023.0, -6},
      {500, 3324.6, 6898.0, -6},
  };
  static const CoeffRow kEin2x[] = {
      {100, 8976.5, 6634.0, -6},
      {125, 7181.2, 7945.0, -6},
      {150, 5984.3, 4756.0, -6},
      {175, 5129.4, 5767.0, -6},
      {200, 5129.4, 4487.0, -6},
      {225, 3989.5, 5361.0, -6},
      {250, 3590.6, 6058.0, -6},
      {500, 17953.0, 6843.0, -7},
  };
  return coeffFromTable(kEin1x, kEin2x, sizeof(kEin1x) / sizeof(kEin1x[0]));
}

LM5066H1::DirectCoefficients LM5066H1::coeffFromTable(
    const CoeffRow* oneX,
    const CoeffRow* twoX,
    size_t count) const {
  const CoeffRow* table = _adcFullScale2x ? twoX : oneX;
  int target = static_cast<int>(lround(_currentLimitMilliVolts * 10.0));
  const CoeffRow* best = &table[0];
  int bestDiff = abs(target - table[0].cl_mv_x10);
  for (size_t i = 1; i < count; ++i) {
    int diff = abs(target - table[i].cl_mv_x10);
    if (diff < bestDiff) {
      best = &table[i];
      bestDiff = diff;
    }
    if (diff == 0) {
      break;
    }
  }
  return {best->m_per_mohm * _senseResistorMilliOhms, best->b, best->R};
}

double LM5066H1::directToValue(int32_t raw, const DirectCoefficients& c) const {
  if (c.m == 0.0) {
    return 0.0;
  }
  double scale = pow(10.0, c.R);
  return (static_cast<double>(raw) / scale - c.b) / c.m;
}

bool LM5066H1::valueToDirect(double value,
                             const DirectCoefficients& c,
                             uint16_t& raw,
                             bool clamp12bit) const {
  if (c.m == 0.0) {
    return false;
  }
  double y = (value * c.m + c.b) * pow(10.0, c.R);
  long rounded = lround(y);

  if (clamp12bit) {
    if (rounded < 0) {
      rounded = 0;
    } else if (rounded > 0x0FFF) {
      rounded = 0x0FFF;
    }
  }

  raw = static_cast<uint16_t>(rounded & 0xFFFF);
  return true;
}

bool LM5066H1::sendByte(uint8_t cmd) {
  _bus->beginTransmission(_address);
  _bus->write(cmd);
  return _bus->endTransmission() == 0;
}

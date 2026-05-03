#pragma once

#include <Arduino.h>
#include <Wire.h>

class LM5066H1Bus {
 public:
  virtual ~LM5066H1Bus() = default;

  virtual void begin() = 0;
  virtual void setClock(uint32_t clockHz) = 0;
  virtual void beginTransmission(uint8_t address) = 0;
  virtual size_t write(uint8_t value) = 0;
  virtual uint8_t endTransmission(bool sendStop = true) = 0;
  virtual size_t requestFrom(uint8_t address, uint8_t quantity) = 0;
  virtual int available() = 0;
  virtual int read() = 0;
};

class LM5066H1TwoWireBus final : public LM5066H1Bus {
 public:
  explicit LM5066H1TwoWireBus(TwoWire& wire) : _wire(wire) {}

  void begin() override { _wire.begin(); }
  void setClock(uint32_t clockHz) override { _wire.setClock(clockHz); }
  void beginTransmission(uint8_t address) override {
    _wire.beginTransmission(address);
  }
  size_t write(uint8_t value) override { return _wire.write(value); }
  uint8_t endTransmission(bool sendStop = true) override {
    return _wire.endTransmission(sendStop);
  }
  size_t requestFrom(uint8_t address, uint8_t quantity) override {
    return _wire.requestFrom(address, quantity);
  }
  int available() override { return _wire.available(); }
  int read() override { return _wire.read(); }

#if defined(ARDUINO_ARCH_STM32) || defined(STM32F1xx) || defined(STM32F1)
  void setSDA(uint32_t pin) { _wire.setSDA(pin); }
  void setSCL(uint32_t pin) { _wire.setSCL(pin); }
#endif

 private:
  TwoWire& _wire;
};

class LM5066H1 {
 public:
  enum class RegisterAccess : uint8_t {
    kSendByte = 0U,
    kReadOnly = 1U,
    kWriteOnly = 2U,
    kReadWrite = 3U,
    kBlockRead = 4U,
    kBlockReadWrite = 5U,
  };

  struct RegisterDescriptor {
    uint8_t code;
    const char* name;
    RegisterAccess access;
    uint8_t width;  /* 0=send-byte command, 1=byte, 2=word, 0xFF=PMBus block */
  };

  enum class CurrentLimitSetting {
    kHigh50mV,
    kLow26mV,
  };

  struct DirectCoefficients {
    double m;
    double b;
    int8_t R;
  };

  struct StatusByteBits {
    bool off = false;
    bool vinUvFault = false;
    bool temperature = false;
    bool cml = false;
    bool other = false;
  };

  struct StatusWordBits {
    bool outStatus = false;
    bool inputStatus = false;
    bool fetFail = false;
    bool pgoodb = false;
    bool cbFault = false;
    bool unknown = false;
    StatusByteBits byte = {};
  };

  struct StatusVoutBits {
    bool voutUvWarn = false;
  };

  struct StatusInputBits {
    bool vinOvFault = false;
    bool vinOvWarn = false;
    bool vinUvWarn = false;
    bool vinUvFault = false;
    bool ocFault = false;
    bool ocWarn = false;
    bool inOpWarn = false;
  };

  struct StatusTemperatureBits {
    bool otFault = false;
    bool otWarn = false;
  };

  struct StatusCmlBits {
    bool invCmd = false;
    bool invData = false;
    bool invPec = false;
    bool memoryFault = false;
    bool noneOfAbove = false;
  };

  struct StatusOtherBits {
    bool cbFault = false;
    bool firstSmba = false;
  };

  struct StatusMfrSpecificBits {
    bool cbFault = false;
    bool fetFail = false;
    bool defaultsLoaded = false;
    bool bbRamFull = false;
    bool fetFaultGate2 = false;
    bool fetFaultGate1 = false;
    bool fetFaultDrain = false;
  };

  struct StatusMfrSpecific2Bits {
    bool watchdogFault = false;
    bool scFault = false;
    bool einOfWarn = false;
    bool vinTran = false;
    bool eeProg = false;
    bool avgDone = false;
    bool retryRec = false;
    bool powerCycleRec = false;
    bool initDone = false;
  };

  struct DiagnosticWordBits {
    bool voutUndervoltageWarn = false;
    bool iinOpWarn = false;
    bool vinUndervoltageWarn = false;
    bool vinOvervoltageWarn = false;
    bool powerGood = false;
    bool overTemperatureWarn = false;
    bool timerLatchedOff = false;
    bool fetFail = false;
    bool configPreset = false;
    bool deviceOff = false;
    bool vinUndervoltageFault = false;
    bool vinOvervoltageFault = false;
    bool iinOcPfetOpFault = false;
    bool overTemperatureFault = false;
    bool cmlFault = false;
    bool circuitBreakerFault = false;
  };

  struct GateMaskBits {
    bool scpFault = false;
    bool fetFail = false;
    bool vinUvFault = false;
    bool vinOvFault = false;
    bool iinPfetFault = false;
    bool overtempFault = false;
    bool watchdogFault = false;
    bool circuitBreakerFault = false;
  };

  struct AlertMaskBits {
    bool voutUndervoltageWarn = false;
    bool iinLimitWarn = false;
    bool vinUndervoltageWarn = false;
    bool vinOvervoltageWarn = false;
    bool powerGood = false;
    bool overtempWarn = false;
    bool watchdogFault = false;
    bool overpowerLimitWarn = false;
    bool scpFault = false;
    bool fetFailFault = false;
    bool vinUndervoltageFault = false;
    bool vinOvervoltageFault = false;
    bool iinPfetFault = false;
    bool overtempFault = false;
    bool cmlFault = false;
    bool circuitBreakerFault = false;
  };

  struct BlockReadData {
    uint16_t diagnosticWord = 0;
    uint16_t readIin = 0;
    uint16_t readVout = 0;
    uint16_t readVin = 0;
    uint16_t readPin = 0;
    uint16_t readTemperature = 0;
  };

  struct AvgBlockReadData {
    uint16_t diagnosticWord = 0;
    uint16_t avgIin = 0;
    uint16_t avgVout = 0;
    uint16_t avgVin = 0;
    uint16_t avgPin = 0;
    uint16_t temperature = 0;
  };

  struct EinData {
    int16_t accumulator = 0;
    uint8_t rolloverCount = 0;
    uint32_t sampleCount = 0;
  };

  struct OcBlankingTimersBits {
    uint8_t blanking2 = 0;
    uint8_t blanking1 = 0;
  };

  struct DelayConfigBits {
    uint8_t retryDelay = 0;
    uint8_t insertionDelay = 0;
  };

  struct WdPlbTimerBits {
    uint8_t watchdogTimer = 0;
    uint8_t powerLimitBlanking = 0;
  };

  struct PkMinAvgBits {
    bool resetPeak = false;
    bool resetAvg = false;
    bool resetMin = false;
  };

  struct P2tTimerBits {
    bool absTempMode = false;
    uint8_t maxDuration = 0;
  };

  struct BbConfigBits {
    bool fetOffWr = false;
    bool fltWr = false;
    bool alertWr = false;
    uint8_t bbTick = 0;
  };

  struct BbRamEntry {
    uint8_t eventId = 0;
    bool timerExpired = false;
    uint8_t tick = 0;
  };

  struct AdcConfig1Bits {
    bool eoc = false;
    bool convst = false;
    uint8_t mode = 0;
    uint8_t convChSel = 0;
  };

  struct AdcConfig2Bits {
    bool fullScale = false;
    uint8_t bufChSel = 0;
    uint8_t decRate = 0;
  };

  struct DeviceSetup1Bits {
    uint8_t retrySetting = 0;
    bool currentLimitSetting = false;
    bool cbClRatio = false;
    bool currentLimitConfig = false;
    bool permanentWriteDisable = false;
  };

  struct DeviceSetup2Bits {
    uint8_t cbClRatio2 = 0;
    uint8_t currentLimitSetting2 = 0;
    bool fastRecoveryCb = false;
    bool vinTranEnable = false;
  };

  struct DeviceSetup3Bits {
    bool currentLimitingDisable = false;
    bool powerLimitProfile = false;
    uint8_t foldbackCurrentLimit = 0;
    uint8_t ocBlanking2Threshold = 0;
    uint8_t ocBlanking1Threshold = 0;
  };

  struct DeviceSetup4Bits {
    bool immediateRetry = false;
    bool regulationTimerDischarge = false;
    bool syncPinFunction = false;
    bool sftSrt = false;
    uint8_t regulationTimerSetting = 0;
    uint8_t powerLimitBlankingVds = 0;
  };

  struct DeviceSetup5Bits {
    bool gate1OvercurrentPullDown = false;
    bool gate2OvercurrentPullDown = false;
    bool gate12OvertempPullDown = false;
    bool gate12OvloPullDown = false;
    bool gate12UvloPullDown = false;
  };

  struct IinOffsetCalibrationBits {
    bool negative = false;
    uint8_t offsetFactor = 0;
  };

  struct BbTimerBits {
    uint8_t bbPtr = 0;
    bool timerExpired = false;
    uint8_t bbTick = 0;
  };

  struct NtcConfig {
    double pullupOhms = 10000.0;
    double nominalOhms = 10000.0;
    double nominalTempC = 25.0;
    double beta = 3950.0;
    double supplyVolts = 3.3;
  };

  static constexpr uint8_t kDefaultAddress = 0x40;

  explicit LM5066H1(uint8_t address = kDefaultAddress, TwoWire& wire = Wire);
  LM5066H1(uint8_t address, LM5066H1Bus& bus);

  /**
   *  ESP32-style begin: explicitly pass SDA / SCL pins.
   *  Use sdaPin = -1, sclPin = -1 to skip pin assignment (STM32 default pins).
   */
  bool begin(int sdaPin = -1, int sclPin = -1, uint32_t clockHz = 100000);

  /**
   *  STM32-friendly overload: the bus has already been configured
   *  by the BSP; this call only sets the clock rate and probes the device.
   */
  bool beginAttached(uint32_t clockHz = 100000);

  bool isPresent();

  void setAddress(uint8_t address);
  uint8_t address() const;

  void setSenseResistorMilliOhms(double rs_mohm);
  double senseResistorMilliOhms() const;

  void setCurrentLimitSetting(CurrentLimitSetting setting);
  CurrentLimitSetting currentLimitSetting() const;

  void setCurrentLimitMilliVolts(double millivolts);
  double currentLimitMilliVolts() const;

  void setAdcFullScale2x(bool enable);
  bool adcFullScale2x() const;

  void setNtcConfig(const NtcConfig& config);
  NtcConfig ntcConfig() const;

  bool writeByte(uint8_t reg, uint8_t value);
  bool writeWord(uint8_t reg, uint16_t value);
  bool writeBlock(uint8_t reg, const uint8_t* data, size_t len);
  bool writeBlockCommand(uint8_t cmd, const uint8_t* data, size_t len);

  bool readByte(uint8_t reg, uint8_t& value);
  bool readWord(uint8_t reg, uint16_t& value);
  bool readBlock(uint8_t reg, uint8_t* data, size_t len);
  bool readBlockCommand(uint8_t cmd, uint8_t* data, size_t maxLen,
                        size_t& outLen);
  bool sendCommand(uint8_t cmd);
  bool readRegisterRaw(uint8_t cmd, uint8_t* data, size_t maxLen,
                       size_t& outLen);
  bool writeRegisterRaw(uint8_t cmd, const uint8_t* data, size_t len);

  bool writeOperationRaw(uint8_t value);
  bool readOperationRaw(uint8_t& value);
  bool readOperation(bool& on);
  bool setOperation(bool on);
  bool clearFaults();
  bool setWriteProtect(uint8_t value);
  bool readWriteProtect(uint8_t& value);
  bool restoreFactoryDefaults();
  bool storeUserAll();
  bool restoreUserAll();
  bool powerCycle();

  bool readCapability(uint8_t& value);
  bool readPmbusRevision(uint8_t& value);

  bool readStatusByte(uint8_t& value);
  bool readStatusByte(StatusByteBits& status);
  bool readStatusWord(uint16_t& value);
  bool readStatusWord(StatusWordBits& status);
  bool readStatusVout(uint8_t& value);
  bool readStatusVout(StatusVoutBits& status);
  bool readStatusInput(uint8_t& value);
  bool readStatusInput(StatusInputBits& status);
  bool readStatusTemperature(uint8_t& value);
  bool readStatusTemperature(StatusTemperatureBits& status);
  bool readStatusCml(uint8_t& value);
  bool readStatusCml(StatusCmlBits& status);
  bool readStatusOther(uint8_t& value);
  bool readStatusOther(StatusOtherBits& status);
  bool readStatusMfrSpecific(uint8_t& value);
  bool readStatusMfrSpecific(StatusMfrSpecificBits& status);
  bool readStatusMfrSpecific2(uint16_t& value);
  bool readStatusMfrSpecific2(StatusMfrSpecific2Bits& status);
  bool readDiagnosticWord(uint16_t& value);
  bool readDiagnosticWord(DiagnosticWordBits& status);

  bool readEinRaw(uint8_t* data, size_t len, size_t& outLen);
  bool readEin(EinData& data);
  bool readVinRaw(uint16_t& value);
  bool readVoutRaw(uint16_t& value);
  bool readVauxRaw(uint16_t& value);
  bool readVauxAvgRaw(uint16_t& value);
  bool readTemperatureRaw(int16_t& value);
  bool readTempAvgRaw(uint16_t& value);
  bool readTempPeakRaw(uint16_t& value);

  bool readIinRaw(uint16_t& value);
  bool readIinStdRaw(uint16_t& value);
  bool readIinMfrRaw(uint16_t& value);
  bool readIoutRaw(uint16_t& value);
  bool readPinRaw(uint16_t& value);
  bool readPinStdRaw(uint16_t& value);
  bool readPinMfrRaw(uint16_t& value);
  bool readPoutRaw(uint16_t& value);
  bool readVinMinRaw(uint16_t& value);
  bool readVinPeakRaw(uint16_t& value);
  bool readIinPeakRaw(uint16_t& value);
  bool readPinPeakRaw(uint16_t& value);
  bool readPinPeakStdRaw(uint16_t& value);
  bool readPinPeakMfrRaw(uint16_t& value);
  bool readVoutMinRaw(uint16_t& value);
  bool clearPinPeak();

  bool readAvgVinRaw(uint16_t& value);
  bool readAvgVoutRaw(uint16_t& value);
  bool readAvgIinRaw(uint16_t& value);
  bool readAvgPinRaw(uint16_t& value);

  bool readIinStd(double& amps);
  bool readIout(double& amps);
  bool readVin(double& volts);
  bool readVout(double& volts);
  bool readVaux(double& volts);
  bool readVauxAvg(double& volts);
  bool readNtcTemperatureC(double& celsius);
  bool readTemperatureC(double& celsius);
  bool readTempAvgC(double& celsius);
  bool readTempPeakC(double& celsius);
  bool readIin(double& amps);
  bool readPin(double& watts);
  bool readPinStd(double& watts);
  bool readPout(double& watts);
  bool readVinMin(double& volts);
  bool readVinPeak(double& volts);
  bool readIinPeak(double& amps);
  bool readPinPeak(double& watts);
  bool readPinPeakStd(double& watts);
  bool readVoutMin(double& volts);

  bool readAvgVin(double& volts);
  bool readAvgVout(double& volts);
  bool readAvgIin(double& amps);
  bool readAvgPin(double& watts);

  bool writeIinOcWarnLimitRaw(uint16_t value);
  bool writeIinOcWarnLimitStdRaw(uint16_t value);
  bool writeIinOcWarnLimitMfrRaw(uint16_t value);
  bool writeOcWarnLimitRaw(uint16_t value);
  bool writeVinOvWarnLimitRaw(uint16_t value);
  bool writeVinUvWarnLimitRaw(uint16_t value);
  bool writeVoutUvWarnLimitRaw(uint16_t value);
  bool writeOtWarnLimitRaw(uint16_t value);
  bool writeOtFaultLimitRaw(uint16_t value);
  bool writePinOpWarnLimitRaw(uint16_t value);

  bool readIinOcWarnLimitRaw(uint16_t& value);
  bool readIinOcWarnLimitStdRaw(uint16_t& value);
  bool readIinOcWarnLimitMfrRaw(uint16_t& value);
  bool readOcWarnLimitRaw(uint16_t& value);
  bool readVinOvWarnLimitRaw(uint16_t& value);
  bool readVinUvWarnLimitRaw(uint16_t& value);
  bool readVoutUvWarnLimitRaw(uint16_t& value);
  bool readOtWarnLimitRaw(uint16_t& value);
  bool readOtFaultLimitRaw(uint16_t& value);
  bool readPinOpWarnLimitRaw(uint16_t& value);

  bool setIinOcWarnLimitStd(double amps);
  bool setOcWarnLimit(double amps);
  bool setVinOvWarnLimit(double volts);
  bool setVinUvWarnLimit(double volts);
  bool setVoutUvWarnLimit(double volts);
  bool setOtWarnLimit(double celsius);
  bool setOtFaultLimit(double celsius);
  bool setIinOcWarnLimit(double amps);
  bool setIinOcWarnLimitMfr(double amps);
  bool setPinOpWarnLimit(double watts);

  bool readIinOcWarnLimitStd(double& amps);
  bool readOcWarnLimit(double& amps);
  bool readVinOvWarnLimit(double& volts);
  bool readVinUvWarnLimit(double& volts);
  bool readVoutUvWarnLimit(double& volts);
  bool readOtWarnLimit(double& celsius);
  bool readOtFaultLimit(double& celsius);
  bool readIinOcWarnLimit(double& amps);
  bool readIinOcWarnLimitMfr(double& amps);
  bool readPinOpWarnLimit(double& watts);

  bool setAlertMask(uint16_t mask);
  bool setAlertMask(const AlertMaskBits& mask);
  bool readAlertMask(uint16_t& mask);
  bool readAlertMask(AlertMaskBits& mask);

  bool setGateMask(uint8_t mask);
  bool setGateMask(const GateMaskBits& mask);
  bool readGateMask(uint8_t& mask);
  bool readGateMask(GateMaskBits& mask);

  bool setDeviceSetup1(uint8_t value);
  bool readDeviceSetup1(uint8_t& value);
  bool setDeviceSetup1(const DeviceSetup1Bits& config);
  bool readDeviceSetup1(DeviceSetup1Bits& config);

  bool setDeviceSetup2(uint8_t value);
  bool readDeviceSetup2(uint8_t& value);
  bool setDeviceSetup2(const DeviceSetup2Bits& config);
  bool readDeviceSetup2(DeviceSetup2Bits& config);

  bool setDeviceSetup3(uint8_t value);
  bool readDeviceSetup3(uint8_t& value);
  bool setDeviceSetup3(const DeviceSetup3Bits& config);
  bool readDeviceSetup3(DeviceSetup3Bits& config);

  bool setDeviceSetup4(uint8_t value);
  bool readDeviceSetup4(uint8_t& value);
  bool setDeviceSetup4(const DeviceSetup4Bits& config);
  bool readDeviceSetup4(DeviceSetup4Bits& config);

  bool setDeviceSetup5(uint8_t value);
  bool readDeviceSetup5(uint8_t& value);
  bool setDeviceSetup5(const DeviceSetup5Bits& config);
  bool readDeviceSetup5(DeviceSetup5Bits& config);

  bool setOcBlankingTimers(uint8_t value);
  bool readOcBlankingTimers(uint8_t& value);
  bool setOcBlankingTimers(const OcBlankingTimersBits& config);
  bool readOcBlankingTimers(OcBlankingTimersBits& config);

  bool setDelayConfig(uint8_t value);
  bool readDelayConfig(uint8_t& value);
  bool setDelayConfig(const DelayConfigBits& config);
  bool readDelayConfig(DelayConfigBits& config);

  bool setWdPlbTimer(uint8_t value);
  bool readWdPlbTimer(uint8_t& value);
  bool setWdPlbTimer(const WdPlbTimerBits& config);
  bool readWdPlbTimer(WdPlbTimerBits& config);

  bool setPkMinAvg(uint8_t value);
  bool readPkMinAvg(uint8_t& value);
  bool setPkMinAvg(const PkMinAvgBits& config);
  bool readPkMinAvg(PkMinAvgBits& config);

  bool setP2tTimer(uint8_t value);
  bool readP2tTimer(uint8_t& value);
  bool setP2tTimer(const P2tTimerBits& config);
  bool readP2tTimer(P2tTimerBits& config);

  bool setBbConfig(uint8_t value);
  bool readBbConfig(uint8_t& value);
  bool setBbConfig(const BbConfigBits& config);
  bool readBbConfig(BbConfigBits& config);

  bool readBbTimer(uint8_t& value);
  bool readBbTimer(BbTimerBits& config);

  bool setAdcConfig1(uint8_t value);
  bool readAdcConfig1(uint8_t& value);
  bool setAdcConfig1(const AdcConfig1Bits& config);
  bool readAdcConfig1(AdcConfig1Bits& config);

  bool setAdcConfig2(uint8_t value);
  bool readAdcConfig2(uint8_t& value);
  bool setAdcConfig2(const AdcConfig2Bits& config);
  bool readAdcConfig2(AdcConfig2Bits& config);

  bool setIinOffsetCalibration(uint8_t value);
  bool readIinOffsetCalibration(uint8_t& value);
  bool setIinOffsetCalibration(const IinOffsetCalibrationBits& config);
  bool readIinOffsetCalibration(IinOffsetCalibrationBits& config);

  bool setPmbusAddress(uint8_t value);
  bool readPmbusAddress(uint8_t& value);

  bool setUserData(uint8_t value);
  bool readUserData(uint8_t& value);

  bool setSamplesForAvg(uint8_t value);
  bool readSamplesForAvg(uint8_t& value);

  bool readBlockRead(uint8_t* data, size_t len, size_t& outLen);
  bool readBlockRead(BlockReadData& data);
  bool readAvgBlockRead(uint8_t* data, size_t len, size_t& outLen);
  bool readAvgBlockRead(AvgBlockReadData& data);

  bool bbClear();
  bool bbErase();
  bool fetchBbEeprom();
  bool readBlackBoxRead(uint8_t* data, size_t len, size_t& outLen);
  bool readSampleBuffer(uint8_t* data, size_t len, size_t& outLen);
  bool readBbRam(uint8_t* data, size_t len, size_t& outLen);
  bool readBbRam(BbRamEntry& entry);
  bool readBbEeprom(uint8_t* data, size_t len, size_t& outLen);

  bool readMfrId(char* out, size_t maxLen);
  bool readMfrModel(char* out, size_t maxLen);
  bool readMfrRevision(char* out, size_t maxLen);

  /** Generic PMBus bridge primitives for the STM32 external-control API.
   *  These intentionally expose raw command bytes so a host can configure
   *  EEPROM/user registers without needing a firmware update for every
   *  LM5066H1 command. */
  bool bridgeRead(uint8_t cmd, uint8_t* data, size_t len, size_t& outLen);
  bool bridgeWrite(uint8_t cmd, const uint8_t* data, size_t len);

  static size_t registerDescriptorCount();
  static const RegisterDescriptor& registerDescriptor(size_t index);
  static const RegisterDescriptor* findRegisterDescriptor(uint8_t cmd);
  static const char* commandName(uint8_t cmd);

  static double ocBlankingTimerMs(uint8_t nibble, bool longRange);
  static double ocBlankingTimer2Ms(uint8_t nibble);
  static double ocBlankingTimer1Ms(uint8_t nibble);
  static double retryDelaySeconds(uint8_t nibble);
  static double insertionDelayMs(uint8_t nibble);
  static double watchdogTimerMs(uint8_t nibble);
  static double powerLimitBlankingMs(uint8_t nibble);
  static double p2tMaxDurationUs(uint8_t nibble);
  static double bbTickMicroseconds(uint8_t bbTick);
  static uint16_t samplesForAvgCount(uint8_t exponent);
  static double samplesForAvgPeriodMs(uint8_t exponent);
  static const char* bbRamEventName(uint8_t eventId);

 private:
  struct CoeffRow {
    int cl_mv_x10;
    double m_per_mohm;
    double b;
    int8_t R;
  };

  DirectCoefficients coeffVin() const;
  DirectCoefficients coeffVinAvg() const;
  DirectCoefficients coeffVout() const;
  DirectCoefficients coeffVoutAvg() const;
  DirectCoefficients coeffVaux() const;
  DirectCoefficients coeffTemperature() const;
  DirectCoefficients coeffTemperatureAvg() const;
  DirectCoefficients coeffIin() const;
  DirectCoefficients coeffIinAvg() const;
  DirectCoefficients coeffIout() const;
  DirectCoefficients coeffPin() const;
  DirectCoefficients coeffPinAvg() const;
  DirectCoefficients coeffPout() const;
  DirectCoefficients coeffEin() const;

  DirectCoefficients coeffFromTable(const CoeffRow* oneX,
                                    const CoeffRow* twoX,
                                    size_t count) const;

  double directToValue(int32_t raw, const DirectCoefficients& c) const;
  bool valueToDirect(double value, const DirectCoefficients& c,
                     uint16_t& raw, bool clamp12bit) const;

  bool sendByte(uint8_t cmd);

  uint8_t _address;
  LM5066H1TwoWireBus _wireBus;
  LM5066H1Bus* _bus;
  double _senseResistorMilliOhms;
  CurrentLimitSetting _currentLimitSetting;
  double _currentLimitMilliVolts;
  bool _adcFullScale2x;
  double _ntcPullupOhms;
  double _ntcNominalOhms;
  double _ntcNominalTempC;
  double _ntcBeta;
  double _ntcSupplyVolts;

  static constexpr uint8_t kCmdOperation = 0x01;
  static constexpr uint8_t kCmdClearFaults = 0x03;
  static constexpr uint8_t kCmdWriteProtect = 0x10;
  static constexpr uint8_t kCmdRestoreFactoryDefaults = 0x12;
  static constexpr uint8_t kCmdStoreUserAll = 0x15;
  static constexpr uint8_t kCmdRestoreUserAll = 0x16;
  static constexpr uint8_t kCmdCapability = 0x19;
  static constexpr uint8_t kCmdVoutUvWarnLimit = 0x43;
  static constexpr uint8_t kCmdOtFaultLimit = 0x4F;
  static constexpr uint8_t kCmdOtWarnLimit = 0x51;
  static constexpr uint8_t kCmdVinOvWarnLimit = 0x57;
  static constexpr uint8_t kCmdVinUvWarnLimit = 0x58;
  static constexpr uint8_t kCmdIinOcWarnLimitStd = 0x5D;
  static constexpr uint8_t kCmdStatusByte = 0x78;
  static constexpr uint8_t kCmdStatusWord = 0x79;
  static constexpr uint8_t kCmdStatusVout = 0x7A;
  static constexpr uint8_t kCmdStatusInput = 0x7C;
  static constexpr uint8_t kCmdStatusTemperature = 0x7D;
  static constexpr uint8_t kCmdStatusCml = 0x7E;
  static constexpr uint8_t kCmdStatusOther = 0x7F;
  static constexpr uint8_t kCmdStatusMfrSpecific = 0x80;
  static constexpr uint8_t kCmdReadEin = 0x86;
  static constexpr uint8_t kCmdReadVin = 0x88;
  static constexpr uint8_t kCmdReadIinStd = 0x89;
  static constexpr uint8_t kCmdReadVout = 0x8B;
  static constexpr uint8_t kCmdReadIout = 0x8C;
  static constexpr uint8_t kCmdReadTemperature = 0x8D;
  static constexpr uint8_t kCmdReadPout = 0x96;
  static constexpr uint8_t kCmdReadPinStd = 0x97;
  static constexpr uint8_t kCmdPmbusRevision = 0x98;
  static constexpr uint8_t kCmdMfrId = 0x99;
  static constexpr uint8_t kCmdMfrModel = 0x9A;
  static constexpr uint8_t kCmdMfrRevision = 0x9B;
  static constexpr uint8_t kCmdReadVinMin = 0xA0;
  static constexpr uint8_t kCmdReadVinPeak = 0xA1;
  static constexpr uint8_t kCmdReadIinPeak = 0xA2;
  static constexpr uint8_t kCmdReadPinPeakStd = 0xA3;
  static constexpr uint8_t kCmdReadVoutMin = 0xA4;
  static constexpr uint8_t kCmdUserData = 0xBC;
  static constexpr uint8_t kCmdReadTempAvg = 0xC7;
  static constexpr uint8_t kCmdReadTempPeak = 0xC8;
  static constexpr uint8_t kCmdReadSampleBuf = 0xC9;
  static constexpr uint8_t kCmdPowerCycle = 0xCA;
  static constexpr uint8_t kCmdDeviceSetup1 = 0xCC;
  static constexpr uint8_t kCmdDeviceSetup4 = 0xCD;
  static constexpr uint8_t kCmdDeviceSetup5 = 0xCE;
  static constexpr uint8_t kCmdReadVaux = 0xD0;
  static constexpr uint8_t kCmdReadIin = 0xD1;
  static constexpr uint8_t kCmdReadPin = 0xD2;
  static constexpr uint8_t kCmdIinOcWarnLimit = 0xD3;
  static constexpr uint8_t kCmdPinOpWarnLimit = 0xD4;
  static constexpr uint8_t kCmdReadPinPeak = 0xD5;
  static constexpr uint8_t kCmdClearPinPeak = 0xD6;
  static constexpr uint8_t kCmdGateMask = 0xD7;
  static constexpr uint8_t kCmdAlertMask = 0xD8;
  static constexpr uint8_t kCmdReadVauxAvg = 0xD9;
  static constexpr uint8_t kCmdBlockRead = 0xDA;
  static constexpr uint8_t kCmdSamplesForAvg = 0xDB;
  static constexpr uint8_t kCmdReadAvgVin = 0xDC;
  static constexpr uint8_t kCmdReadAvgVout = 0xDD;
  static constexpr uint8_t kCmdReadAvgIin = 0xDE;
  static constexpr uint8_t kCmdReadAvgPin = 0xDF;
  static constexpr uint8_t kCmdBbClear = 0xE0;
  static constexpr uint8_t kCmdDiagnosticWord = 0xE1;
  static constexpr uint8_t kCmdAvgBlockRead = 0xE2;
  static constexpr uint8_t kCmdBbErase = 0xE3;
  static constexpr uint8_t kCmdBbConfig = 0xE4;
  static constexpr uint8_t kCmdOcBlankingTimers = 0xE5;
  static constexpr uint8_t kCmdDelayConfig = 0xE7;
  static constexpr uint8_t kCmdWdPlbTimer = 0xE8;
  static constexpr uint8_t kCmdPkMinAvg = 0xE9;
  static constexpr uint8_t kCmdP2tTimer = 0xEA;
  static constexpr uint8_t kCmdFetchBbEeprom = 0xEB;
  static constexpr uint8_t kCmdReadBbRam = 0xEC;
  static constexpr uint8_t kCmdAdcConfig1 = 0xED;
  static constexpr uint8_t kCmdAdcConfig2 = 0xEE;
  static constexpr uint8_t kCmdDeviceSetup2 = 0xEF;
  static constexpr uint8_t kCmdDeviceSetup3 = 0xF0;
  static constexpr uint8_t kCmdIinOffsetCalibration = 0xF2;
  static constexpr uint8_t kCmdStatusMfrSpecific2 = 0xF3;
  static constexpr uint8_t kCmdReadBbEeprom = 0xF4;
  static constexpr uint8_t kCmdBbTimer = 0xF6;
  static constexpr uint8_t kCmdPmbusAddr = 0xF7;
  static constexpr uint8_t kCmdOcWarnLimit = 0xF8;

  static constexpr uint8_t kOperationOn = 0x80;
  static constexpr uint8_t kOperationOff = 0x00;
};

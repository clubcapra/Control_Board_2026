/* =============================================================================
 *  fault_log.cpp - Reset-persistent fault history implementation.
 * =============================================================================
 */
#include "fault_log.h"

#include <Arduino.h>

#if defined(STM32F1xx) || defined(STM32F1)
#  include "stm32f1xx.h"
#  include "stm32f1xx_hal.h"
#endif

namespace pdu {
namespace fault_log {

namespace {

constexpr uint16_t kMagic = 0x464CU;  /* "FL" */
constexpr uint32_t kFlashPageAddress = 0x0800FC00UL;
constexpr uint32_t kFlashPageSize    = 1024UL;
constexpr size_t   kMaxRecords       = 24U;

struct StoredRecord {
  uint16_t magic;
  uint16_t sequence;
  uint8_t  code;
  uint8_t  rail;
  uint16_t status_word;
  uint16_t diag_word;
  uint16_t reset_count;
  uint32_t fault_uptime_ms;
  uint32_t unix_time_s;
  uint32_t reset_flags;
  uint16_t checksum;
} __attribute__((packed, aligned(2)));

static_assert((sizeof(StoredRecord) % 2U) == 0U, "flash writes are halfword based");

Record s_records[kMaxRecords] = {};
size_t s_count = 0U;
uint16_t s_next_sequence = 1U;
uint16_t s_reset_count = 0U;
uint32_t s_boot_reset_flags = 0U;
uint32_t s_time_base_unix_s = 0U;
uint32_t s_time_base_ms = 0U;

uint16_t fold32Low(uint32_t value) {
  return static_cast<uint16_t>(value & 0xFFFFU);
}

uint16_t fold32High(uint32_t value) {
  return static_cast<uint16_t>((value >> 16U) & 0xFFFFU);
}

uint16_t checksum(const StoredRecord& r) {
  uint16_t c = kMagic;
  c ^= r.magic;
  c ^= r.sequence;
  c ^= (static_cast<uint16_t>(r.code) << 8U) | r.rail;
  c ^= r.status_word;
  c ^= r.diag_word;
  c ^= r.reset_count;
  c ^= fold32Low(r.fault_uptime_ms);
  c ^= fold32High(r.fault_uptime_ms);
  c ^= fold32Low(r.unix_time_s);
  c ^= fold32High(r.unix_time_s);
  c ^= fold32Low(r.reset_flags);
  c ^= fold32High(r.reset_flags);
  return c;
}

uint32_t currentResetFlags() {
#if defined(STM32F1xx) || defined(STM32F1)
  return RCC->CSR;
#else
  return 0U;
#endif
}

void clearResetFlags() {
#if defined(STM32F1xx) || defined(STM32F1)
  RCC->CSR |= RCC_CSR_RMVF;
#endif
}

uint32_t currentUnixTime() {
  if (s_time_base_unix_s == 0U) {
    return 0U;
  }
  return s_time_base_unix_s + ((millis() - s_time_base_ms) / 1000UL);
}

Record toRecord(const StoredRecord& sr) {
  Record r = {};
  r.valid = true;
  r.sequence = sr.sequence;
  r.code = static_cast<Code>(sr.code);
  r.rail = static_cast<Rail>(sr.rail);
  r.status_word = sr.status_word;
  r.diag_word = sr.diag_word;
  r.fault_uptime_ms = sr.fault_uptime_ms;
  r.unix_time_s = sr.unix_time_s;
  r.reset_flags = sr.reset_flags;
  r.reset_count = sr.reset_count;
  return r;
}

StoredRecord toStored(const Record& r) {
  StoredRecord sr = {};
  sr.magic = kMagic;
  sr.sequence = r.sequence;
  sr.code = static_cast<uint8_t>(r.code);
  sr.rail = static_cast<uint8_t>(r.rail);
  sr.status_word = r.status_word;
  sr.diag_word = r.diag_word;
  sr.reset_count = r.reset_count;
  sr.fault_uptime_ms = r.fault_uptime_ms;
  sr.unix_time_s = r.unix_time_s;
  sr.reset_flags = r.reset_flags;
  sr.checksum = checksum(sr);
  return sr;
}

const StoredRecord* flashRecord(size_t index) {
  return reinterpret_cast<const StoredRecord*>(
      kFlashPageAddress + (index * sizeof(StoredRecord)));
}

bool isErased(const StoredRecord& sr) {
  const uint16_t* p = reinterpret_cast<const uint16_t*>(&sr);
  for (size_t i = 0U; i < (sizeof(StoredRecord) / sizeof(uint16_t)); ++i) {
    if (p[i] != 0xFFFFU) {
      return false;
    }
  }
  return true;
}

bool isValid(const StoredRecord& sr) {
  return sr.magic == kMagic && sr.checksum == checksum(sr);
}

bool flashErasePage() {
#if defined(STM32F1xx) || defined(STM32F1)
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef erase = {};
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = kFlashPageAddress;
  erase.NbPages = 1U;
  uint32_t page_error = 0U;
  const bool ok = (HAL_FLASHEx_Erase(&erase, &page_error) == HAL_OK);
  HAL_FLASH_Lock();
  return ok;
#else
  return true;
#endif
}

bool flashAppend(const StoredRecord& sr) {
#if defined(STM32F1xx) || defined(STM32F1)
  size_t slot = kMaxRecords;
  for (size_t i = 0U; i < kMaxRecords; ++i) {
    const StoredRecord candidate = *flashRecord(i);
    if (isErased(candidate)) {
      slot = i;
      break;
    }
  }
  if (slot >= kMaxRecords) {
    return false;
  }

  const uint16_t* words = reinterpret_cast<const uint16_t*>(&sr);
  uint32_t address = kFlashPageAddress + (slot * sizeof(StoredRecord));
  HAL_FLASH_Unlock();
  bool ok = true;
  for (size_t i = 0U; i < (sizeof(StoredRecord) / sizeof(uint16_t)); ++i) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, words[i]) != HAL_OK) {
      ok = false;
      break;
    }
    address += 2U;
  }
  HAL_FLASH_Lock();
  return ok;
#else
  (void)sr;
  return true;
#endif
}

void loadFromFlash() {
  s_count = 0U;
  s_next_sequence = 1U;
  for (size_t i = 0U; i < kMaxRecords; ++i) {
    const StoredRecord sr = *flashRecord(i);
    if (isErased(sr)) {
      break;
    }
    if (isValid(sr) && s_count < kMaxRecords) {
      s_records[s_count] = toRecord(sr);
      if (s_records[s_count].sequence >= s_next_sequence) {
        s_next_sequence = static_cast<uint16_t>(s_records[s_count].sequence + 1U);
      }
      ++s_count;
    }
  }
}

void rewriteAll() {
  (void)flashErasePage();
  for (size_t i = 0U; i < s_count; ++i) {
    (void)flashAppend(toStored(s_records[i]));
  }
}

void appendRecord(const Record& r) {
  if (s_count < kMaxRecords) {
    s_records[s_count] = r;
    ++s_count;
    if (!flashAppend(toStored(r))) {
      rewriteAll();
    }
    return;
  }

  /* FIFO full: drop oldest, keep latest kMaxRecords-1, then append new one. */
  for (size_t i = 1U; i < kMaxRecords; ++i) {
    s_records[i - 1U] = s_records[i];
  }
  s_records[kMaxRecords - 1U] = r;
  rewriteAll();
}

}  // namespace

Status init() {
  s_boot_reset_flags = currentResetFlags();
  clearResetFlags();
  loadFromFlash();
  if (s_count > 0U) {
    s_reset_count = s_records[s_count - 1U].reset_count;
  }
  if (s_reset_count != 0xFFFFU) {
    ++s_reset_count;
  }
  return Status::kOk;
}

void setUnixTime(uint32_t unix_time_s) {
  s_time_base_unix_s = unix_time_s;
  s_time_base_ms = millis();
}

Record last() {
  if (s_count == 0U) {
    return Record{};
  }
  return s_records[s_count - 1U];
}

size_t count() {
  return s_count;
}

size_t capacity() {
  return kMaxRecords;
}

size_t copyLatest(Record* out, size_t max_records) {
  if (out == nullptr || max_records == 0U) {
    return 0U;
  }
  const size_t n = (s_count < max_records) ? s_count : max_records;
  const size_t start = s_count - n;
  for (size_t i = 0U; i < n; ++i) {
    out[i] = s_records[start + i];
  }
  return n;
}

uint32_t bootResetFlags() {
  return s_boot_reset_flags;
}

uint16_t resetCount() {
  return s_reset_count;
}

void clear() {
  s_count = 0U;
  s_next_sequence = 1U;
  (void)flashErasePage();
}

void record(Code code, Rail rail, uint16_t status_word, uint16_t diag_word) {
  Record r = {};
  r.valid = true;
  r.sequence = s_next_sequence;
  if (s_next_sequence != 0xFFFFU) {
    ++s_next_sequence;
  }
  r.code = code;
  r.rail = rail;
  r.status_word = status_word;
  r.diag_word = diag_word;
  r.fault_uptime_ms = millis();
  r.unix_time_s = currentUnixTime();
  r.reset_flags = s_boot_reset_flags;
  r.reset_count = s_reset_count;
  appendRecord(r);
}

void recordAndReset(Code code, Rail rail, uint16_t status_word, uint16_t diag_word) {
  record(code, rail, status_word, diag_word);
  delay(10);
  NVIC_SystemReset();
}

}  /* namespace fault_log */
}  /* namespace pdu */

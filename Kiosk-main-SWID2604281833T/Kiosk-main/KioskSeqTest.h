#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <U8x8lib.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

namespace KioskSeqTest {

struct Config {
  U8X8* oled = nullptr;
  hd44780_I2Cexp* lcd = nullptr;
  uint8_t hwid = 0;
  const char* swid = nullptr;
};

// Runs the IO Sequential Test mode. Never returns (exit via reset).
[[noreturn]] void run(const Config& cfg);

} // namespace KioskSeqTest

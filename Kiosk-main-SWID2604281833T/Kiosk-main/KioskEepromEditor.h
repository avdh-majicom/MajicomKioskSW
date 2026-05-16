/*
  Refactor note (structure, not behaviour):
  - KioskEepromEditor.h/.cpp form a self-contained EEPROM editor module.
  - Host firmware provides:
      * initialisation for Wire / displays / PN532 reset pin
      * optional HWID->text mapping via weak symbol KioskFormatHwIdText()
      * SWID metadata via KIOSK_SWID
      * the decision logic that determines when to enter the editor at boot
  - The editor never returns; it exits via watchdog reset (long BACK or explicit reset paths).
*/

// KioskEepromEditor.h
// ---------------------
// EEPROM editor module for the kiosk firmware.
// - Intended to be called after boot if a button chord is held.
// - Does not return (exit via reset/watchdog).
//
// This header is intentionally self-contained (includes the concrete types)
// because KioskEepromEditor.cpp uses nested EEPROM types and display/NFC APIs.
//
// NOTE: hd44780_I2Cexp requires Wire to be declared; include <Wire.h> first.

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include <U8x8lib.h>

#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

#include <Adafruit_PN532.h>

#include "KioskIOpins.h"   // canonical kiosk pin naming
#include "KioskBuildInfo.h"

#include "KioskEeprom.h"   // Must provide Kiosk::KioskEeprom and its nested types

namespace Kiosk {
namespace KioskEepromEditor {

// Build/version metadata should be defined by the host firmware (preferred).
// These defaults only apply if the host does not define them.
#ifndef KIOSK_SWID
#define KIOSK_SWID "SWID:0000000000T"
#endif

// Configuration for running the EEPROM editor module.
// Assumptions:
// - Buttons are active-low with external pullups and RC debouncing.
// - PN532 must be held in RESET when not actively scanning to avoid I2C bus issues.
struct Config {
  // Required modules
  KioskEeprom*    ee   = nullptr;   // EEPROM manager module
  U8X8*             oled = nullptr;   // OLED in U8x8 text mode (16x16 chars)
  hd44780_I2Cexp*   lcd  = nullptr;   // 16x4 I2C LCD

  // Optional modules
  Adafruit_PN532*   nfc  = nullptr;   // PN532 NFC reader (optional)

  // Button pins (active-low)
  uint8_t pinUp     = KioskPins::BTN_CONT_CIRC;    // D13
  uint8_t pinDown   = KioskPins::BTN_WATER_INLET;  // D12
  uint8_t pinSelect = KioskPins::BTN_SENSOR_BYPASS;    // A12 (D66)
  uint8_t pinBack   = KioskPins::BTN_BACKWASH_CTL;    // A13 (D67)

  // Optional “reset chord” guard pin (active-low). Use 255 if unused.
  uint8_t pinReinitGuard = 255;

  // PN532 reset pin (active-low reset line on PN532 module). Use 255 if no PN532.
  uint8_t pinPn532Reset  = KioskPins::PN532_RESET;   // D45

  // If false: NFC token table is read-only (browse only; no add/delete/remove-all).
  bool allowTokenWrites = true;

  // If true: editor will configure its button pins as INPUT_PULLUP (for test harness).
  // In production firmware, prefer false and let host do all pin init.
  bool initPins = false;

  // If true: start directly on the PCB HW ID screen (skip the top menu).
  bool startInPcbHwid = false;
};

// Call-once entrypoint.
// - Never returns; module exits via watchdog reset.
[[noreturn]] void run(const Config& cfg);

} // namespace KioskEepromEditor
} // namespace Kiosk

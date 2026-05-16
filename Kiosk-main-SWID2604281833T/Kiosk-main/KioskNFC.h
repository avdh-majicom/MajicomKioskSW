#pragma once
#include <Arduino.h>

#include "KioskIOpins.h" // canonical kiosk pin naming

// NFC read routine extracted from FullyWorking_TEST_FW_2512041712.ino
// Assumptions:
// - PN532 is held in reset (KioskPins::PN532_RESET LOW) when not in use.
// - Your system-wide pin init (e.g., KioskPins::initPins()) should configure:
//     * KioskPins::PN532_RESET as OUTPUT and drive LOW by default
//     * KioskPins::PN532_IRQ as INPUT
//
// This module re-initialises the PN532 inside readNFC() each time it is called.

int readNFC(uint32_t &lastSeenHash);

// Optional hook: if your project stores authorised NFC hashes in EEPROM,
// implement this in your main sketch or another module.
// If you do not provide an implementation, a weak default returns false.
bool isHashInEepromTokens(uint32_t hash);

#include "KioskSeqTest.h"

#include <OneWire.h>
#include <DallasTemperature.h>

#include "KioskIO.h"
#include "KioskIOpins.h"

extern "C" void KioskFormatHwIdText(uint8_t hwid, char* out, size_t outLen);

namespace KioskSeqTest {

static OneWire g_tempBus1(KioskPins::IN_TEMP_T1);
static OneWire g_tempBus2(KioskPins::IN_TEMP_T2);
static DallasTemperature g_temp1(&g_tempBus1);
static DallasTemperature g_temp2(&g_tempBus2);

static void pad16(char* s) {
  size_t n = strlen(s);
  if (n > 16) {
    s[16] = 0;
    return;
  }
  while (n < 16) s[n++] = ' ';
  s[16] = 0;
}

static void lcdPrintLinePadded(hd44780_I2Cexp* lcd, uint8_t row, const char* text) {
  if (!lcd) return;
  char buf[17];
  snprintf(buf, sizeof(buf), "%s", text);
  pad16(buf);
  lcd->setCursor(0, row);
  lcd->print(buf);
}

static void oledClearRow(U8X8* oled, uint8_t row) {
  if (!oled) return;
  oled->drawString(0, row, "                ");
}

static void oledDrawCenteredRow(U8X8* oled, uint8_t row, const char* text) {
  if (!oled) return;
  oledClearRow(oled, row);
  const size_t len = strlen(text);
  const uint8_t start = (len >= 16) ? 0 : (uint8_t)((16 - len) / 2);
  oled->drawString(start, row, text);
}

static void formatHwidLine(uint8_t hwid, char* out, size_t outLen) {
  char hwidText[16];
  KioskFormatHwIdText(hwid, hwidText, sizeof(hwidText));
  snprintf(out, outLen, "HWID:%s", hwidText);
  pad16(out);
}

static bool chordActive() {
  return !KioskIO::btnOzoneCtl() || !KioskIO::btnContDisp();
}

static void showReleasePrompt(const Config& cfg) {
  if (!cfg.oled || !cfg.lcd) return;

  cfg.oled->clearDisplay();
  cfg.oled->drawString(0, 0, "I/O SeqTest Mode");
  cfg.oled->drawString(0, 1, "----------------");
  oledDrawCenteredRow(cfg.oled, 5, "Release the");
  oledDrawCenteredRow(cfg.oled, 6, "invocation");
  oledDrawCenteredRow(cfg.oled, 7, "button(s)");
  oledDrawCenteredRow(cfg.oled, 8, "when ready");
  oledDrawCenteredRow(cfg.oled, 9, "to proceed");

  char hwidLine[17];
  formatHwidLine(cfg.hwid, hwidLine, sizeof(hwidLine));
  lcdPrintLinePadded(cfg.lcd, 0, "I/O SeqTest Mode");
  lcdPrintLinePadded(cfg.lcd, 1, "----------------");
  lcdPrintLinePadded(cfg.lcd, 2, hwidLine);
  if (cfg.swid) lcdPrintLinePadded(cfg.lcd, 3, cfg.swid);
}

static void renderBaseScreen(const Config& cfg) {
  if (!cfg.oled || !cfg.lcd) return;
  cfg.oled->clearDisplay();
  cfg.oled->drawString(0, 0, "I/O SeqTest Mode");
  cfg.oled->drawString(0, 1, "----------------");
  oledDrawCenteredRow(cfg.oled, 2, "Press button to");
  oledDrawCenteredRow(cfg.oled, 3, "cycle between");
  oledDrawCenteredRow(cfg.oled, 4, "I/O outputs");
  oledClearRow(cfg.oled, 6);

  lcdPrintLinePadded(cfg.lcd, 0, "I/O SeqTest Mode");
  lcdPrintLinePadded(cfg.lcd, 1, "----------------");
}

[[noreturn]] void run(const Config& cfg) {
  // TODO: Migrate SeqTest direct pin reads/writes to KioskIO getters/setters once available.
  if (!cfg.oled || !cfg.lcd) {
    while (1) { delay(1000); }
  }

  showReleasePrompt(cfg);
  while (chordActive()) {
    delay(10);
  }

  renderBaseScreen(cfg);

  struct SeqOutput {
    const char* name;
    uint8_t pin;
    bool isPwm;
    bool isKlaran;
  };

  auto disableAllOutputs = [](const SeqOutput* outputs, size_t count) {
    for (size_t i = 0; i < count; ++i) {
      pinMode(outputs[i].pin, OUTPUT);
      if (outputs[i].isPwm) {
        analogWrite(outputs[i].pin, 0);
      } else {
        digitalWrite(outputs[i].pin, LOW);
      }
    }
  };

  auto applyOutputs = [](const SeqOutput* outputs, size_t count, size_t activeIndex, bool allowKlaran) {
    for (size_t i = 0; i < count; ++i) {
      const bool on = (i == activeIndex) && (!outputs[i].isKlaran || allowKlaran);
      pinMode(outputs[i].pin, OUTPUT);
      if (outputs[i].isPwm) {
        analogWrite(outputs[i].pin, on ? 255 : 0);
      } else {
        digitalWrite(outputs[i].pin, on ? HIGH : LOW);
      }
    }
  };

  auto renderActiveOutput = [&](const char* name, int32_t remainingSec) {
    char line[17];
    memset(line, ' ', 16);
    line[16] = '\0';
    const size_t nameLen = strlen(name);
    const uint8_t nameStart = (nameLen >= 16) ? 0 : (uint8_t)((16 - nameLen) / 2);
    for (size_t i = 0; i < nameLen && (nameStart + i) < 16; ++i) {
      line[nameStart + i] = name[i];
    }
    if (remainingSec >= 0) {
      char secBuf[5];
      snprintf(secBuf, sizeof(secBuf), "%ld", (long)remainingSec);
      const size_t secLen = strlen(secBuf);
      const uint8_t secStart = (secLen >= 16) ? 0 : (uint8_t)(16 - secLen);
      for (size_t i = 0; i < secLen && (secStart + i) < 16; ++i) {
        line[secStart + i] = secBuf[i];
      }
    }
    cfg.oled->drawString(0, 6, line);
  };

  SeqOutput outputs[16];
  size_t count = 0;

  outputs[count++] = { "Dispense Button", KioskPins::PWM_DISP_BTTN_LED, true, false };
  outputs[count++] = { "Backwash Sol", KioskPins::PWM_BACKWASH_SOL, true, false };
  outputs[count++] = { "Backwash Pump", KioskPins::OUT_BACKWASH_PUMP, false, false };
  outputs[count++] = { "WaterSenseByp", KioskPins::PWM_SENSOR_BYP_SOL, true, false };
  outputs[count++] = { "InletBoostPump", KioskPins::OUT_INLET_BOOST_PUMP, false, false };
  outputs[count++] = { "Inlet Sol PWM", KioskPins::PWM_INLET_SOL, true, false };
  outputs[count++] = { "Dispense Pump", KioskPins::OUT_WATERDISP_PUMP, false, false };
  outputs[count++] = { "Dispense Sol", KioskPins::PWM_WATERDISP_SOL, true, false };

  if (cfg.hwid < 3) {
    outputs[count++] = { "Klaran UV", KioskPins::OUT_KLARAN_UV, false, true };
    outputs[count++] = { "External UV", KioskPins::OUT_EXT_UV, false, false };
    outputs[count++] = { "Ozone", KioskPins::OUT_OZONE, false, false };
  } else {
    outputs[count++] = { "Coin Acceptor", KioskPins::OUT_COIN_ACCEPTOR, false, false };
    outputs[count++] = { "Ext Relay 2", KioskPins::OUT_EXT_RELAY_2, false, false };
    outputs[count++] = { "Ext Relay 1", KioskPins::OUT_EXT_RELAY_1, false, false };
    outputs[count++] = { "Generic UV", KioskPins::OUT_EXT_UV, false, false };
    outputs[count++] = { "Klaran UV", KioskPins::OUT_KLARAN_UV, false, true };
  }

  disableAllOutputs(outputs, count);

  const size_t kNoActive = count;
  size_t activeIndex = kNoActive;
  uint32_t activeSinceMs = millis();
  bool klaranAllowed = true;
  bool outputsDirty = false;
  bool displayDirty = false;
  int32_t lastRemainingSec = -1;
  char lastSwLine[17] = "";
  char lastBtnLine[17] = "";
  char lastFloatLine[17] = "";
  char lastTempLine[17] = "";
  char lastLevelLine[17] = "";
  char lastKlaranLine[17] = "";

  static bool tempInit = false;
  static uint32_t lastTempRequestMs = 0;
  static float temp1C = 0.0f;
  static float temp2C = 0.0f;
  static bool temp1Valid = false;
  static bool temp2Valid = false;
  static float temp1Raw = 0.0f;
  static float temp2Raw = 0.0f;

  bool prevOzone = KioskIO::btnOzoneCtl();
  bool prevContDisp = KioskIO::btnContDisp();
  bool prevContCirc = KioskIO::btnContCirc();
  bool prevWaterInlet = KioskIO::btnWaterInlet();

  while (1) {
    const uint32_t now = millis();

    const bool curOzone = KioskIO::btnOzoneCtl();
    const bool curContDisp = KioskIO::btnContDisp();
    const bool curContCirc = KioskIO::btnContCirc();
    const bool curWaterInlet = KioskIO::btnWaterInlet();

    const bool nextEdge = (prevOzone && !curOzone)
                       || (prevContDisp && !curContDisp)
                       || (prevContCirc && !curContCirc);
    const bool prevEdge = (prevWaterInlet && !curWaterInlet);

    prevOzone = curOzone;
    prevContDisp = curContDisp;
    prevContCirc = curContCirc;
    prevWaterInlet = curWaterInlet;

    if (nextEdge) {
      if (activeIndex == kNoActive) {
        activeIndex = 0;
      } else if (activeIndex + 1 < count) {
        activeIndex++;
      }
      if (activeIndex != kNoActive) {
        activeSinceMs = now;
        klaranAllowed = true;
        outputsDirty = true;
        displayDirty = true;
        lastRemainingSec = -1;
      }
    } else if (prevEdge) {
      if (activeIndex == kNoActive && count > 0) {
        activeIndex = count - 1;
      } else if (activeIndex > 0) {
        activeIndex--;
      }
      if (activeIndex != kNoActive) {
        activeSinceMs = now;
        klaranAllowed = true;
        outputsDirty = true;
        displayDirty = true;
        lastRemainingSec = -1;
      }
    }

    int32_t remainingSec = -1;
    if (activeIndex != kNoActive && outputs[activeIndex].isKlaran) {
      const uint32_t elapsed = (uint32_t)(now - activeSinceMs);
      const uint32_t remainMs = (elapsed >= 15000UL) ? 0 : (15000UL - elapsed);
      remainingSec = (int32_t)((remainMs + 999UL) / 1000UL);
      if (klaranAllowed && elapsed >= 15000UL) {
        klaranAllowed = false;
        outputsDirty = true;
      }
      if (remainingSec != lastRemainingSec) {
        lastRemainingSec = remainingSec;
        displayDirty = true;
      }
    }

    if (outputsDirty && activeIndex != kNoActive) {
      applyOutputs(outputs, count, activeIndex, klaranAllowed);
      outputsDirty = false;
    }

    if (displayDirty && activeIndex != kNoActive) {
      renderActiveOutput(outputs[activeIndex].name, remainingSec);
      displayDirty = false;
    }

    const bool sw1Low = (digitalRead(KioskPins::OVERRIDE_SWITCH) == LOW);
    const bool sw2Low = (digitalRead(KioskPins::IN_SW_2) == LOW);
    const bool btn1Low = (digitalRead(KioskPins::BTN_FRONT_DISPENSE) == LOW);
    const bool btn2Low = (digitalRead(KioskPins::BTN_SPARE_BUTTON) == LOW);
    const bool floatActive = (digitalRead(KioskPins::IN_WATER_LEVEL_FULL) == LOW);
    const bool levelMedHi = (digitalRead(KioskPins::IN_WATER_LEVEL_MED) == HIGH);
    const bool levelLowHi = (digitalRead(KioskPins::IN_WATER_LEVEL_LOW) == HIGH);
    const bool klaranOkActive = (digitalRead(KioskPins::IN_KLARAN_UV_OK) == HIGH);
    const bool klaranCheckActive =
      (activeIndex != kNoActive && outputs[activeIndex].isKlaran && klaranAllowed);

    char swLine[17];
    char btnLine[17];
    char floatLine[17];
    char tempLine[17];
    char levelLine[17];
    char klaranLine[17];

    snprintf(swLine, sizeof(swLine), "SW: 1=%s 2=%s", sw1Low ? " ON" : "OFF", sw2Low ? " ON" : "OFF");
    snprintf(btnLine, sizeof(btnLine), "Btn: 1=%s 2=%s", btn1Low ? " IN" : "OUT", btn2Low ? " IN" : "OUT");
    snprintf(floatLine, sizeof(floatLine), "FloatSW:%s", floatActive ? " Active" : "Inactive");
    snprintf(levelLine, sizeof(levelLine), "Level: L=%s M=%s", levelLowHi ? "HI" : "LO", levelMedHi ? "HI" : "LO");
    if (!klaranCheckActive) {
      snprintf(klaranLine, sizeof(klaranLine), "Klaran OK: N/A");
    } else {
      snprintf(klaranLine, sizeof(klaranLine), "Klaran OK: %s", klaranOkActive ? "OK" : "NO");
    }

    if (!tempInit) {
      g_temp1.begin();
      g_temp2.begin();
      g_temp1.setWaitForConversion(false);
      g_temp2.setWaitForConversion(false);
      g_temp1.requestTemperatures();
      g_temp2.requestTemperatures();
      lastTempRequestMs = now;
      tempInit = true;
    }
    if ((uint32_t)(now - lastTempRequestMs) >= 1000UL) {
      const float t1 = g_temp1.getTempCByIndex(0);
      const float t2 = g_temp2.getTempCByIndex(0);
      temp1Raw = t1;
      temp2Raw = t2;
      temp1Valid = (t1 > -100.0f && t1 < 150.0f);
      temp2Valid = (t2 > -100.0f && t2 < 150.0f);
      if (temp1Valid) temp1C = t1;
      if (temp2Valid) temp2C = t2;
      g_temp1.requestTemperatures();
      g_temp2.requestTemperatures();
      lastTempRequestMs = now;
    }
    char t1buf[4];
    char t2buf[4];
    if (temp1Valid) snprintf(t1buf, sizeof(t1buf), "%3d", (int)((temp1C >= 0.0f) ? (temp1C + 0.5f) : (temp1C - 0.5f)));
    else snprintf(t1buf, sizeof(t1buf), " --");
    if (temp2Valid) snprintf(t2buf, sizeof(t2buf), "%3d", (int)((temp2C >= 0.0f) ? (temp2C + 0.5f) : (temp2C - 0.5f)));
    else snprintf(t2buf, sizeof(t2buf), " --");
    snprintf(tempLine, sizeof(tempLine), "Temp 1:%s 2:%s", t1buf, t2buf);

    auto writeLineIfChanged = [&](uint8_t row, const char* next, char* cache) {
      char padded[17];
      snprintf(padded, sizeof(padded), "%s", next);
      pad16(padded);
      if (strncmp(cache, padded, 16) == 0) return;
      memcpy(cache, padded, 17);
      cfg.oled->drawString(0, row, padded);
    };

    writeLineIfChanged(8, swLine, lastSwLine);
    writeLineIfChanged(9, btnLine, lastBtnLine);
    writeLineIfChanged(10, floatLine, lastFloatLine);
    writeLineIfChanged(11, tempLine, lastTempLine);
    writeLineIfChanged(12, levelLine, lastLevelLine);
    writeLineIfChanged(13, klaranLine, lastKlaranLine);

    delay(20);
  }
}

} // namespace KioskSeqTest

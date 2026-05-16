// KioskIOpins.h
// -----------------------------------------------------------------------------
// Canonical physical pin map for the Kiosk (Mega 2560).
// HEADER-ONLY:
//  - Pin names (constexpr / static const)
//  - Pin list arrays
//  - One init-table for safe startup configuration
//
// Notes:
//  - Keep this file "data only" (no pinMode/digitalWrite logic).
//  - After reset pins are inputs; set the required HIGH/LOW before pinMode(OUTPUT) to avoid glitches.
// -----------------------------------------------------------------------------

#pragma once
#include <Arduino.h>

namespace KioskPins {

   // -------------------- NFC / PN532 --------------------
   constexpr uint8_t PN532_IRQ   = 2;
   constexpr uint8_t PN532_RESET = 45;

   // -------------------- Front panel / buttons --------------------
   // (Using BTN_ nomenclature for buttons/toggles)
   constexpr uint8_t BTN_FRONT_DISPENSE   = A15;  // Front Dispense Button (front panel)
   constexpr uint8_t BTN_SPARE_BUTTON     = A14;  // Spare button (no current function)
   
   // -------------------- Switches / status --------------------
   constexpr uint8_t OVERRIDE_SWITCH      = 48;   // SW_1 (override mode when HIGH)
   constexpr uint8_t IN_SW_2              = 49;   // SW_2

   // -------------------- Coin / payment --------------------
   constexpr uint8_t IN_COIN_PULSE        = A10;  // Coin_Pulse (IN)
   constexpr uint8_t IN_COIN_ACCEPT       = 3;    // Coin_Accept (IN)

   // -------------------- Flow / dispense sensing --------------------
   // Test firmware uses A8 and A9 as inlet and dispense flow pulse input.
   constexpr uint8_t IN_DISP_FLOW_PULSE   = A9;   // DispFlowPulse (IN)
   constexpr uint8_t IN_INLET_FLOW_PULSE  = A8;   // InletWaterFlowPulse (IN)
   constexpr uint8_t IN_DISP_VOL_PULSER   = 47;   // No longer used (IN)

   // -------------------- Water level / sensors --------------------
   constexpr uint8_t IN_WATER_LEVEL_LOW   = 34;   // WaterLevelSenseLow (IN)
   constexpr uint8_t IN_WATER_LEVEL_MED   = 37;   // WaterLevelSenseMed (IN)
   constexpr uint8_t IN_WATER_LEVEL_FULL  = 40;   // WaterLevelSenseFull (IN)

   constexpr uint8_t IN_KLARAN_UV_OK      = 26;   // KlaranUV_OK (IN)
   constexpr uint8_t IN_DFP_BUSY          = 42;   // DFP_Busy (IN)
   constexpr uint8_t IN_HUMAN_PRESENCE    = 41;   // HmnPresSense (IN)

   // One-wire temperature sensors
   constexpr uint8_t IN_TEMP_T1           = 32;   // TempSenseT1 (IN)
   constexpr uint8_t IN_TEMP_T2           = 35;   // TempSenseT2 (IN)

   // Analog sensors
   constexpr uint8_t ANA_TURBIDITY        = A0;   // Sense_Turbidity (ANA-IN)
   constexpr uint8_t ANA_SPARE2           = A1;   // Sense_Spare2 (ANA-IN)
   constexpr uint8_t ANA_PH               = A2;   // Sense_PH (ANA-IN)
   constexpr uint8_t ANA_SPARE1           = A3;   // Sense_Spare1 (ANA-IN)

   // -------------------- I2C pins - Configured & managed by Wire.h -------------
   //                              Do not reconfigure!
   constexpr uint8_t I2C_SDA = 20; // SDA
   constexpr uint8_t I2C_SCL = 21; // SCL

   // -------------------- Outputs (test FW output index set) --------------------
   constexpr uint8_t PWM_BACKWASH_SOL     = 4;   // PWM - Default to 0
   constexpr uint8_t OUT_BACKWASH_PUMP    = 44;  // OUT - Default to LO
   constexpr uint8_t PWM_SENSOR_BYP_SOL   = 5;   // PWM - Default to 0
   constexpr uint8_t OUT_INLET_BOOST_PUMP = 39;  // OUT - Default to LO
   constexpr uint8_t PWM_INLET_SOL        = 7;   // PWM - Default to 0
   constexpr uint8_t OUT_WATERDISP_PUMP   = 36;  // OUT - Default to LO
   constexpr uint8_t PWM_WATERDISP_SOL    = 8;   // PWM - Default to 0
   constexpr uint8_t OUT_KLARAN_UV        = 27;  // OUT - Default to LO
   constexpr uint8_t OUT_EXT_UV           = 24;  // OUT - Default to LO
   constexpr uint8_t OUT_OZONE            = 25;  // OUT - Default to LO
   constexpr uint8_t OUT_COIN_ACCEPTOR    = 23;  // OUT - Coin Acceptor Unit (HWID > 4)
   constexpr uint8_t OUT_EXT_RELAY_2      = 28;  // OUT - External Relay Drive 2 (HWID > 4)
   constexpr uint8_t OUT_EXT_RELAY_1      = OUT_OZONE; // OUT - External Relay Drive 1 (Output 25)

   // Additional outputs from broader pin map
   constexpr uint8_t OUT_LCD_BL_CTRL      = 46;  // LCD_BL_Ctrl     // OUT - Default to LO
   constexpr uint8_t OUT_485_SBL_CTRL     = 33;  // 485_SBL_Ctrl    // OUT - Default to LO
   constexpr uint8_t OUT_485_LTE_CTRL     = 29;  // 485_LTE_Ctrl    // OUT - Default to LO
   constexpr uint8_t OUT_SPI_CS1          = A7;  // SPI_CS1         // OUT - Default to HI
   constexpr uint8_t OUT_12VMAINS_CTRL    = 43;  // 12VmainsCtrl    // OUT - Default to HI

   // -------------------- Test firmware "feature toggle" buttons --------------------
   constexpr uint8_t BTN_BACKWASH_CTL     = A13;
   constexpr uint8_t BTN_SENSOR_BYPASS    = A12;
   constexpr uint8_t BTN_CONT_CIRC        = 13;
   constexpr uint8_t BTN_WATER_INLET      = 12;
   constexpr uint8_t BTN_CONT_DISP        = 10;
   constexpr uint8_t BTN_SINGLE_DISP      = 11;
   constexpr uint8_t BTN_OZONE_CTL        = A11; // Ozone Gen / Seq step (shared in test FW)

   // -------------------- LED ring / UI outputs --------------------
   // Front Button LED Ring (PWM capable)
   constexpr uint8_t PWM_DISP_BTTN_LED    = 6;    // OC4A (test FW uses analogWrite)

   // -------------------- Convenience Outpin pin list --------------------
   static const uint8_t kOutputTestPins[] = {
      PWM_BACKWASH_SOL,
      OUT_BACKWASH_PUMP,
      PWM_SENSOR_BYP_SOL,
      OUT_INLET_BOOST_PUMP,
      PWM_INLET_SOL,
      OUT_WATERDISP_PUMP,
      PWM_WATERDISP_SOL,
      OUT_KLARAN_UV,
      OUT_EXT_UV,
      OUT_OZONE,
      PWM_DISP_BTTN_LED
   };
   static constexpr uint8_t kOutputTestPinCount =
      sizeof(kOutputTestPins) / sizeof(kOutputTestPins[0]);

   // -------------------- Convenience Button pin list --------------------
   static const uint8_t kTestButtonPins[] = {
      BTN_BACKWASH_CTL,
      BTN_SENSOR_BYPASS,
      BTN_CONT_CIRC,
      BTN_WATER_INLET,
      BTN_CONT_DISP,
      BTN_SINGLE_DISP,
      BTN_OZONE_CTL,
      BTN_FRONT_DISPENSE
   };
   static constexpr uint8_t kTestButtonPinCount =
      sizeof(kTestButtonPins) / sizeof(kTestButtonPins[0]);

   // -------------------- Unified init table --------------------
   // Iterated once at boot by KioskIO::begin().
   // Init logic should implement:
   //   - OutLow:    digitalWrite(pin, LOW)  then pinMode(pin, OUTPUT)
   //   - OutHigh:   digitalWrite(pin, HIGH) then pinMode(pin, OUTPUT)
   //   - PwmLow:    digitalWrite(pin, LOW)  then pinMode(pin, OUTPUT)
   //   - PwmHigh:   digitalWrite(pin, HIGH) then pinMode(pin, OUTPUT)
   //   - Input:     pinMode(pin, INPUT)
   enum InitRole : uint8_t {
      OutLow,
      OutHigh,
      PWMoff,
      Input
   };

   struct PinInit {
      uint8_t pin;
      InitRole role;
   };

   static const PinInit kInitTable[] = {

      // --- Inputs ---
      { OVERRIDE_SWITCH,       Input },
      { IN_SW_2,               Input },

      { IN_COIN_PULSE,         Input },
      { IN_COIN_ACCEPT,        Input },

      { IN_DISP_FLOW_PULSE,    Input },
      { IN_INLET_FLOW_PULSE,   Input },
      { IN_DISP_VOL_PULSER,    Input },

      { IN_WATER_LEVEL_LOW,    Input },
      { IN_WATER_LEVEL_MED,    Input },
      { IN_WATER_LEVEL_FULL,   Input },

      { IN_KLARAN_UV_OK,       Input },
      { IN_DFP_BUSY,           Input },
      { IN_HUMAN_PRESENCE,     Input },

      { IN_TEMP_T1,            Input },
      { IN_TEMP_T2,            Input },

      { ANA_TURBIDITY,         Input },
      { ANA_SPARE2,            Input },
      { ANA_PH,                Input },
      { ANA_SPARE1,            Input },

      // PN532
      { PN532_IRQ,             Input },
      { PN532_RESET,          OutLow },   // hold PN532 in reset until NFC init

      // --- Outputs (default LOW) ---
      { PWM_BACKWASH_SOL,     PWMoff },
      { OUT_BACKWASH_PUMP,    OutLow },
      { PWM_SENSOR_BYP_SOL,   PWMoff },
      { OUT_INLET_BOOST_PUMP, OutLow },
      { PWM_INLET_SOL,        PWMoff },
      { OUT_WATERDISP_PUMP,   OutLow },
      { PWM_WATERDISP_SOL,    PWMoff },
      { OUT_KLARAN_UV,        OutLow },
      { OUT_EXT_UV,           OutLow },
      { OUT_OZONE,            OutLow },
      { OUT_EXT_RELAY_2,      OutLow },

      { OUT_LCD_BL_CTRL,      OutLow },
      { OUT_485_SBL_CTRL,     OutLow },
      { OUT_485_LTE_CTRL,     OutLow },
      { OUT_SPI_CS1,         OutHigh },
      { OUT_12VMAINS_CTRL,    OutLow },

      { BTN_BACKWASH_CTL,      Input },
      { BTN_SENSOR_BYPASS,     Input },
      { BTN_CONT_CIRC,         Input },
      { BTN_WATER_INLET,       Input },
      { BTN_CONT_DISP,         Input },
      { BTN_SINGLE_DISP,       Input },
      { BTN_OZONE_CTL,         Input },

      // Dispense Button and LED ring:
      { PWM_DISP_BTTN_LED,    PWMoff },
      { BTN_FRONT_DISPENSE,    Input },
      { BTN_SPARE_BUTTON,      Input }
   };

   static constexpr uint8_t kInitTableCount =
      sizeof(kInitTable) / sizeof(kInitTable[0]);

   
      // -------------------- One-shot init function --------------------
   inline void initPins() {
      for (uint8_t i = 0; i < kInitTableCount; i++) {
         const PinInit &p = kInitTable[i];

         switch (p.role) {
            case Input:
               pinMode(p.pin, INPUT);
               break;

            case OutLow:
               digitalWrite(p.pin, LOW);
               pinMode(p.pin, OUTPUT);
               break;

            case OutHigh:
               digitalWrite(p.pin, HIGH);
               pinMode(p.pin, OUTPUT);
               break;

            case PWMoff:
               digitalWrite(p.pin, LOW);
               pinMode(p.pin, OUTPUT);
               break;
         }
      }
   }

} // namespace KioskPins

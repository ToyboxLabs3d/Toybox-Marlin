/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

//
// ASD ESP32-HC V2.0 (HC32f460kcta)
// Derived from XXXXX
//
#include "env_validate.h"

#if HAS_MULTI_HOTEND || E_STEPPERS > 1
  #error "ASD ESP32-HC V4.0 only supports one hotend and three E-stepper"
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "ESP32-HC V4.0"
#endif
#ifndef DEFAULT_MACHINE_NAME
  #define DEFAULT_MACHINE_NAME "ASD multi_color"
#endif

//
// Onboard crystal oscillator
//
#ifndef BOARD_XTAL_FREQUENCY
  #define BOARD_XTAL_FREQUENCY           8000000  // 8 MHz XTAL
#endif

//
// Release JTAG pins but keep SWD enabled
// - PA15 (JTDI / E0_DIR_PIN)
// - PB3 (JTDO / E0_STEP_PIN)
// - PB4 (NJTRST / E0_ENABLE_PIN)
//
//#define DISABLE_DEBUG
#define DISABLE_JTAG

//
// EEPROM
//
// Use one of these or SDCard-based Emulation will be used
//#define SRAM_EEPROM_EMULATION                   // Use BackSRAM-based EEPROM emulation
// #define FLASH_EEPROM_EMULATION                  // Use Flash-based EEPROM emulation
// #if ANY(NO_EEPROM_SELECTED, I2C_EEPROM)
//   #define I2C_EEPROM
//   #define MARLIN_EEPROM_SIZE              0x1000  // 4K
//   #define I2C_SCL_PIN                       -1
//   #define I2C_SDA_PIN                       -1
// #endif
//
// EEPROM
//
#if NO_EEPROM_SELECTED
  #define IIC_BL24CXX_EEPROM
  //#define SDCARD_EEPROM_EMULATION
  #undef NO_EEPROM_SELECTED
#endif

#if ENABLED(IIC_BL24CXX_EEPROM)
  #define IIC_EEPROM_SDA                    PA12
  #define IIC_EEPROM_SCL                    PA11
  #define MARLIN_EEPROM_SIZE               0x800 // 2K (24C16)
#elif ENABLED(SDCARD_EEPROM_EMULATION)
  #define MARLIN_EEPROM_SIZE               0x800 // 2K
#endif

//
// Servos
//
#ifndef SERVO0_PIN
  #define SERVO0_PIN                        -1 
#endif

//
// Limit Switches
//
#define X_STOP_PIN                          PC13
#define Y_STOP_PIN                          PC0
#define Z_STOP_PIN                          PC12

//
// Filament Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    -1   // "Pulled-high" *
#endif

//
// Steppers
//
#define X_ENABLE_PIN                        PC7
#define X_STEP_PIN                          PC6
#define X_DIR_PIN                           PB12

#define Y_ENABLE_PIN                        PB10
#define Y_STEP_PIN                          PC1
#define Y_DIR_PIN                           PB2

#define Z_ENABLE_PIN                        PB0
#define Z_STEP_PIN                          PB1
#define Z_DIR_PIN                           PC5
//A
#define E0_ENABLE_PIN                       PA4
#define E0_STEP_PIN                         PC4
#define E0_DIR_PIN                          PA5

#if HAS_TMC_UART
  //
  // Software serial
  // No Hardware serial for steppers
  //
  #define X_SERIAL_TX_PIN                   PA11
  #define Y_SERIAL_TX_PIN                   PC2
  #define Z_SERIAL_TX_PIN                   PA12
  #define E0_SERIAL_TX_PIN                  PC3

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART
//
// Temperature Sensors
//
#define TEMP_0_PIN                          PA0   // HEATER1 ADC1_IN0
#define TEMP_BED_PIN                        -1   // HOT BED ADC1_IN14

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PC9   // HEATER1
#define HEATER_BED_PIN                      -1    // HOT BED

#define FAN0_PIN                            PA2   // FAN0
//#define FAN1_PIN                            PA2   // FAN1


//
// USART Pins
//

// Display
#define BOARD_USART1_TX_PIN                 PA9   // LCD
#define BOARD_USART1_RX_PIN                 PA10

// Host
#define BOARD_USART2_TX_PIN                 PC10   // USB
#define BOARD_USART2_RX_PIN                 PC11

// Onboard LED (HIGH = off, LOW = on)
#ifndef LED_BUILTIN
  #define LED_BUILTIN                       PA3
#endif

//
// SD Card
//
//#define SD_DETECT_PIN                       PC10
//#define ONBOARD_SPI_DEVICE                  1
//#define ONBOARD_SD_CS_PIN                   PC3   // SDSS

//#define SDCARD_CONNECTION            ONBOARD

#if SD_CONNECTION_IS(ONBOARD)
#define ENABLE_SPI3
#define SD_SS_PIN                         -1
#define SDSS                              PA15   // SPI3_NSS
#define SD_SCK_PIN                        PB14   // SPI3_CLK
#define SD_MISO_PIN                       PB13   // SPI3_MISO
#define SD_MOSI_PIN                       PB12   // SPI3_MOSI
#define SD_DETECT_PIN                     PC10   // SD_DETECT_PIN doesn't work with NO_SD_HOST_DRIVE disabled
#endif


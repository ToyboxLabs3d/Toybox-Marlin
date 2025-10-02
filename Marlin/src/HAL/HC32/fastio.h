/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

/**
 * Fast I/O interfaces for HC32F460
 * These use GPIO functions instead of Direct Port Manipulation.
 */
#include <wiring_digital.h>
#include <wiring_analog.h>
#include <drivers/gpio/gpio.h>


#if HAS_FILAMENT_SENSOR
  extern uint32_t raw_HALL_ADC_value;
  extern bool simulate_filament_runout;
  extern bool fake_filament_pin_state;
  #define READ(IO) ((IO) == FIL_RUNOUT_PIN ? _FIL_RUNOUT_FAKE_READ(): _READ(IO))
  #define _FIL_RUNOUT_FAKE_READ() ((simulate_filament_runout || fake_filament_pin_state) ? HIGH : LOW)
  #define _READ(IO) (GPIO_GetBit(IO) ? HIGH : LOW)
#else
  #define READ(IO) (GPIO_GetBit(IO) ? HIGH : LOW)
#endif

#define WRITE(IO, V) (((V) > 0) ? GPIO_SetBits(IO) : GPIO_ResetBits(IO))
#define TOGGLE(IO) (GPIO_Toggle(IO))

#define _GET_MODE(IO) getPinMode(IO)
#define _SET_MODE(IO, M) pinMode(IO, M)
#define _SET_OUTPUT(IO) _SET_MODE(IO, OUTPUT)

#define OUT_WRITE(IO, V) \
  do {                   \
    _SET_OUTPUT(IO);     \
    WRITE(IO, V);        \
  } while (0)

#define SET_INPUT(IO) _SET_MODE(IO, INPUT_FLOATING)
#define SET_INPUT_PULLUP(IO) _SET_MODE(IO, INPUT_PULLUP)
#define SET_INPUT_PULLDOWN(IO) _SET_MODE(IO, INPUT_PULLDOWN)
#define SET_OUTPUT(IO) OUT_WRITE(IO, LOW)
#define SET_PWM(IO) _SET_MODE(IO, OUTPUT_PWM)

#define IS_INPUT(IO) (               \
  _GET_MODE(IO) == INPUT ||          \
  _GET_MODE(IO) == INPUT_FLOATING || \
  _GET_MODE(IO) == INPUT_ANALOG ||   \
  _GET_MODE(IO) == INPUT_PULLUP ||   \
  _GET_MODE(IO) == INPUT_PULLDOWN)

#define IS_OUTPUT(IO) (          \
  _GET_MODE(IO) == OUTPUT ||     \
  _GET_MODE(IO) == OUTPUT_PWM || \
  _GET_MODE(IO) == OUTPUT_OPEN_DRAIN)

#define PWM_PIN(IO) isAnalogWritePin(IO)

#define extDigitalRead(IO) digitalRead(IO)
#define extDigitalWrite(IO, V) digitalWrite(IO, V)

#define NO_COMPILE_TIME_PWM   // Can't check for PWM at compile time

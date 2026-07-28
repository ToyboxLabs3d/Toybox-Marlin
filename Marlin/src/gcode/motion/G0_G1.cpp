/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../gcode.h"
#include "../../module/motion.h"

#include "../../MarlinCore.h"

#if ALL(FWRETRACT, FWRETRACT_AUTORETRACT)
  #include "../../feature/fwretract.h"
#endif

#include "../../sd/cardreader.h"

#if ENABLED(NANODLP_Z_SYNC)
  #include "../../module/planner.h"
#endif

#if ENABLED(SOVOL_SV06_RTS)
  #include "../../lcd/sovol_rts/sovol_rts.h"
#endif

extern xyze_pos_t destination;

#if ENABLED(VARIABLE_G0_FEEDRATE)
  feedRate_t fast_move_feedrate = MMM_TO_MMS(G0_FEEDRATE);
#endif

/**
 * G0, G1: Coordinated movement of X Y Z E axes
 * 
 * Toybox Alex: Additional params:
 * L min desired X move amount (always absolute, takes precedence over calculated/given abs position, )
 * M min desired Y move amount (always absolute, takes precedence over calculated/given abs position, )
 * N min desired Z move amount (always absolute, takes precedence over calculated/given abs position, )
 * S moves are safe (skip if not homed, clamp to safe range otherwise)
 */
void GcodeSuite::G0_G1(TERN_(HAS_FAST_MOVES, const bool fast_move/*=false*/)) {
  if (!MOTION_CONDITIONS) return;

  TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_RUNNING));

  #ifdef G0_FEEDRATE
    feedRate_t old_feedrate;
    #if ENABLED(VARIABLE_G0_FEEDRATE)
      if (fast_move) {
        old_feedrate = feedrate_mm_s;             // Back up the (old) motion mode feedrate
        feedrate_mm_s = fast_move_feedrate;       // Get G0 feedrate from last usage
      }
    #endif
  #endif

  get_destination_from_command();                 // Get X Y [Z[I[J[K]]]] [E] F (and set cutter power)

  #ifdef G0_FEEDRATE
    if (fast_move) {
      #if ENABLED(VARIABLE_G0_FEEDRATE)
        fast_move_feedrate = feedrate_mm_s;       // Save feedrate for the next G0
      #else
        old_feedrate = feedrate_mm_s;             // Back up the (new) motion mode feedrate
        feedrate_mm_s = MMM_TO_MMS(G0_FEEDRATE);  // Get the fixed G0 feedrate
      #endif
    }
  #endif

  #if ALL(FWRETRACT, FWRETRACT_AUTORETRACT)

    const float echange = destination.e - current_position.e;
    const bool is_retract = echange < 0.0f;
    const bool is_emove = echange != 0.0f;
    const bool is_only_emove = is_emove && !parser.seen(STR_AXES_MAIN);

    #ifdef TBOX_ADV_AUTORETRACT
    bool need_to_track = is_emove;
    #endif

    if (MIN_AUTORETRACT <= MAX_AUTORETRACT) {
      // When M209 Autoretract is enabled, convert E-only moves to firmware retract/recover moves
      if (fwretract.autoretract_enabled && is_only_emove) {
        // Handle E-only moves

        if ( WITHIN(ABS(echange), MIN_AUTORETRACT, MAX_AUTORETRACT)) {
          // E-only move, handled by (advanced) autoretract.
          
          #ifdef TBOX_ADV_AUTORETRACT
          if(fwretract.in_advanced_autoretract_mode()){
            SERIAL_ECHOLNPGM("G0_G1() e_only move. clamping");
            fwretract.clamp_move();
            need_to_track = false;
          } else 
          #endif
          {
            current_position.e = destination.e;       // Hide a G1-based retract/recover from calculations
            sync_plan_position_e();                   // AND from the planner
            // SERIAL_ECHOLNPGM("G0_G1() converting to G10/G11. is_retract: ", AS_DIGIT(is_retract), " echange: ", echange);
            fwretract.retract(is_retract); // convert to G10/G11
            return;
          }


        } 
      } 
      #ifdef TBOX_ADV_AUTORETRACT
      else if (fwretract.in_advanced_autoretract_mode() && is_emove && is_retract && WITHIN(ABS(echange), MIN_AUTORETRACT, MAX_AUTORETRACT)){
        // SERIAL_ECHOLNPGM("G0_G1() e_move but not e_only_move retract move. clamping");
        fwretract.clamp_move();
        need_to_track = false;
      } 
      #endif
    }
    
    #ifdef TBOX_ADV_AUTORETRACT
      if(need_to_track){
        fwretract.track_change(echange);
      }
    #endif

  #endif // FWRETRACT

  #if ENABLED(TOYBOX_FAST_CMDS)
    // FIXME (Toybox Alex): Do these stop running move commands invalidate the current position? 
    // We already set the destination, but we're not applying it, and it will probably be overwritten
    // later.
    if(stop_running_move) {
      return;
    }
  #endif

  // -- Toybox Alex: min move and safe move feature

  const bool safe_mode = parser.seen('S'); // don't move if not homed, clamp to safe range (no grinding)
  const char axes[] = { 'X', 'Y', 'Z'};
  const char min_move_flags[] = { 'L', 'M', 'N'}; // minimum move deltas (can be positive or negative), will override the X,Y,Z flags if needed.
  
  for (uint8_t i = 0; i < COUNT(axes); i++) {
    const bool seen_min_move = parser.seenval(min_move_flags[i]);
    if(!seen_min_move && !parser.seen(axes[i])) {
      continue; 
    }
    if (seen_min_move){
      const float min_move = parser.value_linear_units();
      if(!safe_mode) {
        SERIAL_ECHOLNPGM("Error: G0/G1 min move feature requires S parameter to be set. Ignoring min move for axis ", C(axes[i]));
        continue;
      }
      if ((min_move > 0.0f && (destination[i] - current_position[i]) < min_move) 
          || (min_move < 0.0f && (destination[i] - current_position[i]) > min_move)) {
        destination[i] = current_position[i] + min_move;
      }
    }
    if (safe_mode){
      #if !ENABLED(MAX_SOFTWARE_ENDSTOPS) || !ENABLED(MIN_SOFTWARE_ENDSTOPS)
        SERIAL_ECHOLNPGM("Error: G0/G1 safe move feature requires software endstops to be enabled. Ignoring safe move for axis ", C(axes[i]));
        destination[i] = current_position[i];
      #else
        // Move should be clamped by software endstops elsewhere. We just need to make sure the axis is homed.
        if (!axis_was_homed((AxisEnum)i)) {
          SERIAL_ECHOLNPGM("Warning: G0/G1 safe move feature requires axis ", C(axes[i]), " to be homed. Ignoring safe move for this axis.");
          destination[i] = current_position[i];
        }
      #endif
    }
  }

  #if ANY(IS_SCARA, POLAR)
    fast_move ? prepare_fast_move_to_destination() : prepare_line_to_destination();
  #else
    prepare_line_to_destination();
  #endif

  #ifdef G0_FEEDRATE
    // Restore the motion mode feedrate
    if (fast_move) feedrate_mm_s = old_feedrate;
  #endif

  #if ENABLED(NANODLP_Z_SYNC)
    #if ENABLED(NANODLP_ALL_AXIS)
      #define _MOVE_SYNC parser.seenval('X') || parser.seenval('Y') || parser.seenval('Z')  // For any move wait and output sync message
    #else
      #define _MOVE_SYNC parser.seenval('Z')  // Only for Z move
    #endif
    if (_MOVE_SYNC) {
      planner.synchronize();
      SERIAL_ECHOLNPGM(STR_Z_MOVE_COMP);
    }
    TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_IDLE));
  #else
    TERN_(FULL_REPORT_TO_HOST_FEATURE, report_current_grblstate_moving());
  #endif

  TERN_(SOVOL_SV06_RTS, RTS_PauseMoveAxisPage());
}

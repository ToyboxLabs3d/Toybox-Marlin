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

/**
 * fwretract.cpp - Implement firmware-based retraction
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(FWRETRACT)

#include "fwretract.h"

FWRetract fwretract; // Single instance - this calls the constructor

#include "../module/motion.h"
#include "../module/planner.h"

#include "../gcode/gcode.h"

#if ENABLED(RETRACT_SYNC_MIXING)
  #include "mixing.h"
#endif

#if defined(TBOX_ADV_AUTORETRACT) && HAS_MULTI_EXTRUDER
  #error "TBOX_ADV_AUTORETRACT is not compatible with multiple extruders"
#endif

// private:

#if HAS_MULTI_EXTRUDER
  Flags<EXTRUDERS> FWRetract::retracted_swap;         // Which extruders are swap-retracted
#endif

// public:

fwretract_settings_t FWRetract::settings;             // M207 S F Z W, M208 S F W R

#if ENABLED(FWRETRACT_AUTORETRACT)
  bool FWRetract::autoretract_enabled;                // M209 S - Autoretract switch
#endif

#ifdef TBOX_ADV_AUTORETRACT
  float FWRetract::retracted_amnt = 0.0f;           
  AutoRetractMode FWRetract::autoretract_mode = 
  #ifdef TBOX_ADV_AUTORETRACT_ON_DEFAULT
    AutoRetractMode::ADVANCED;
  #else
    AutoRetractMode::OFF; 
  #endif
#endif

Flags<EXTRUDERS> FWRetract::retracted;                // Which extruders are currently retracted

float FWRetract::current_retract[EXTRUDERS] = {0.0f};          // Retract value used by planner
float FWRetract::current_hop = 0.0f;

void FWRetract::reset() {
  #if ENABLED(FWRETRACT_AUTORETRACT)
    #ifdef TBOX_ADV_AUTORETRACT
      #ifdef TBOX_ADV_AUTORETRACT_ON_DEFAULT
        autoretract_enabled = true;
        autoretract_mode = AutoRetractMode::ADVANCED;
      #else
        autoretract_enabled = false;
        autoretract_mode = AutoRetractMode::OFF;
      #endif
    #else
      autoretract_enabled = false;
    #endif
  #endif
  settings.retract_length = RETRACT_LENGTH;
  settings.retract_feedrate_mm_s = RETRACT_FEEDRATE;
  settings.retract_zraise = RETRACT_ZRAISE;
  settings.retract_recover_extra = RETRACT_RECOVER_LENGTH;
  settings.retract_recover_feedrate_mm_s = RETRACT_RECOVER_FEEDRATE;
  settings.swap_retract_length = RETRACT_LENGTH_SWAP;
  settings.swap_retract_recover_extra = RETRACT_RECOVER_LENGTH_SWAP;
  settings.swap_retract_recover_feedrate_mm_s = RETRACT_RECOVER_FEEDRATE_SWAP;
  current_hop = 0.0;

#ifdef TBOX_ADV_AUTORETRACT
  if(autoretract_mode != AutoRetractMode::ADVANCED)
#endif
  {
    retracted.reset();
    EXTRUDER_LOOP() {
      E_TERN_(retracted_swap.clear(e));
      current_retract[e] = 0.0;
    }
  }
}

/**
 * Retract or recover according to firmware settings
 *
 * This function handles retract/recover moves for G10 and G11,
 * plus auto-retract moves sent from G0/G1 when E-only moves are done.
 *
 * To simplify the logic, doubled retract/recover moves are ignored.
 *
 * Note: Auto-retract will apply the set Z hop in addition to any Z hop
 *       included in the G-code. Use M207 Z0 to to prevent double hop.
 * 
 * Toybox Alex: In advanced autoretract mode, this is only called from G10/G11.
 * In advanced autoretract mode, moves are possibly skipped based on current 
 * retracted_amnt, or clamped.
 * 
 * If advanced autoretract is not enabled, this does the same thing as before, 
 * except the retracted_amnt is still tracked, and the retracted[active_extruder]
 * flag will still have been possibly set from when we were in advanced autoretract mode.
 * 
 */
void FWRetract::retract(const bool retracting E_OPTARG(bool swapping/*=false*/)) 
{
  // SERIAL_ECHOLNPGM(">> FWRetract::retract() retracting: ", AS_DIGIT(retracting), 
  // " retracted_amnt: ", retracted_amnt, 
  // " current_retract[active_extruder]: ", current_retract[active_extruder], 
  // " retracted[active_extruder]: ", AS_DIGIT(retracted[active_extruder]));

  #ifdef TBOX_ADV_AUTORETRACT
    bool next_retracted_state = retracting; 
    float delta_retracted_amnt = 0.0f;

    if(autoretract_mode == AutoRetractMode::ADVANCED) {
      // don't G10 retract past RETRACT_LENGTH
      const float e_move_for_retract = - max(0.0f, RETRACT_LENGTH - retracted_amnt);
      const float attempted_e_move = retracting ? e_move_for_retract : retracted_amnt;

      const float old_retracted_amnt = retracted_amnt;

      if(retracting && retracted_amnt >= TBOX_ADV_AUTORETRACT_MAX_PERMITED_RETRACT_LENGTH){
        SERIAL_ECHOLNPGM("Retracted amount super high, pressumably we got into this state while advance-autoretract was off.");
        return;
      }
      retracted_amnt -= attempted_e_move;
      retracted_amnt = constrain(retracted_amnt, 0.0f, TBOX_ADV_AUTORETRACT_MAX_PERMITED_RETRACT_LENGTH);

      if(retracted_amnt < 0.0001f && retracted_amnt > -0.0001f){
        // rounding error fix.
        retracted_amnt = 0.0f;
        next_retracted_state = false;
      }else {
        next_retracted_state = true;
      }

      delta_retracted_amnt = retracted_amnt - old_retracted_amnt;

      if (delta_retracted_amnt < 0.0001f && delta_retracted_amnt > -0.0001f) {
        retracted[active_extruder] = next_retracted_state;
        // SERIAL_ECHOLNPGM("delta_retracted_amnt is 0.0f ");
        // SERIAL_ECHOLNPGM("<< FWRetract::retract() retracting: ", AS_DIGIT(retracting), 
        //     " retracted_amnt: ", retracted_amnt, 
        //     " current_retract[active_extruder]: ", current_retract[active_extruder], 
        //     " retracted[active_extruder]: ", AS_DIGIT(retracted[active_extruder]));
        return;
      }


    } else
  #endif
  {
    if (retracted[active_extruder] == retracting){
      // SERIAL_ECHOLNPGM("retracted[active_extruder] == retracting. ignoring.");
      return;
    } 
  }

  #if HAS_MULTI_EXTRUDER
    // Allow G10 S1 only after G11
    if (swapping && retracted_swap[active_extruder] == retracting) return;
    // G11 priority to recover the long retract if activated
    if (!retracting) swapping = retracted_swap[active_extruder];
  #else
    constexpr bool swapping = false;
  #endif

  /* // debugging
    SERIAL_ECHOLNPGM(
      "retracting ", AS_DIGIT(retracting),
      " swapping ", swapping,
      " active extruder ", active_extruder
    );
    EXTRUDER_LOOP() {
      SERIAL_ECHOLNPGM("retracted[", e, "] ", AS_DIGIT(retracted[e]));
      #if HAS_MULTI_EXTRUDER
        SERIAL_ECHOLNPGM("retracted_swap[", e, "] ", AS_DIGIT(retracted_swap[e]));
      #endif
    }
    SERIAL_ECHOLNPGM("current_position.z ", current_position.z);
    SERIAL_ECHOLNPGM("current_position.e ", current_position.e);
    SERIAL_ECHOLNPGM("current_hop ", current_hop);
  //*/

  #ifdef TBOX_ADV_AUTORETRACT
    if(!in_advanced_autoretract_mode()){
      delta_retracted_amnt = TERN1(RETRACT_SYNC_MIXING, (MIXING_STEPPERS))
      * (swapping ? settings.swap_retract_length : settings.retract_length);
    }
  #else
    const float base_retract = TERN1(RETRACT_SYNC_MIXING, (MIXING_STEPPERS))
                  * (swapping ? settings.swap_retract_length : settings.retract_length);
  #endif
  // The current position will be the destination for E and Z moves
  destination = current_position;

  #if ENABLED(RETRACT_SYNC_MIXING)
    const uint8_t old_mixing_tool = mixer.get_current_vtool();
    mixer.T(MIXER_AUTORETRACT_TOOL);
  #endif

  const feedRate_t fr_max_z = planner.settings.max_feedrate_mm_s[Z_AXIS];
  if (retracting) {
    // Retract by moving from a faux E position back to the current E position
    #ifdef TBOX_ADV_AUTORETRACT
      current_retract[active_extruder] += delta_retracted_amnt;
    #else
      current_retract[active_extruder] = base_retract;
    #endif

      prepare_internal_move_to_destination(                 // set current from destination
        MUL_TERN(RETRACT_SYNC_MIXING, settings.retract_feedrate_mm_s, MIXING_STEPPERS)
      );

    // Is a Z hop set, and has the hop not yet been done?
    if (!current_hop && settings.retract_zraise > 0.01f) {  // Apply hop only once
      current_hop += settings.retract_zraise;               // Add to the hop total (again, only once)
      // Raise up, set_current_to_destination. Maximum Z feedrate
      prepare_internal_move_to_destination(fr_max_z);
    }
  }
  else {
    // If a hop was done and Z hasn't changed, undo the Z hop
    if (current_hop) {
      current_hop = 0;
      // Lower Z, set_current_to_destination. Maximum Z feedrate
      prepare_internal_move_to_destination(fr_max_z);
    }

    const float extra_recover = swapping ? settings.swap_retract_recover_extra : settings.retract_recover_extra;
    if (extra_recover) {
      current_position.e -= extra_recover;          // Adjust the current E position by the extra amount to recover
      sync_plan_position_e();                       // Sync the planner position so the extra amount is recovered
    }

    #ifdef TBOX_ADV_AUTORETRACT 
      current_retract[active_extruder] += delta_retracted_amnt;
    #else
      current_retract[active_extruder] = 0;
    #endif

      prepare_internal_move_to_destination(                 
        MUL_TERN(RETRACT_SYNC_MIXING, swapping ? settings.swap_retract_recover_feedrate_mm_s : settings.retract_recover_feedrate_mm_s, MIXING_STEPPERS)
      );
  }

  TERN_(RETRACT_SYNC_MIXING, mixer.T(old_mixing_tool));   // Restore original mixing tool



  
  #ifdef TBOX_ADV_AUTORETRACT
    if(in_advanced_autoretract_mode()){
      retracted[active_extruder] = next_retracted_state;   
    }else
  #endif
  {
    retracted.set(active_extruder, retracting);           
  }
  // If swap retract/recover update the retracted_swap flag too
  #if HAS_MULTI_EXTRUDER
    if (swapping) retracted_swap.set(active_extruder, retracting);
  #endif


    SERIAL_ECHOLNPGM("<< FWRetract::retract() retracting: ", AS_DIGIT(retracting), 
      " retracted_amnt: ", retracted_amnt, 
      " current_retract[active_extruder]: ", current_retract[active_extruder], 
      " retracted[active_extruder]: ", AS_DIGIT(retracted[active_extruder]));

  /* // debugging
        SERIAL_ECHOLNPGM("retracting ", AS_DIGIT(retracting));
    SERIAL_ECHOLNPGM("swapping ", AS_DIGIT(swapping));
    SERIAL_ECHOLNPGM("active_extruder ", active_extruder);
    EXTRUDER_LOOP() {
      SERIAL_ECHOLNPGM("retracted[", e, "] ", AS_DIGIT(retracted[e]));
      #if HAS_MULTI_EXTRUDER
        SERIAL_ECHOLNPGM("retracted_swap[", e, "] ", AS_DIGIT(retracted_swap[e]));
      #endif
    }
    SERIAL_ECHOLNPGM("current_position.z ", current_position.z);
    SERIAL_ECHOLNPGM("current_position.e ", current_position.e);
    SERIAL_ECHOLNPGM("current_hop ", current_hop);
  //*/
}

#ifdef TBOX_ADV_AUTORETRACT

/**
 * Toybox Alex:
 * Limit emoves in both directions.
 * 
 * Only called in advanced autoretract mode.
 * 
 * Called from G0/G1 for all E-only moves, and from G0/G1/G2/G3 for all backwards E moves.
 * 
 * Fucks with current_position.e to fake out the planner and make it think it's in a 
 * different position so it moves a different amount. The actual move will still happen at
 * the same time as everything else from G0/G1/G2/G3, so the caller will still have to
 * go through that procedure.
 * 
 * Updates the retracted_amnt and the retracted[active_extruder] flag to keep track of the
 * current state.
 */
void FWRetract::clamp_move(){
    // SERIAL_ECHOLNPGM(">> FWRetract::clamp_move()   retracted_amnt: ", retracted_amnt, 
    //   " current_retract[active_extruder]: ", current_retract[active_extruder], 
    //   " retracted[active_extruder]: ", AS_DIGIT(retracted[active_extruder]));
    if(!in_advanced_autoretract_mode()){
      SERIAL_ECHOLNPGM("ERROR: clamp_move() called but not in advanced autoretract mode -----------------------------------");
      return;
    }
    const float e_retraction = current_position.e - destination.e;

    // SERIAL_ECHOLNPGM("clamp_move() e_retraction: ", e_retraction);
    const float start_retracted_amnt = retracted_amnt;
    float delta_retracted_amnt = e_retraction;

    // Toybox Alex: retracted_amnt might be greater than TBOX_ADV_AUTORETRACT_MAX_PERMITED_RETRACT_LENGTH 
    // if there were moves when we were not in advanced autoretract mode. retracted_amnt should never be 
    // less than zero though. 
    if(retracted_amnt >= TBOX_ADV_AUTORETRACT_MAX_PERMITED_RETRACT_LENGTH && e_retraction > 0.0f){
        SERIAL_ECHOLNPGM("clamp_move() retracting but already beyond max permitted. ignoring.");
        delta_retracted_amnt = 0.0f;
    } else {
      retracted_amnt = constrain(retracted_amnt + e_retraction, 0.0f, TBOX_ADV_AUTORETRACT_MAX_PERMITED_RETRACT_LENGTH);
      delta_retracted_amnt = retracted_amnt - start_retracted_amnt;
    }
    const float amnt_reduced = e_retraction - delta_retracted_amnt;

    // SERIAL_ECHOLNPGM("clamp_move() start_retracted_amnt: ", start_retracted_amnt, " retracted_amnt: ", retracted_amnt, " delta_retracted_amnt: ", delta_retracted_amnt, " amnt_reduced: ", amnt_reduced);

    current_position.e -= amnt_reduced; // Hide a G1-based retract from calculations
    sync_plan_position_e();             // AND from the planner

    if(retracted_amnt < 0.00001f && retracted_amnt > -0.00001f){
      retracted_amnt = 0.0f; // rounding error fix
    }

    // SERIAL_ECHOLNPGM("<< FWRetract::clamp_move()   retracted_amnt: ", retracted_amnt, 
    //   " current_retract[active_extruder]: ", current_retract[active_extruder], 
    //   " retracted[active_extruder]: ", AS_DIGIT(retracted[active_extruder]));
}

/**
 * Toybox Alex:
 * Track the change in retracted_amnt.
 * If we're in advanced autoretract mode, update the retracted[active_extruder] flag.
 * 
 * Called for all E moves that weren't handled by another FWRetract function.
 * 
 */
void FWRetract::track_change(const float e_move){

    // SERIAL_ECHOLNPGM(">> FWRetract::track_change() e_move: ", e_move,
    //   " retracted_amnt: ", retracted_amnt, 
    //   " current_retract[active_extruder]: ", current_retract[active_extruder], 
    //   " retracted[active_extruder]: ", AS_DIGIT(retracted[active_extruder]));

  retracted_amnt = constrain(retracted_amnt - e_move, 0.0f, TBOX_ADV_AUTORETRACT_MAX_AMNT_CONSIDERED_RETRACTED);

  if(retracted_amnt < 0.0001f && retracted_amnt > -0.0001f){
    retracted_amnt = 0.0f; // rounding error fix
  }
  if(in_advanced_autoretract_mode()){
    retracted[active_extruder] = retracted_amnt != 0.0f;    
  }

  // SERIAL_ECHOLNPGM("<< FWRetract::track_change() e_move: ", e_move,
  // " retracted_amnt: ", retracted_amnt, 
  // " current_retract[active_extruder]: ", current_retract[active_extruder], 
  // " retracted[active_extruder]: ", AS_DIGIT(retracted[active_extruder]));
}


#endif
/**
 * M207: Set firmware retraction values
 *
 *   S[+units]    retract_length
 *   W[+units]    swap_retract_length (multi-extruder)
 *   F[units/min] retract_feedrate_mm_s
 *   Z[units]     retract_zraise
 */
void FWRetract::M207() {
  if (!parser.seen("FSWZ")) return M207_report();
  if (parser.seenval('S')) settings.retract_length        = parser.value_axis_units(E_AXIS);
  if (parser.seenval('F')) settings.retract_feedrate_mm_s = MMM_TO_MMS(parser.value_axis_units(E_AXIS));
  if (parser.seenval('Z')) settings.retract_zraise        = parser.value_linear_units();
  if (parser.seenval('W')) settings.swap_retract_length   = parser.value_axis_units(E_AXIS);
}

void FWRetract::M207_report() {
  TERN_(MARLIN_SMALL_BUILD, return);

  SERIAL_ECHOLNPGM_P(
      PSTR("  M207 S"), LINEAR_UNIT(settings.retract_length)
    , PSTR(" W"), LINEAR_UNIT(settings.swap_retract_length)
    , PSTR(" F"), LINEAR_UNIT(MMS_TO_MMM(settings.retract_feedrate_mm_s))
    , SP_Z_STR, LINEAR_UNIT(settings.retract_zraise)
  );
}

/**
 * M208: Set firmware un-retraction values
 *
 *   S[+units]    retract_recover_extra (in addition to M207 S*)
 *   W[+units]    swap_retract_recover_extra (multi-extruder)
 *   F[units/min] retract_recover_feedrate_mm_s
 *   R[units/min] swap_retract_recover_feedrate_mm_s
 */
void FWRetract::M208() {
  if (!parser.seen("FSRW")) return M208_report();
  if (parser.seen('S')) settings.retract_recover_extra              = parser.value_axis_units(E_AXIS);
  if (parser.seen('F')) settings.retract_recover_feedrate_mm_s      = MMM_TO_MMS(parser.value_axis_units(E_AXIS));
  if (parser.seen('R')) settings.swap_retract_recover_feedrate_mm_s = MMM_TO_MMS(parser.value_axis_units(E_AXIS));
  if (parser.seen('W')) settings.swap_retract_recover_extra         = parser.value_axis_units(E_AXIS);
}

void FWRetract::M208_report() {
  TERN_(MARLIN_SMALL_BUILD, return);

  SERIAL_ECHOLNPGM(
      "  M208 S", LINEAR_UNIT(settings.retract_recover_extra)
    , " W", LINEAR_UNIT(settings.swap_retract_recover_extra)
    , " F", LINEAR_UNIT(MMS_TO_MMM(settings.retract_recover_feedrate_mm_s))
  );
}

#if ENABLED(FWRETRACT_AUTORETRACT)

  /**
   * M209: Enable automatic retract (M209 S1)
   *   For slicers that don't support G10/11, reversed extrude-only
   *   moves will be classified as retraction.
   */
  void FWRetract::M209() {
    if (!parser.seen('S')) return M209_report();
    if (MIN_AUTORETRACT <= MAX_AUTORETRACT){
      #ifdef TBOX_ADV_AUTORETRACT
        AutoRetractMode mode;
        if(!parser.seenval('S')){
          mode = AutoRetractMode::NORMAL;
        }else{
          uint8_t mode_byte = parser.value_byte();
          if(mode_byte >= static_cast<uint8_t>(AutoRetractMode::NUM_MODES)){
            SERIAL_ECHOLNPGM("Invalid autoretract mode ", mode_byte);
            return M209_report();
          }
          mode = static_cast<AutoRetractMode>(mode_byte);
        }
        set_autoretract_mode(mode);
      #else
        enable_autoretract(parser.value_bool());
      #endif
    }   
  }

  #ifdef TBOX_ADV_AUTORETRACT
  void FWRetract::set_autoretract_mode(const AutoRetractMode new_mode) {
    autoretract_mode = new_mode;
    switch (new_mode) {
      case AutoRetractMode::OFF:
        autoretract_enabled = false;
        SERIAL_ECHOLNPGM("Autoretract disabled");
        break;
      case AutoRetractMode::NORMAL:
        autoretract_enabled = true;
        SERIAL_ECHOLNPGM("Basic autoretract enabled");
        break;
      case AutoRetractMode::ADVANCED:
        retracted[active_extruder] = retracted_amnt != 0.0f;
        autoretract_enabled = true;
        SERIAL_ECHOLNPGM("Advanced autoretract enabled");
        break;
      default:
        SERIAL_ECHOLNPGM("Invalid autoretract mode ", static_cast<uint8_t>(new_mode));
    }
  }
  #endif


  void FWRetract::M209_report() {
    TERN_(MARLIN_SMALL_BUILD, return);

    SERIAL_ECHOLNPGM("  M209 S", AS_DIGIT(autoretract_enabled));
  }

#endif // FWRETRACT_AUTORETRACT

#endif // FWRETRACT

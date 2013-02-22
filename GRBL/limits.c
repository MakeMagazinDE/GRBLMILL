/*
  limits.c - code pertaining to limit-switches and performing the homing cycle
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.


  War buggy. Kein Enable gesetzt.
*/

#include <util/delay.h>
#include <avr/io.h>
#include "stepper.h"
#include "settings.h"
#include "nuts_bolts.h"
#include "config.h"
#include "motion_control.h"
#include "planner.h"

// TODO: Deprecated. Need to update for new version. Sys.position now tracks position relative
// to the home position. Limits should update this vector directly.

void limits_init() {
  LIMIT_DDR &= ~(LIMIT_MASK);
}

static void homing_cycle(bool x_axis, bool y_axis, bool z_axis, bool reverse_direction, uint32_t microseconds_per_pulse) {
  // First home the Z axis
  uint32_t step_delay = microseconds_per_pulse - settings.pulse_microseconds;
  uint8_t out_bits;
  uint8_t limit_bits;


// STEP_MASK: nur Pulse-Bits
// DIRECTION_MASK: nur Direction-Bits
// STEPPING_MASK: Pulse-Bits und Direction-Bits
// LIMIT_MASK: nur Limit-Eingänge
// siehe auch stepper.h

  out_bits = DIRECTION_MASK; // Direction-Bits setzen
  // Set direction bits if this is a reverse homing_cycle
  if (reverse_direction) {
    out_bits = 0;
  }

  if (x_axis) { out_bits |= (1<<X_STEP_BIT); }
  if (y_axis) { out_bits |= (1<<Y_STEP_BIT); }
  if (z_axis) { out_bits |= (1<<Z_STEP_BIT); }

  // Erst einmal nur Direction-Bits setzen, Invertierung berücksichtigen
  STEPPING_PORT = ((STEPPING_PORT & ~DIRECTION_MASK) | (out_bits & DIRECTION_MASK)) ^ (settings.invert_mask & STEPPING_MASK);
  delay_us(settings.pulse_microseconds);

  for(;;) {
    limit_bits = LIMIT_PIN ^ LIMIT_INVMASK;
    if (reverse_direction) {
      // Invert limit_bits if this is a reverse homing_cycle
      limit_bits ^= LIMIT_MASK;
    }
    if (x_axis && !(limit_bits & (1<<X_LIMIT_BIT))) {
      x_axis = false;
      out_bits ^= (1<<X_STEP_BIT);	// Bit invertieren
    }
    if (y_axis && !(limit_bits & (1<<Y_LIMIT_BIT))) {
      y_axis = false;
      out_bits ^= (1<<Y_STEP_BIT);	// Bit invertieren
    }
    if (z_axis && !(limit_bits & (1<<Z_LIMIT_BIT))) {
      z_axis = false;
      out_bits ^= (1<<Z_STEP_BIT);	// Bit invertieren
    }

    STEPPING_PORT |= (out_bits ^ settings.invert_mask) & STEP_MASK;	// Pulse-Bits setzen
    delay_us(settings.pulse_microseconds);
    STEPPING_PORT ^= (out_bits ^ settings.invert_mask) & STEP_MASK;	// Pulse-Bits löschen
    // simple ramp if not leaving switch
    if ((step_delay > HOMING_CYCLE) & (!reverse_direction)) {
      step_delay = (step_delay * 98) / 100;
    }
    delay_us(step_delay);

    // Check if we are done
    if(!(x_axis || y_axis || z_axis)) {
      return;
    }
 }
  return;
}


static void approach_limit_switch(bool x, bool y, bool z) {
  homing_cycle(x, y, z, false, (HOMING_CYCLE << 4)); // HOMING_CYCLE Ramp Start Values
}

static void leave_limit_switch(bool x, bool y, bool z) {
  uint8_t my_count;
  uint8_t out_bits = 0;
  homing_cycle(x, y, z, true, (HOMING_CYCLE << 5));  // HOMING_CYCLE Ramp Start Values

  // noch Impulse hinterherschieben, damit Schalter sicher verlassen wird
  if (x) { out_bits |= (1<<X_STEP_BIT); }
  if (y) { out_bits |= (1<<Y_STEP_BIT); }
  if (z) { out_bits |= (1<<Z_STEP_BIT); }
  for (my_count=0; my_count<HOMING_LEAVE_PULSES; my_count++) {
    STEPPING_PORT |= (out_bits ^ settings.invert_mask) & STEP_MASK;	// Pulse-Bits setzen
    delay_us(settings.pulse_microseconds);
    STEPPING_PORT ^= (out_bits ^ settings.invert_mask) & STEP_MASK;	// Pulse-Bits löschen
    delay_us(HOMING_CYCLE << 5);
    }
}

void limits_go_home() {
  plan_synchronize();
  // Enable steppers by resetting the stepper disable port
  STEPPERS_ENABLE_PORT |= (1<<STEPPERS_ENABLE_BIT);
  STEPPERS_ENABLE_PORT |= (1<<STEPPERS_ACTIVITY_BIT);
//  STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT);

  sys.position[X_AXIS]=0;
  sys.position[Y_AXIS]=0;
  sys.position[Z_AXIS]=0;

#ifdef Z_LIMIT_PRESENT
  approach_limit_switch(false, false, true); // First home the z axis
  delay_ms(50);
  // Now carefully leave the limit switches
  leave_limit_switch(false, false, true);
  delay_ms(50);
#endif


#ifdef Y_HOME_FIRST

#ifdef Y_LIMIT_PRESENT
  approach_limit_switch(false, true, false); // Home Y axis
  delay_ms(50);
  leave_limit_switch(false, true, false);
  delay_ms(50);
#endif
#ifdef X_LIMIT_PRESENT
  approach_limit_switch(true, false, false); // Home X axis
  delay_ms(50);
  leave_limit_switch(true, false, false);
  delay_ms(50);
#endif

#else

#ifdef X_LIMIT_PRESENT
  approach_limit_switch(true, false, false); // Home X axis
  delay_ms(50);
  leave_limit_switch(true, false, false);
  delay_ms(50);
#endif
#ifdef Y_LIMIT_PRESENT
  approach_limit_switch(false, true, false); // Home Y axis
  delay_ms(50);
  leave_limit_switch(false, true, false);
  delay_ms(50);
#endif

#endif


  // Disable steppers by setting stepper disable
  delay_ms(50);
  STEPPERS_ENABLE_PORT &= ~(1<<STEPPERS_ENABLE_BIT);
  STEPPERS_ENABLE_PORT &= ~(1<<STEPPERS_ACTIVITY_BIT);
//  STEPPERS_DISABLE_PORT |= (1<<STEPPERS_DISABLE_BIT);


// Hardware-Zähler zurücksetzen
  st_counter_null();

}

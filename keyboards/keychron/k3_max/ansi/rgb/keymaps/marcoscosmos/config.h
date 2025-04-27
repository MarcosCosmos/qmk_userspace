/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once


// disable one shot tap toggle as we are using our custom double-tap-to-release without toggling on to begin with (this is only being applied to modifiers but one shot layers will get a similar treatment in future)
#ifdef ONESHOT_TAP_TOGGLE
#undef ONESHOT_TAP_TOGGLE
#endif

/* Set LED driver current */
#ifdef SNLED27351_CURRENT_TUNE
#undef SNLED27351_CURRENT_TUNE
#endif
#define SNLED27351_CURRENT_TUNE \
        { 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30 }

#ifdef RGB_MATRIX_DEFAULT_MODE
#undef RGB_MATRIX_DEFAULT_MODE
#endif
#define RGB_MATRIX_DEFAULT_MODE RGB_MATRIX_SOLID_COLOR

#ifdef RGB_MATRIX_DEFAULT_VAL
#undef RGB_MATRIX_DEFAULT_VAL
#endif
#define RGB_MATRIX_DEFAULT_VAL

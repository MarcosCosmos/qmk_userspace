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

#include <stdint.h>
#include "action_util.h"
#include "color.h"
#include "config.h"
#include "debug.h"
#include "keycodes.h"
#include "quantum.h"
#include "quantum_keycodes.h"
#include "rgb_matrix.h"
#include QMK_KEYBOARD_H
#include "print.h"
#include "keychron_common.h"

#define LAYER_INDICATOR_COLOR RGB_PURPLE
#define ACTIVE_INDICATOR_COLOR RGB_GOLDENROD

#define DEV_NOOP_INDICATOR_COLOR RGB_RED
#define DEV_BUG_OFF_INDICATOR_COLOR RGB_BLUE

#define WM_PSEUDOMODIFIER X_F14

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0')

enum layers {
    LAYER_MIN,
    LYR_BASE=LAYER_MIN,
    LYR_FN,
    LYR_WM,
    LYR_DEV,
    LAYER_MAX=LYR_DEV
};

// window management keys, uses a modifier that isn't really a modifier (such as F14) instead of GUI in order to circumvent windows hardcoding GUI shortcuts.
// the idea is to use it with a layer, e.g. layer-tap so it can still send a normal GUI on tap but provides window management specific keybinds when held.
enum {
    WM_UP=NEW_SAFE_RANGE,
    WM_LEFT,
    WM_DOWN,
    WM_RGHT,
    NEW_NEW_SAFE_RANGE
};

// clang-format off
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
[LYR_BASE] = LAYOUT_ansi_84(
     KC_ESC,   KC_BRID,  KC_BRIU,  LCA(KC_TAB), KC_F4, LAG(KC_K),  G(KC_N),  KC_MPRV,  KC_MPLY,  KC_MNXT,  KC_MUTE,  KC_VOLD,  KC_VOLU,  OSL(LYR_FN),  KC_DEL,   KC_F13,
     KC_GRV,   KC_1,     KC_2,     KC_3,     KC_4,     KC_5,     KC_6,     KC_7,     KC_8,     KC_9,     KC_0,     KC_MINS,  KC_EQL,   KC_BSPC,            KC_PGUP,
     KC_TAB,   KC_Q,     KC_W,     KC_E,     KC_R,     KC_T,     KC_Y,     KC_U,     KC_I,     KC_O,     KC_P,     KC_LBRC,  KC_RBRC,  KC_BSLS,            KC_PGDN,
     KC_CAPS,  KC_A,     KC_S,     KC_D,     KC_F,     KC_G,     KC_H,     KC_J,     KC_K,     KC_L,     KC_SCLN,  KC_QUOT,            KC_ENT,             KC_HOME,
     OSM(MOD_LSFT),            KC_Z,     KC_X,     KC_C,     KC_V,     KC_B,     KC_N,     KC_M,     KC_COMM,  KC_DOT,   KC_SLSH,            KC_RSFT,  KC_UP,    KC_END,
     OSM(MOD_LCTL),  OSM(MOD_LGUI), OSM(MOD_LALT),                               KC_SPC,                                 KC_RALT,LT(LYR_WM, KC_RGUI),KC_RCTL,  KC_LEFT,  KC_DOWN,  KC_RGHT),

[LYR_FN] = LAYOUT_ansi_84(
     _______,  KC_F1,    KC_F2,    KC_F3,    KC_F4,    KC_F5,    KC_F6,    KC_F7,    KC_F8,    KC_F9,    KC_F10,   KC_F11,   KC_F12,   XXXXXXX,  _______,  OSL(LYR_DEV),
     _______,  BT_HST1,  BT_HST2,  BT_HST3,  P2P4G,    _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,            _______,
     _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,            _______,
     _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,            _______,            _______,
     _______,            _______,  _______,  _______,  _______,  BAT_LVL,  _______,  _______,  _______,  _______,  _______,            _______,  _______,  _______,
     _______,  _______,  _______,                                _______,                                _______,  _______,  _______,  _______,  _______,  _______),

[LYR_WM] = LAYOUT_ansi_84(
     _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______, _______,
     _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______, _______,            _______,
     _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,            _______,
     _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,            _______,            _______,
     _______,            _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,            _______,  WM_UP,  _______,
     _______,  _______,  _______,                                _______,                                _______,  _______,  _______,  WM_LEFT,  WM_DOWN,  WM_RGHT),


[LYR_DEV] = LAYOUT_ansi_84(
    XXXXXXX,  XXXXXXX,    XXXXXXX,    XXXXXXX,    XXXXXXX,    XXXXXXX,    XXXXXXX,    XXXXXXX,    XXXXXXX,    XXXXXXX,    XXXXXXX,  XXXXXXX,   XXXXXXX,   XXXXXXX,  XXXXXXX,  XXXXXXX,
    XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,    XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,            XXXXXXX,
    XXXXXXX,  XXXXXXX,  XXXXXXX,  DB_TOGG,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,            XXXXXXX,
    XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,            XXXXXXX,            XXXXXXX,
    XXXXXXX,            XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  QK_BOOT,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,            XXXXXXX,  XXXXXXX,  XXXXXXX,
    XXXXXXX,  XXXXXXX,  XXXXXXX,                                XXXXXXX,                                XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX,  XXXXXXX)
};

// clang-format on


uint8_t extract_osm_bits(uint16_t keycode) {
    uint8_t result = 0;
    switch(keycode) {
        case OSM(MOD_LCTL):
            result |= MOD_MASK_CTRL;
        break;
        case OSM(MOD_LSFT):
            result |= MOD_MASK_SHIFT;
            break;
        case OSM(MOD_LALT):
            result |= MOD_MASK_ALT;
            break;
        case OSM(MOD_LGUI):
            result |= MOD_MASK_GUI;
            break;
    }
    return result;
}

void unregister_osm_codes(uint8_t modifiers) {
    if (MOD_BIT_LCTRL & modifiers) unregister_code(KC_LCTL);
    if (MOD_BIT_LSHIFT & modifiers) unregister_code(KC_LSFT);
    if (MOD_BIT_LGUI & modifiers) unregister_code(KC_LGUI);
    if (MOD_BIT_LALT & modifiers) unregister_code(KC_LALT);
}

#define WM_SEND_STRING(k) SEND_STRING(SS_DOWN(WM_PSEUDOMODIFIER) SS_TAP(k) SS_UP(WM_PSEUDOMODIFIER));

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    if (!process_record_keychron_common(keycode, record)) {
        return false;
    }

    switch (keycode)
    {
        case QK_ONE_SHOT_MOD ... QK_ONE_SHOT_MOD_MAX:
        {
            uint8_t current_osm_state = get_oneshot_mods();
            uint8_t requested = extract_osm_bits(keycode);
            uint8_t overlap = requested & current_osm_state;
            if (!record->event.pressed && overlap)
            {
                set_oneshot_mods(requested ^ current_osm_state);
                unregister_osm_codes(overlap);
            }
            break;
        }
        case WM_UP:
            WM_SEND_STRING(X_UP);
            break;
        case WM_LEFT:
            WM_SEND_STRING(X_LEFT);
            break;
        case WM_DOWN:
            WM_SEND_STRING(X_DOWN);
            break;
        case WM_RGHT:
            WM_SEND_STRING(X_RGHT);
            break;
    }
    return true;
}

bool rgb_matrix_indicators_advanced_user(uint8_t led_min, uint8_t led_max) {
    uint8_t current_osm_state = get_oneshot_mods();
    for (uint8_t row = 0; row < MATRIX_ROWS; ++row) {
        for (uint8_t col = 0; col < MATRIX_COLS; ++col) {
            uint8_t index = g_led_config.matrix_co[row][col];
            keypos_t key = (keypos_t){col,row};
            uint16_t keycode = keymap_key_to_keycode(layer_switch_get_layer(key), key);
            if (
                index >= led_min
                && index < led_max
                && index != NO_LED
            ) {
                uint8_t active_layer = layer_switch_get_layer((keypos_t){col,row});
                switch (active_layer)
                {
                    case LYR_BASE:
                    case LYR_WM:
                    {
                        if (current_osm_state != 0)
                        {
                            uint8_t implied_mod = extract_osm_bits(keycode);
                            if (implied_mod && (implied_mod & current_osm_state)) {
                                rgb_matrix_set_color(index, ACTIVE_INDICATOR_COLOR);
                            }
                        }
                        break;
                    }
                    case LYR_FN:
                        if (keycode == XXXXXXX) {
                            rgb_matrix_set_color(index, ACTIVE_INDICATOR_COLOR);
                        } else {
                            rgb_matrix_set_color(index, LAYER_INDICATOR_COLOR);
                        }
                        break;
                    case LYR_DEV:
                    {
                        keypos_t key = (keypos_t){col,row};
                        uint16_t keycode = keymap_key_to_keycode(layer_switch_get_layer(key), key);
                        switch (keycode)
                        {
                            case DB_TOGG:
                                if (debug_enable) {
                                    rgb_matrix_set_color(index, 0x00, 0xFF, 0x00);
                                } else {
                                    rgb_matrix_set_color(index, DEV_BUG_OFF_INDICATOR_COLOR);
                                }
                                break;
                            case QK_BOOT:
                                rgb_matrix_set_color(index, LAYER_INDICATOR_COLOR);
                                break;
                            case XXXXXXX:
                                rgb_matrix_set_color(index, DEV_NOOP_INDICATOR_COLOR);
                        }
                        break;
                    }

                }
            }
        }
    }

    if (host_keyboard_led_state().caps_lock)
    {
        rgb_matrix_set_color(CAPS_LOCK_INDEX, ACTIVE_INDICATOR_COLOR);
        rgb_matrix_set_color(CAPS_LOCK_INDEX+1, ACTIVE_INDICATOR_COLOR);
        rgb_matrix_set_color(g_led_config.matrix_co[2][0], ACTIVE_INDICATOR_COLOR);
    }

    return false;
}

// bool led_update_user(led_t led_state)
// {
//     if (host_keyboard_led_state().caps_lock)
//     {
//         rgb_matrix_set_color(CAPS_LOCK_INDEX, ACTIVE_INDICATOR_COLOR);
//         rgb_matrix_set_color(CAPS_LOCK_INDEX+1, ACTIVE_INDICATOR_COLOR);
//         rgb_matrix_set_color(g_led_config.matrix_co[2][0], ACTIVE_INDICATOR_COLOR);
//     }
//     return true;
// }


void keyboard_post_init_user(void) {
}

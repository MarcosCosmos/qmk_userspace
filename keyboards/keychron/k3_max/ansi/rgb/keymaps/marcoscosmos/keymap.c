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

#include QMK_KEYBOARD_H
#include "keychron_common.h"

#define LAYER_INDICATOR_COLOR RGB_GOLDENROD

enum layers {
    MAC_BASE,
    MAC_FN,
    WIN_BASE,
    WIN_FN,
};

// clang-format off
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
[MAC_BASE] = LAYOUT_ansi_84(
     KC_ESC,   KC_BRID,  KC_BRIU,  LCA(KC_TAB), KC_F4, LAG(KC_K),  G(KC_N),  KC_MPRV,  KC_MPLY,  KC_MNXT,  KC_MUTE,  KC_VOLD,  KC_VOLU,  OSL(1),  KC_DEL,   KC_F13,
     KC_GRV,   KC_1,     KC_2,     KC_3,     KC_4,     KC_5,     KC_6,     KC_7,     KC_8,     KC_9,     KC_0,     KC_MINS,  KC_EQL,   KC_BSPC,            KC_PGUP,
     KC_TAB,   KC_Q,     KC_W,     KC_E,     KC_R,     KC_T,     KC_Y,     KC_U,     KC_I,     KC_O,     KC_P,     KC_LBRC,  KC_RBRC,  KC_BSLS,            KC_PGDN,
     KC_CAPS,  KC_A,     KC_S,     KC_D,     KC_F,     KC_G,     KC_H,     KC_J,     KC_K,     KC_L,     KC_SCLN,  KC_QUOT,            KC_ENT,             KC_HOME,
     OSM(MOD_LSFT),            KC_Z,     KC_X,     KC_C,     KC_V,     KC_B,     KC_N,     KC_M,     KC_COMM,  KC_DOT,   KC_SLSH,            KC_RSFT,  KC_UP,    KC_END,
     OSM(MOD_LCTL),  OSM(MOD_LGUI), OSM(MOD_LALT),                               KC_SPC,                                 KC_RALT,KC_RGUI,KC_RCTL,  KC_LEFT,  KC_DOWN,  KC_RGHT),

[MAC_FN] = LAYOUT_ansi_84(
     _______,  KC_F1,    KC_F2,    KC_F3,    KC_F4,    KC_F5,    KC_F6,    KC_F7,    KC_F8,    KC_F9,    KC_F10,   KC_F11,   KC_F12,   _______,  _______,  QK_BOOT,
     _______,  BT_HST1,  BT_HST2,  BT_HST3,  P2P4G,    _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,            _______,
     _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,            _______,
     _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,            _______,            _______,
     _______,            _______,  _______,  _______,  _______,  BAT_LVL,  _______,  _______,  _______,  _______,  _______,            _______,  _______,  _______,
     _______,  _______,  _______,                                _______,                                _______,  _______,  _______,  _______,  _______,  _______),

[WIN_BASE] = LAYOUT_ansi_84(
     KC_ESC,   KC_F1,    KC_F2,    KC_F3,    KC_F4,    KC_F5,    KC_F6,    KC_F7,    KC_F8,    KC_F9,    KC_F10,   KC_F11,   KC_F12,   KC_PSCR,  KC_DEL,   RGB_MOD,
     KC_GRV,   KC_1,     KC_2,     KC_3,     KC_4,     KC_5,     KC_6,     KC_7,     KC_8,     KC_9,     KC_0,     KC_MINS,  KC_EQL,   KC_BSPC,            KC_PGUP,
     KC_TAB,   KC_Q,     KC_W,     KC_E,     KC_R,     KC_T,     KC_Y,     KC_U,     KC_I,     KC_O,     KC_P,     KC_LBRC,  KC_RBRC,  KC_BSLS,            KC_PGDN,
     KC_CAPS,  KC_A,     KC_S,     KC_D,     KC_F,     KC_G,     KC_H,     KC_J,     KC_K,     KC_L,     KC_SCLN,  KC_QUOT,            KC_ENT,             KC_HOME,
     KC_LSFT,            KC_Z,     KC_X,     KC_C,     KC_V,     KC_B,     KC_N,     KC_M,     KC_COMM,  KC_DOT,   KC_SLSH,            KC_RSFT,  KC_UP,    KC_END,
     KC_LCTL,  KC_LGUI,  KC_LALT,                                KC_SPC,                                 KC_RALT, MO(WIN_FN),KC_RCTL,  KC_LEFT,  KC_DOWN,  KC_RGHT),

[WIN_FN] = LAYOUT_ansi_84(
     _______,  KC_BRID,  KC_BRIU,  KC_TASK,  KC_FILE,  RGB_VAD,  RGB_VAI,  KC_MPRV,  KC_MPLY,  KC_MNXT,  KC_MUTE,  KC_VOLD,  KC_VOLU,  _______,  _______,  RGB_TOG,
     _______,  BT_HST1,  BT_HST2,  BT_HST3,  P2P4G,    _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,            _______,
     RGB_TOG,  RGB_MOD,  RGB_VAI,  RGB_HUI,  RGB_SAI,  RGB_SPI,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,            _______,
     _______,  RGB_RMOD, RGB_VAD,  RGB_HUD,  RGB_SAD,  RGB_SPD,  _______,  _______,  _______,  _______,  _______,  _______,            _______,            _______,
     _______,            _______,  _______,  _______,  _______,  BAT_LVL,  NK_TOGG,  _______,  _______,  _______,  _______,            _______,  _______,  _______,
     _______,  _______,  _______,                                _______,                                _______,  _______,  _______,  _______,  _______,  _______)
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

uint8_t stored_osm_state = 0;

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    if (!process_record_keychron_common(keycode, record)) {
        return false;
    }

    switch (keycode)
    {
        case QK_ONE_SHOT_MOD ... QK_ONE_SHOT_MOD_MAX:
        {
            uint8_t requested = extract_osm_bits(keycode);
            uint8_t overlap = requested & stored_osm_state;
            if (!record->event.pressed && overlap)
            {
                set_oneshot_mods(requested ^ stored_osm_state);
                unregister_osm_codes(overlap);
            }
            break;
        }
    }
    return true;
}

void oneshot_mods_changed_user(uint8_t mods) {
    stored_osm_state = get_oneshot_mods();
}

bool rgb_matrix_indicators_advanced_user(uint8_t led_min, uint8_t led_max) {
    if (get_highest_layer(layer_state) > 0) {
        for (uint8_t row = 0; row < MATRIX_ROWS; ++row) {
            for (uint8_t col = 0; col < MATRIX_COLS; ++col) {
                uint8_t index = g_led_config.matrix_co[row][col];
                if (
                    index >= led_min
                    && index < led_max
                    && index != NO_LED
                    && layer_switch_get_layer((keypos_t){col,row}) > 0
                ) {
                    rgb_matrix_set_color(index, LAYER_INDICATOR_COLOR);
                }
            }
        }
    }

    if (host_keyboard_led_state().caps_lock)
    {
        rgb_matrix_set_color(CAPS_LOCK_INDEX, LAYER_INDICATOR_COLOR);
        rgb_matrix_set_color(CAPS_LOCK_INDEX+1, LAYER_INDICATOR_COLOR);
        rgb_matrix_set_color(g_led_config.matrix_co[2][0], LAYER_INDICATOR_COLOR);
        // rgb_matrix_set_color(48, RGB_GREEN);s
    }

    if (stored_osm_state != 0)
    {
        for (uint8_t row = 0; row < MATRIX_ROWS; ++row) {
            for (uint8_t col = 0; col < MATRIX_COLS; ++col) {
                uint8_t index = g_led_config.matrix_co[row][col];
                if (
                    index >= led_min
                    && index < led_max
                    && index != NO_LED
                ) {
                    keypos_t key = (keypos_t){col,row};
                    uint16_t keycode = keymap_key_to_keycode(layer_switch_get_layer(key), key);

                    uint8_t implied_mod = extract_osm_bits(keycode);
                    if (implied_mod && (implied_mod & stored_osm_state)) {
                        rgb_matrix_set_color(index, LAYER_INDICATOR_COLOR);
                    }
                }
            }
        }
    }

    // for (uint8_t row = 0; row < MATRIX_ROWS; ++row) {
    //     for (uint8_t col = 0; col < MATRIX_COLS; ++col) {
    //         uint8_t index = g_led_config.matrix_co[row][col];
    //         rgb_matrix_set_color(index, RGB_GREEN);
    //     }
    // }

    return false;
}

bool led_update_user(led_t led_state)
{
    // if (host_keyboard_led_state().caps_lock) {
    //     rgb_matrix_set_color(CAPS_LOCK_INDEX, 0, 255, 255);
    // }
    return true;
}


void keyboard_post_init_user(void) {
    rgb_matrix_mode_noeeprom(RGB_MATRIX_SOLID_COLOR);
    rgb_matrix_sethsv_noeeprom(HSV_OFF);
}

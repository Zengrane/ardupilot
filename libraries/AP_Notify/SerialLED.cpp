/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "SerialLED.h"
#include "AP_Notify/AP_Notify.h"

#if AP_NOTIFY_SERIALLED_ENABLED

extern const AP_HAL::HAL& hal;

SerialLED::SerialLED(uint8_t led_off, uint8_t led_bright, uint8_t led_medium, uint8_t led_dim) :
    RGBLed(led_off, led_bright, led_medium, led_dim)
{
}

bool SerialLED::init()
{
    enable_mask = init_ports();
    return true;
}

bool SerialLED::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (enable_mask == 0) {
        // nothing is enabled, no pins set as LED output
        return true;
    }

    AP_SerialLED *led = AP_SerialLED::get_singleton();
    if (led == nullptr) {
        return false;
    }

    if (rgb_source() != Source::split_standard) {
        for (uint16_t chan=0; chan<16; chan++) {
            if ((1U<<chan) & enable_mask) {
                led->set_RGB(chan+1, -1, red, green, blue);
            }
        }
    } else {
        // switch red and green for half the LEDs in split standard
        // so that copters can display different armed colours on different sides
        for (uint16_t chan=0; chan<16; chan++) {
            const uint8_t led_len = pNotify->get_led_len();
            if ((1U<<chan) & enable_mask) {
                for (uint8_t i = 0; i < led_len/2; i++) {
                    led->set_RGB(chan+1, i, red, green, blue);
                }
                for (uint8_t i = led_len/2; i < led_len; i++) {
                    led->set_RGB(chan+1, i, green, red, blue);
                }
            }
        }
    }

    for (uint16_t chan=0; chan<16; chan++) {
        if ((1U<<chan) & enable_mask) {
            led->send(chan+1);
        }
    }

    return true;
}

#endif  // AP_NOTIFY_SERIALLED_ENABLED

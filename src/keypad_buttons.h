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

    Lorenzo Miori (C) 2016-2017 [ 3M4|L: memoryS60<at>gmail.com ]

    Version History
        * 1.0 initial

*/

#ifndef KEYPAD_KEYPAD_BUTTONS_TEMPLATE_H_
#define KEYPAD_KEYPAD_BUTTONS_TEMPLATE_H_

#define KEY_DEBOUNCE_CLICK    1U       /**< Debounce time [debounce cycles] */
#define KEY_DEBOUNCE_HOLD     75U      /**< Debounce time [debounce cycles] */

/**< Enumeration of buttons */
typedef enum e_buttons_
{
    BUTTON_POWER,
    BUTTON_EJECT,
    BUTTON_FFWD,
    BUTTON_REW,
    BUTTON_PLAY,
    BUTTON_STOP,
    /*
    BUTTON_LEFT,
    BUTTON_RIGHT,

    add all your buttons that are required to tailor
    the specific application requirements

    */
    /** This entry MUST be there and be the last one */
    NUM_BUTTONS
} e_key;

#endif /* KEYPAD_KEYPAD_BUTTONS_TEMPLATE_H_ */

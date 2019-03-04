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

    Lorenzo Miori (C) 2016 [ 3M4|L: memoryS60<at>gmail.com ]

    Version History
        * 1.0 initial

*/

/**
 * @file configuration.h
 * @author Lorenzo Miori
 * @date Aug 2016
 * @brief Display configuration
 */

#ifndef SRC_DEASPLAY_H_
#define SRC_DEASPLAY_CONFIGURATION_H_

/** Enable this to have the full fledged printf facility
 * NOTE: this might take up to 2kB of code memory! */
/* #define DISPLAY_HAS_PRINTF */

#define DEASPLAY_PT6311

#define DEASPLAY_LINES 1
#define DEASPLAY_CHARS 10

#include "deasplay-pt6311/pt6311.h"

#endif /* SRC_DEASPLAY_H_ */

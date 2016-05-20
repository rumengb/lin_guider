/*
 * maindef.cpp
 *
 *      Author: gm
 *
 *
 * This file is part of Lin_guider.
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "maindef.h"

// define some colors
const u_char DEF_BKGD_COLOR[3] 		= {0, 0, 0};
const u_char DEF_RA_COLOR[3]		= {255, 128, 0};
const u_char DEF_DEC_COLOR[3]		= {0, 165, 255};
const u_char DEF_GRID_COLOR[3]		= {128, 128, 128};
const u_char DEF_WHITE_COLOR[3]		= {255, 255, 255};
const u_char DEF_GRID_FONT_COLOR[3]	= {0, 255, 128};
const u_char DEF_RET_ORG_COLOR[3]   = {255, 0, 0};
const u_char DEF_OSF_COLOR[3]       = {255, 0, 255};

const u_char DEF_SQR_OVL_COLOR[3]	= {0, 255, 0};

bool DBG_VERBOSITY = false;

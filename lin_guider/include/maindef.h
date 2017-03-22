/*
 * maindef.h
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

#ifndef MAINDEF_H_
#define MAINDEF_H_

#include <sys/types.h>

#define APP_NAME "Lin-guider"
#define VERSION "4.2.0"
#define CPY_RIGHT(ver) APP_NAME " v" ver " (c)GM software'08-17"

typedef struct
{
	int x, y;
}point_t;

extern const u_char DEF_BKGD_COLOR[];
extern const u_char DEF_RA_COLOR[];
extern const u_char DEF_DEC_COLOR[];
extern const u_char DEF_GRID_COLOR[];
extern const u_char DEF_WHITE_COLOR[];
extern const u_char DEF_GRID_FONT_COLOR[];
extern const u_char DEF_RET_ORG_COLOR[];
extern const u_char DEF_OSF_COLOR[];

extern const u_char DEF_SQR_OVL_COLOR[];

static const int LOOP_DELAY	= 10000;

extern bool DBG_VERBOSITY;

#endif /*MAINDEF_H_*/

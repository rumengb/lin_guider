/*
 * fio.h
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

#ifndef FIO_H_
#define FIO_H_

#include <stdio.h>

#define FIO_OK		0
#define FIO_EXIST	1
#define FIO_ERROR	2

class fio
{
public:
	fio();
	~fio();

	bool start( const char *fname );
	bool stop( void );
	static int  check_file_name( const char *fname );
	bool add_drift( double ra, double dec );
private:
	FILE *out;
	bool initialized;
};

#endif /*FIO_H_*/

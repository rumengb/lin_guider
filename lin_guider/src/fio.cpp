/*
 * fio.cpp
 *
 *      Author: gm
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

#include <unistd.h>
#include "fio.h"
#include "utils.h"


fio::fio()
{
	out = NULL;
	initialized = false;
}


fio::~fio()
{
	stop();
}


bool fio::start( const char *fname )
{
	if( initialized )
		return false;
	
	out = fopen( fname, "wt" );
	if( out == NULL )
	{
		log_e( "fio::start: Unable to open file '%s\n'", fname );
		return false;
	}
	
	initialized = true;
	
 return true;
}


bool fio::stop( void )
{
	if( !initialized )
		return false;
	
	if( out )
	{
		fclose( out );
		out = NULL;
	}
	
	initialized = false;
	
 return true;
}


int fio::check_file_name( const char *fname )
{
 int result;
 FILE *tmp;
 
 	result = access( fname, F_OK );
 	if( result == 0 )
 		return FIO_EXIST;
 	
 	tmp = fopen( fname, "wt" );
 	if( tmp == NULL )
 		return FIO_ERROR;
 	fclose( tmp );
 	
 	unlink( fname );
 	
 return FIO_OK;
}


bool fio::add_drift( double ra, double dec )
{
	if( !initialized )
		return false;
	
	fprintf( out, "%.2lf %.2lf\n", ra, dec );
	
 return true;
}

/*
 * utils.cpp
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

#include <string.h>
#include <stdarg.h>
#include <unistd.h>

#include <string>
#include <algorithm>

#include <QFileInfo>

#include "utils.h"


void u_msg( const char *fmt, ...)
{
/*
		va_list args;
		char buf[1024];

		va_start(args, fmt);
		int ret = vsnprintf( buf, sizeof(buf)-1, fmt, args );
		va_end(args);
*/
        va_list     argptr;
        QString     text;

        va_start (argptr,fmt);
        text.vsprintf(fmt, argptr);
        va_end (argptr);

        QMessageBox::information( NULL, "Info...", text, QMessageBox::Ok, QMessageBox::Ok );

        //QMessageBox::information( this, "Info...", QString().sprintf("test = %d", 13), QMessageBox::Ok, QMessageBox::Ok );
}


bool u_yes( const QString &question )
{
	if( QMessageBox::question( NULL, "Question", question, QMessageBox::Yes | QMessageBox::No ) == QMessageBox::Yes )
	    return true;

 return false;
}


double u_pow( double a, int n )
{
 int i;
 double res = 1;

       if( n <= 0 )
           return 1;

       for( i = 0;i < n;i++ )
            res *= a;

 return res;
}


void log_msg( FILE *stm, const char *fmt, ... )
{
 va_list	argptr;
 struct timespec time;
 //char		text[];

 clock_gettime( CLOCK_REALTIME, &time );
 fprintf( stm, "[%ld:%.3ld] ", time.tv_sec, time.tv_nsec/1000000 );

 	va_start( argptr, fmt );
	vfprintf( stm, fmt, argptr);
	va_end (argptr);

	fflush( stm );

}




uint32_t u_rgb( unsigned char r, unsigned char g, unsigned char b )
{
 uint32_t c = 0x0;
 uint32_t mask_r, mask_g, mask_b;

       mask_b = (uint32_t)b;
       mask_b <<= 16;
       mask_g = (uint32_t)g;
       mask_g <<= 8;
       mask_r = (uint32_t)r;

       // b
       c |= mask_b;
       // g
       c |= mask_g;
       // r
       c |= mask_r;

 return c;
}

QString u_extract_file_ext( QString fname )
{
/*
 char *delim;

	delim = strrchr( fname.toAscii().data(), '.' );
	if( delim != NULL )
		return QString( delim+1 );

 return QString("");
 */
	QFileInfo fi( fname );
	QString ext = fi.suffix();
	if( ext != QString("") )
		ext = QString(".") + ext;

 return ext;
}


QString u_extract_file_name( QString fname )
{
	/*
 char *delim, *str;
 int len = 0;

 	str = fname.toAscii().data();
	delim = strrchr( str, '/' );
	if( delim != NULL )
	{
		len = strlen( str );
		if( delim - str < len )
			return QString( delim+1 );
	}

	return QString("");
	*/
	QFileInfo fi( fname );
	QString name = fi.fileName();

 return name;
}


bool u_file_exists( const char *fname )
{
 int result;

	result = access( fname, F_OK );

 return result ? false : true;
}


QString u_get_app_path( void )
{
	int length;
	char fullpath[PATH_MAX];

	/* /proc/self is a symbolic link to the process-ID subdir
	 * of /proc, e.g. /proc/4323 when the pid of the process
	 * of this program is 4323.
	 *
	 * Inside /proc/<pid> there is a symbolic link to the
	 * executable that is running as this <pid>.  This symbolic
	 * link is called "exe".
	 *
	 * So if we read the path where the symlink /proc/self/exe
	 * points to we have the full path of the executable.
	 */
	length = readlink("/proc/self/exe", fullpath, sizeof(fullpath));

	/* Catch some errors: */
	if( length < 0 )
	{
		log_e("Error resolving symlink /proc/self/exe.\n");
		exit(EXIT_FAILURE);
	}
	if( length >= PATH_MAX )
	{
		log_e("Path too long. Truncated.\n");
		exit(EXIT_FAILURE);
	}
	// finish string
	fullpath[length] = '\0';

	QString fp = QString(fullpath);

	QString path = QFileInfo( fp ).path();

 return path;
}


int u_memtok( const char *str, unsigned slen, char c, const char **val, unsigned *vlen, unsigned *pos )
{
    const char *end = NULL, *v = NULL;
    unsigned l = 0;
    unsigned p = pos ? *pos : 0;

    if( p >= slen )
        return 0;

    v = str + p;
    if( (end = reinterpret_cast< const char* >( memchr(v, c, slen - p) )) )
    {
        l = end - v;
        if( pos )
            *pos += l + 1;
    }
    else
    {
        l = slen - p;
        if( pos )
            *pos += l;
    }
    if( val )
        *val = v;
    if( vlen )
        *vlen = l;

    return 1;
}


bool u_make_safe_str( const char* mem, size_t mem_sz, size_t str_sz_max, char *str, size_t *str_len )
{
	if( !mem || !mem_sz || !str_sz_max || !str || !str_len )
		return false;

	*str_len = std::min( mem_sz, str_sz_max-1 );
	memcpy( str, mem, *str_len );
	str[ *str_len ] = '\0';

	return true;
}


unsigned int u_jshash( const std::string& str )
{
	unsigned int hash = 1315423911;
	size_t length = str.length();
	const char *p = str.data();

	for( size_t i = 0;i < length;i++ )
		hash ^= ((hash << 5) + p[i] + (hash >> 2));

	return hash;
}

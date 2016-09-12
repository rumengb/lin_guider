/*
 * utils.h
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

//---------------------------------------------------------------------------

#ifndef UtilsH
#define UtilsH
#include <limits.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

#include <string>

#include <Qt>
#include <QMessageBox>

#undef MIN
#undef MAX

#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))

#define ARRAY_SIZE(a)		(sizeof(a) / sizeof((a)[0]))

#define HIWORD(x) ((unsigned short)((uint32_t)(x) >> 16))
#define LOWORD(x) ((unsigned short)((uint32_t)(x) & 0xffff))
#define MAKELONG(l, h) ((uint32_t) (((unsigned short) (l)) | ((uint32_t) ((unsigned short) (h))) << 16))

// c-strings misc.
#define STRSZ(str)      (str), (sizeof(str)-1)
#define STRLN(str)      (sizeof(str)-1)

bool u_yes( const QString &question );
void u_msg( const char *fmt, ...);
//void U_Error( char *fmt, ...);

double u_pow( double a, int n );
void log_msg( FILE *stm, const char *fmt, ... );
#define log_i( fmt, ... )	log_msg( stdout, fmt "\n", ##__VA_ARGS__ )
#define log_e( fmt, ... )	log_msg( stderr, "ERROR: " fmt "\n", ##__VA_ARGS__ )

uint32_t u_rgb( unsigned char r, unsigned char g, unsigned char b );
QString u_extract_file_ext( QString fname );
QString u_extract_file_name( QString fname );
bool u_file_exists( const char *fname );
QString u_get_app_path( void );
int u_memtok( const char *str, unsigned slen, char c, const char **val, unsigned *vlen, unsigned *pos );
bool u_make_safe_str( const char* mem, size_t mem_sz, size_t str_sz_max, char *str, size_t *str_len );
unsigned int u_jshash( const std::string& str );

#endif

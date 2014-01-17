/*
 * lusb.h
 *
 *  Created on: 23.05.2011
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

#ifndef LUSB_H_
#define LUSB_H_

class lusb
{
public:
	static int  initialize( void );
	static int  release( void );
	static bool is_initialized( void );
private:
	static int m_init_cnt;
};

#endif /* LUSB_H_ */

/*
 * vect.cpp
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
#include <math.h>
#include <stdlib.h>
#include "vect.h"

//---------------------------------------------------------------------------


Vector operator ^ (const Vector& u, const Vector& v)
{
  return Vector ( u.y*v.z - u.z*v.y,
                  u.z*v.x - u.x*v.z,
                  u.x*v.y - u.y*v.x
                );
}

Vector RndVector ()
{
 Vector v (rand() - 0.5*RAND_MAX, rand() - 0.5*RAND_MAX,
           rand() - 0.5*RAND_MAX
          );
 return Normalize( v );         
}

Vector& Clip( Vector& v)
{
  if( v.x < 0.0 ) v.x = 0.0;
  else
      if( v.x > 1.0 ) v.x = 1.0;
  if( v.y < 0.0 ) v.y = 0.0;
  else
      if( v.y > 1.0 ) v.y = 1.0;
  if( v.z < 0.0 ) v.z = 0.0;
  else
      if( v.z > 1.0 ) v.z = 1.0;

  return v;
}









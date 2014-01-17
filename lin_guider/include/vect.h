/*
 * vect.h
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
#ifndef vectH
#define vectH

#include <math.h>

class Vector
{
 public:
 double x,y,z;
 Vector () { x = y = z = 0.0; };
 Vector ( double v ) { x = y = z = v; };
 Vector ( const Vector& v ) { x = v.x; y = v.y; z = v.z; };
 Vector ( double vx, double vy, double vz ) { x = vx; y = vy; z = vz; };
 ~Vector(){};

 Vector& operator = (const Vector& v) {x = v.x;
                                       y = v.y;
                                       z = v.z;
                                       return *this;
                                      };
 Vector& operator = (double f) {x = y = z = f;
                                return *this;
                               };
 Vector  operator - () const;
 Vector& operator += ( const Vector& );
 Vector& operator -= ( const Vector& );
 Vector& operator *= ( const Vector& );
 Vector& operator *= ( double );
 Vector& operator /= ( double );

 friend Vector operator + ( const Vector&, const Vector& );
 friend Vector operator - ( const Vector&, const Vector& );
 friend Vector operator * ( const Vector&, const Vector& );
 friend Vector operator * ( double, const Vector& );
 friend Vector operator * ( const Vector&, double );
 friend Vector operator / ( const Vector&, double );
 friend Vector operator / ( const Vector&, const Vector& );
 friend double operator & ( const Vector& u, const Vector& v )
                          {
                            return u.x*v.x + u.y*v.y + u.z*v.z;
                          };
 friend Vector operator ^ ( const Vector&, const Vector& );
 double operator ! () const {return (double)sqrt(x*x+y*y+z*z);};
 double& operator [] (int n) {return * (&x + n);};
 int operator < (double v) {return x<v && y<v && z<v;};
 int operator > (double v) {return x>v && y>v && z>v;};
};

class Ray
{
 public:
   Vector Org;
   Vector Dir; // Must be normalized
   Ray () {};
   Ray& operator = (const Ray& r) {Org = r.Org;
                                   Dir = r.Dir;
                                   return *this;
                                  };

   Ray ( Vector& o, Vector& d ) { Org = o; Dir = d;};
   ~Ray(){};
  Vector Point ( double t ) {return Org + Dir*t;};
};

inline Vector Vector :: operator - () const
{
  return Vector( -x, -y, -z );
}

inline Vector operator + (const Vector& u, const Vector& v)
{
  return Vector( u.x+v.x, u.y+v.y, u.z+v.z );
}

inline Vector operator - (const Vector& u, const Vector& v)
{
  return Vector( u.x-v.x, u.y-v.y, u.z-v.z );
}

inline Vector operator * (const Vector& u, const Vector& v)
{
  return Vector( u.x*v.x, u.y*v.y, u.z*v.z );
}

inline Vector operator * (const Vector& u, double f)
{
  return Vector( u.x*f, u.y*f, u.z*f );
}

inline Vector operator * (double f, const Vector& v)
{
  return Vector( f*v.x, f*v.y, f*v.z );
}

inline Vector operator / (const Vector& v, double f)
{
  return Vector( v.x/f, v.y/f, v.z/f );
}

inline Vector operator / (const Vector& u, const Vector& v)
{
  return Vector( u.x/v.x, u.y/v.y, u.z/v.z );
}

inline Vector& Vector :: operator += (const Vector& v)
{
  x += v.x;
  y += v.y;
  z += v.z;
  return *this;
}

inline Vector& Vector :: operator -= (const Vector& v)
{
  x -= v.x;
  y -= v.y;
  z -= v.z;
  return *this;
}

inline Vector& Vector :: operator *= (double v)
{
  x *= v;
  y *= v;
  z *= v;
  return *this;
}

inline Vector& Vector :: operator *= (const Vector& v)
{
  x *= v.x;
  y *= v.y;
  z *= v.z;
  return *this;
}

inline Vector& Vector :: operator /= (double v)
{
  x /= v;
  y /= v;
  z /= v;
  return *this;
}

////////
inline Vector Normalize ( const Vector& v ) {return v / !v; };
Vector RndVector ();
Vector& Clip ( Vector& );
//---------------------------------------------------------------------------
#endif

/*
 * matr.h
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
#ifndef matrH
#define matrH

#include "vect.h"

class Matrix
{
public:
  double x [4][4];
  Matrix();
  Matrix( double );
  Matrix& operator  = ( const Matrix& );
  Matrix& operator += ( const Matrix& );
  Matrix& operator -= ( const Matrix& );
  Matrix& operator *= ( const Matrix& );
  Matrix& operator *= ( double );
  Matrix& operator /= ( double );
  void Invert ();
  void Transpose ();
  friend Matrix operator + (const Matrix&, const Matrix&);
  friend Matrix operator - (const Matrix&, const Matrix&);
  friend Matrix operator * (const Matrix&, double);
  friend Matrix operator * (const Matrix&, const Matrix&);
  friend Vector operator * (const Vector&, const Matrix&);

};

Matrix Translate( const Vector& );
Matrix Scale( const Vector&);
Matrix RotateX(double);
Matrix RotateY(double);
Matrix RotateZ(double);
Matrix Rotate(const Vector& v, double angle);
Matrix Transform(const Vector& v1, const Vector& v2, const Vector& v3);
Matrix MirrorX();
Matrix MirrorY();
Matrix MirrorZ();
//---------------------------------------------------------------------------
#endif

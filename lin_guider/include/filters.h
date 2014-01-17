/*
 * filters.h
 *
 *  Created on: 21.06.2012
 *      Author: gm
 *  Original code from: http://www.librow.com
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

#ifndef FILTERS_H_
#define FILTERS_H_

class filters
{
public:
	//   1D MEDIAN FILTER wrapper
	//     signal - input signal
	//     result - output signal
	//     N      - length of the signal
	template <typename T>
	static void medianfilter( T* signal, T* result, int N )
	{
	   //   Check arguments
	   if( !signal || N < 1 )
	      return;
	   //   Treat special case N = 1
	   if( N == 1 )
	   {
	      if( result )
	         result[0] = signal[0];
	      return;
	   }
	   //   Allocate memory for signal extension
	   T* extension = new T[N + 4];
	   //   Check memory allocation
	   if( !extension )
	      return;
	   //   Create signal extension
	   memcpy( extension + 2, signal, N * sizeof(T) );
	   for( int i = 0; i < 2; ++i )
	   {
	      extension[i] = signal[1 - i];
	      extension[N + 2 + i] = signal[N - 1 - i];
	   }
	   //   Call median filter implementation
	   medianfilter_1d( extension, result ? result : signal, N + 4 );
	   //   Free memory
	   delete[] extension;
	}

	//   2D MEDIAN FILTER wrapper
	//     image  - input image
	//     result - output image
	//     N      - width of the image
	//     M      - height of the image
	template <typename T>
	static void medianfilter( T* image, T* result, int N, int M )
	{
		//   Check arguments
		if( !image || N < 1 || M < 1 )
			return;

		//   Allocate memory for signal extension
		T* extension = new T[(N + 2) * (M + 2)];
		//   Check memory allocation
		if( !extension )
			return;
		//   Create image extension
		for( int i = 0;i < M;i++ )
		{
			memcpy( extension + (N + 2) * (i + 1) + 1, image + N * i, N * sizeof(T) );
			extension[(N + 2) * (i + 1)] = image[N * i];
			extension[(N + 2) * (i + 2) - 1] = image[N * (i + 1) - 1];
		}
		//   Fill first line of image extension
		memcpy( extension, extension + N + 2, (N + 2) * sizeof(T) );
		//   Fill last line of image extension
		memcpy( extension + (N + 2) * (M + 1), extension + (N + 2) * M, (N + 2) * sizeof(T) );
		//   Call median filter implementation
		medianfilter_2d( extension, result ? result : image, N + 2, M + 2 );
		//   Free memory
		delete[] extension;
	}

private:
	//   1D MEDIAN FILTER implementation
	//     signal - input signal
	//     result - output signal
	//     N      - length of the signal
	template <typename T>
	static void medianfilter_1d( const T* signal, T* result, int N )
	{
	   //   Move window through all elements of the signal
	   for( int i = 2; i < N - 2; ++i )
	   {
	      //   Pick up window elements
	      T window[5];
	      for( int j = 0; j < 5; ++j )
	    	  window[j] = signal[i - 2 + j];
	      //   Order elements (only half of them)
	      for( int j = 0; j < 3; ++j )
	      {
	         //   Find position of minimum element
	         int min = j;
	         for( int k = j + 1; k < 5; ++k )
	            if( window[k] < window[min] )
	               min = k;
	         //   Put found minimum element in its place
	         const T temp = window[j];
	         window[j] = window[min];
	         window[min] = temp;
	      }
	      //   Get result - the middle element
	      result[i - 2] = window[2];
	   }
	}

	//   2D MEDIAN FILTER implementation
	//     image  - input image
	//     result - output image
	//     N      - width of the image
	//     M      - height of the image
	template <typename T>
	static void medianfilter_2d( const T* image, T* result, int N, int M )
	{
		//   Move window through all elements of the image
		for( int m = 1; m < M - 1; ++m )
		{
			for( int n = 1; n < N - 1; ++n )
			{
				//   Pick up window elements
				int k = 0;
				T window[9];
				for( int j = m - 1; j < m + 2; ++j )
					for( int i = n - 1; i < n + 2; ++i )
						window[k++] = image[j * N + i];
				//   Order elements (only half of them)
				for( int j = 0;j < 5;++j )
				{
					//   Find position of minimum element
					int min = j;
					for( int l = j + 1; l < 9; ++l )
						if( window[l] < window[min] )
							min = l;
					//   Put found minimum element in its place
					const T temp = window[j];
					window[j] = window[min];
					window[min] = temp;
				}
				//   Get result - the middle element
				result[(m - 1) * (N - 2) + n - 1] = window[4];
			}
		}
	}

};


#endif /* FILTERS_H_ */

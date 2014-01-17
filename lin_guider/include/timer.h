/*
 * timer.h
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

#ifndef TIMER_H_
#define TIMER_H_

#include <time.h>
#include <sys/time.h>
#include <sys/types.h>


class ctimer
{
 public:
	 
	 ctimer()
     {
		 start();
     };
     ~ctimer(){};

     void start( void )
     {
    	 clock_gettime( CLOCK_REALTIME, &tp1 );
     }
     unsigned long gettime( void )
     {
    	 clock_gettime( CLOCK_REALTIME, &tp2 );
    	  
    	 //res = diff( &tp1, &tp2 );
    	 //msec = res.tv_sec*1000 + res.tv_nsec*1e-6;
    	 msec = 1000*(tp2.tv_sec - tp1.tv_sec) + (tp2.tv_nsec - tp1.tv_nsec)*(1e-6); 
         return msec;
     }
      
 private:
 	 struct timespec tp1, tp2, res;
 	unsigned long msec;
 	 
 	 timespec diff(timespec *start, timespec *end)
 	 {
 	 	timespec temp;
 	 	if( (end->tv_nsec-start->tv_nsec) < 0 ) 
 	 	{
 	 		temp.tv_sec  = end->tv_sec-start->tv_sec-1;
 	 		temp.tv_nsec = 1000000000+end->tv_nsec-start->tv_nsec;
 	 	} else 
 	 	{
 	 		temp.tv_sec  = end->tv_sec-start->tv_sec;
 	 		temp.tv_nsec = end->tv_nsec-start->tv_nsec;
 	 	}
 	 	return temp;
 	 }      
      
};


#endif /*TIMER_H_*/

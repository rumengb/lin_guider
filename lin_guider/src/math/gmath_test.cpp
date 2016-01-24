/*
 * cgmath_test.cpp
 *
 *      Author: gm
 */

#include <stdlib.h>

#include "gmath.h"
#include "common.h"
#include "vect.h"
#include "utils.h"
#include "gmath_test.h"


cgmath_test::cgmath_test( const common_params &comm_params ) :
	cgmath( comm_params )
{

}


cgmath_test::~cgmath_test()
{
}


Vector cgmath_test::find_star_local_pos( void ) const
{
	int wd, ht;
	const double *data = get_data_buffer( &wd, &ht, NULL, NULL );
	double r_x, r_y;
	get_reticle_params( &r_x, &r_y, NULL );

	double x, y;
	x = r_x + rand()%10 - 5;
	y = r_y + rand()%10 - 5;
	if( x < 0 ) x = 0;
	if( x > (double)wd-1 ) x = wd-1;
	if( y < 0 ) y = 0;
	if( y > (double)ht-1 ) y = ht-1;

	return Vector( x, y, 0 );
}


void cgmath_test::on_start( void )
{
	log_i( "cgmath_test::%s", __FUNCTION__ );
}


void cgmath_test::on_stop( void )
{
	log_i( "cgmath_test::%s", __FUNCTION__ );
}

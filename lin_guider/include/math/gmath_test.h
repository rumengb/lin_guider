/*
 * cgmath_test.h
 *
 *      Author: gm
 */

#ifndef GMATH_TEST_H_
#define GMATH_TEST_H_

#include "gmath.h"


class common_params;

class cgmath_test : public cgmath
{
public:
	cgmath_test( const common_params &comm_params );
	virtual ~cgmath_test();

protected:
	virtual Vector find_star_local_pos( void ) const;
	virtual void on_start( void );
	virtual void on_stop( void );
};

#endif /* GMATH_TEST_H_ */

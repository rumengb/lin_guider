/*
 * cgmath_helper.h
 *
 *      Author:  Author: gm
 */

#ifndef GMATH_HELPER_H_
#define GMATH_HELPER_H_

#include "gmath.h"


namespace lg_math
{

class cgmath_helper : public cgmath
{
public:
	cgmath_helper( const common_params &comm_params ) :
	    cgmath( comm_params )
	{}
	virtual ~cgmath_helper()
	{}

protected:
    void copy_subframe(double *subframe, int x_offset, int y_offset, int sf_width, int sf_height) const
    {
        int i;
        int ci = 0;
        int li = 0;
 
        if (!subframe) return;
        int video_width, len;
        double *pdata = get_data_buffer( &video_width, NULL, &len, NULL );
        int max = sf_width * sf_height;
        for(i=0; i < max; i++) {
            subframe[i] = pdata[ video_width * (y_offset + li) + x_offset + ci ];
            ci++;
            if (ci == sf_width) {
                ci = 0;
                li++;
            }
        }
    }

};

}

#endif /* GMATH_HELPER_H_ */

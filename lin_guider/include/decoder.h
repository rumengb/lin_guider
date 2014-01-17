/*
 * decoder.h
 *
 *       Author: gm
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

#ifndef DECODER_H_
#define DECODER_H_

void convert_yuv420p_to_rgb32(unsigned int width, unsigned int height,
	    						const unsigned char *py,
	    						const unsigned char *pu,
	    						const unsigned char *pv,
	    						unsigned char *d);

unsigned int convert_yuv422_to_rgb32(unsigned char * input_ptr, unsigned char * output_ptr, unsigned int image_width, unsigned int image_height);

void init_decoder(void);
void free_decoder(void);

//////////////////////////////////////////////////////////////////////////////
#define ERR_NO_SOI 1
#define ERR_NOT_8BIT 2
#define ERR_HEIGHT_MISMATCH 3
#define ERR_WIDTH_MISMATCH 4
#define ERR_BAD_WIDTH_OR_HEIGHT 5
#define ERR_TOO_MANY_COMPPS 6
#define ERR_ILLEGAL_HV 7
#define ERR_QUANT_TABLE_SELECTOR 8
#define ERR_NOT_YCBCR_221111 9
#define ERR_UNKNOWN_CID_IN_SCAN 10
#define ERR_NOT_SEQUENTIAL_DCT 11
#define ERR_WRONG_MARKER 12
#define ERR_NO_EOI 13
#define ERR_BAD_TABLES 14
#define ERR_DEPTH_MISMATCH 15

int jpeg_decode(unsigned char **pic, unsigned char *buf, int *width,
		int *height);
int
get_picture(unsigned char *buf,int size);
int
get_pictureYV2(unsigned char *buf,int width,int height);



#endif /*DECODER_H_*/

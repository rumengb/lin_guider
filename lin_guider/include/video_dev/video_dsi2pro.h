/*
 * video_dsi2pro.h
 *
 *  Created on: 24.05.2011
 *      Author: gm
 *
 *  Original DSI2PRO source code by Maxim Parygin
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

#ifndef VIDEO_DSI2PRO_H_
#define VIDEO_DSI2PRO_H_

#include <libusb-1.0/libusb.h>

#include "video.h"

namespace video_drv
{

// Endpoints
#define EP_IN 0x01
#define EP_OUT 0x81
#define EP_DATA 0x86

// ACK & NACK
#define RES_ACK 0x06
#define RES_NAK 0x15

// Commands
#define CMD_PING 0x00				//
#define CMD_RESET 0x01
#define CMD_ABORT 0x02
#define CMD_TRIGGER 0x03
#define CMD_CLEAR_TS 0x04

#define CMD_GET_VERSION 0x14
#define CMD_GET_STATUS 0x15
#define CMD_GET_TIMESTAMP 0x16
#define CMD_GET_EEPROM_LENGTH 0x1e
#define CMD_GET_EEPROM_BYTE 0x1f

#define CMD_SET_EEPROM_BYTE 0x20

#define CMD_SET_GAIN 0x33                        //
#define CMD_GET_OFFSET 0x34                      //
#define CMD_SET_OFFSET 0x35                      //
#define CMD_GET_EXP_TIME 0x36                    //
#define CMD_SET_EXP_TIME 0x37                    //
#define CMD_GET_EXP_MODE 0x38                    //
#define CMD_SET_EXP_MODE 0x39                    //
#define CMD_GET_VDD_MODE 0x3a                    //
#define CMD_SET_VDD_MODE 0x3b                    //
#define CMD_SET_FLUSH_MODE 0x3d                  //
#define CMD_GET_CLEAN_MODE 0x3e                  //
#define CMD_SET_CLEAN_MODE 0x3f                  //
#define CMD_GET_READOUT_SPD 0x40                 //
#define CMD_GET_READOUT_MODE 0x42                //
#define CMD_GET_NORM_READOUT_DELAY 0x44          //
#define CMD_GET_TEMP 0x4a                        //
#define CMD_GET_EXP_TIMER_COUNT 0x4b

#define CMD_AD_READ 0x68
#define CMD_AD_WRITE 0x69
#define CMD_CCD_VDD_OFF 0x67                     //
#define CMD_CCD_VDD_ON 0x66                      //
#define CMD_ERASE_EEPROM 110
#define CMD_GET_DEBUG_VALUE 0x6b
#define CMD_GET_EEPROM_VIDPID 0x6c
#define CMD_GET_FLUSH_MODE 60                    //
#define CMD_GET_GAIN 50                          //
#define CMD_GET_ROW_COUNT_EVEN 0x48              //
#define CMD_GET_ROW_COUNT_ODD 70                 //
#define CMD_PS_OFF 0x65                          //
#define CMD_PS_ON 100                            //
#define CMD_SET_EEPROM_VIDPID 0x6d
#define CMD_SET_NORM_READOUT_DELAY 0x45          //
#define CMD_SET_READOUT_MODE 0x43                //
#define CMD_SET_READOUT_SPD 0x41                 //
#define CMD_SET_ROW_COUNT_EVEN 0x49              //
#define CMD_SET_ROW_COUNT_ODD 0x47               //
#define CMD_TEST_PATTERN 0x6a

// Parameters values
#define VDD_MODE_AUTOMATIC 0x00
#define VDD_MODE_ALWAYS_ON 0x01
#define VDD_MODE_ALWAYS_OFF 0x02

#define READOUT_MODE_DUAL_EXPOSURE 0x00
#define READOUT_MODE_SINGLE_EXPOSURE 0x01
#define READOUT_MODE_ODD_FIELDS 0x02
#define READOUT_MODE_EVEN_FIELDS 0x03

#define FLUSH_MODE_CONTINUOUS 0x00
#define FLUSH_MODE_BEFORE_EXPOSURE 0x01
#define FLUSH_MODE_BEFORE_NEVER 0x02

#define EXPOSURE_MODE_SINGLE 0x00
#define EXPOSURE_MODE_CONTINUOUS 0x01

#define CLEAN_MODE_ENABLED 0x00
#define CLEAN_MODE_DISABLED 0x01

#define READOUT_SPEED_NORMAL 0x00
#define READOUT_SPEED_HIGH 0x01


// HW properties
#define VID 0x156C
#define PID 0x0101

#define DSI2PRO_IMG_WIDTH 752
#define DSI2PRO_IMG_HEIGHT 582
#define IMG_BPP	16
#define IMG_EVEN 299
#define IMG_ODD 298
#define IMG_CHUNK 2048
#define IMG_CHUKN_EVEN (IMG_CHUNK*IMG_EVEN)
#define IMG_CHUKN_ODD (IMG_CHUNK*IMG_ODD)


typedef struct internal_params
{
	int cmd;
	int arg;
	int in;
	int out;
	int response; // uses as internal response
}internal_params_t;


/*
 * dsi2pro camera
 */
class cvideo_dsi2pro : public cvideo_base
{
public:
	cvideo_dsi2pro();
	virtual ~cvideo_dsi2pro();

	virtual time_fract_t set_fps( const time_fract_t &new_fps );

protected:
	virtual int open_device( void );		// open device
	virtual int close_device( void );		// close device
	virtual int get_vcaps( void );

	virtual int set_control( unsigned int control_id, const param_val_t &val );
	virtual int get_control( unsigned int control_id, param_val_t *val );

private:
	virtual int init_device( void );		// get&check capabilities, apply format
	virtual int uninit_device( void );		// deinit device
	virtual int start_capturing( void );	// turn on stream
	virtual int stop_capturing( void );		// stop stream
	virtual int read_frame( void );			// read frame
	virtual int set_format( void );

	virtual int enum_controls( void );

private:
	bool send_command( const char *cmd, int cmd_len, char *param, int param_len );

	struct libusb_device_handle *m_handle;
	static int m_seq;
	unsigned char m_rawA[IMG_CHUKN_EVEN];
	unsigned char m_rawB[IMG_CHUKN_ODD];
};

}

#endif /* VIDEO_DSI2PRO_H_ */

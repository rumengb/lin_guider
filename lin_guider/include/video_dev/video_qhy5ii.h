/*
 * video_qhy5ii.h
 *
 *  Created on: 05.10.2013
 *      Author: gm
 *
 * Device access code is based on original QHY code from https://github.com/qhyccd-lzr
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

#ifndef VIDEO_QHY5II_H_
#define VIDEO_QHY5II_H_

#include "video.h"
#include "qhy5ii_core.h"


namespace video_drv
{

// QHY5LII
#define QHY5LII_MATRIX_WIDTH    1280
#define QHY5LII_MATRIX_HEIGHT   960

#define QHY5LII_WIDTH_B1	QHY5LII_MATRIX_WIDTH
#define QHY5LII_HEIGHT_B1	QHY5LII_MATRIX_HEIGHT

#define QHY5LII_WIDTH_B2	1024
#define QHY5LII_HEIGHT_B2	768

#define QHY5LII_WIDTH_B3	800
#define QHY5LII_HEIGHT_B3	600

#define QHY5LII_WIDTH_B4    640
#define QHY5LII_HEIGHT_B4   480


// QHY5II
#define QHY5II_MATRIX_WIDTH    1280
#define QHY5II_MATRIX_HEIGHT   1024

#define QHY5II_WIDTH_B1		QHY5II_MATRIX_WIDTH
#define QHY5II_HEIGHT_B1	QHY5II_MATRIX_HEIGHT

#define QHY5II_WIDTH_B2		1024
#define QHY5II_HEIGHT_B2	768

#define QHY5II_WIDTH_B3		640
#define QHY5II_HEIGHT_B3	480

#define V4L2_CID_USER_8BIT		(V4L2_CID_USER_CLASS + 1)
#define V4L2_CID_USER_USB_TRAF 	(V4L2_CID_USER_CLASS + 2)
#define V4L2_CID_USER_DSPEED    (V4L2_CID_USER_CLASS + 3)
#define V4L2_CID_USER_DODEBAYER (V4L2_CID_USER_CLASS + 4)

/*
 * qhy5ii camera
 */
class cvideo_qhy5ii : public cvideo_base
{
public:
	cvideo_qhy5ii();
	virtual ~cvideo_qhy5ii();

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

	qhy5ii_core_shared *m_qhy5ii_obj;
private:
	void set_resolution( int x, int y );
	void set_speed( bool isHigh );
	void set_usb_traffic( int i );
	void set_transfer_bit( int Bit );
	int  set_exposure_time( double exptime );
	int  set_gain( unsigned short gain );

	double get_temperature( void );

	unsigned int get_pix_fmt( void );

	// current settings
	// int m_exposuretime;	// stores in capture_params.fps
	// int m_gain;	// stores in capture_params.gain
	int m_data_size;

	//----------------------------------------------------------------
	struct QHY5LII
	{
		QHY5LII() : QHY5L_PLL_Ratio( 0 ), CheckBoxQHY5LIILoneExpMode( false ), longExpMode( false )
		{}
	    int QHY5L_PLL_Ratio;
	    bool CheckBoxQHY5LIILoneExpMode;
	    bool longExpMode;
	};
	QHY5LII m_QCam5LII;

	// QHY legacy fields
	bool m_is_color;
	int m_dev_type;
	int m_width;
	int m_height;
	int m_transfer_bit; // 8: Bit Transfer Mode Transfer Mode 16:16
	int m_bin;
	int m_transfer_speed; //Transmission speed 0: Low 1: High-speed
	int m_usb_traf;

	int m_wbblue; //Used to set the white balance
	int m_wbred;
	bool m_do_debayer;

	bool m_qhy5iiDeNoise;

	static int m_gain_lut[73];

	void set_wb_blue( int blue );
	void set_wb_green( int green );
	void set_wb_red( int red );

	void   InitQHY5LIIRegs( void );
	void   SetQHY5LIIGain( unsigned short gain );
	void   SetExposureTime_QHY5LII( uint32_t i );
	void   SetGainMonoQHY5LII( double gain);
	void   SetGainColorQHY5LII( double gain, double RG, double BG );
	void   CorrectQHY5LIIWH( int *w,int *h );
	double setQHY5LREG_PLL( unsigned char clock );
	void   initQHY5LII_1280X960( void );
	void   initQHY5LII_QVGA( void );
	void   initQHY5LII_SVGA (void );
	void   initQHY5LII_VGA( void );
	void   initQHY5LII_XGA( void );
	void   SetQHY5LIIHDR( bool on );
	void   Set14Bit( unsigned char i );
	void   SWIFT_MSBLSBQHY5LII( unsigned char *ImgData );
	void   SetSpeedQHY5LII( unsigned char i );

	//------------------------------------------
	unsigned char MSB( unsigned short i );
	unsigned char LSB( unsigned short i );
	void I2CTwoWrite( uint16_t addr, uint16_t value );
	unsigned short I2CTwoRead( uint16_t addr );

	//------------------ QHY5-II -------------------
	void SetExposureTime_QHY5II( uint32_t i );
	void QHY5IISetResolution( int x, int y );
	void CorrectQHY5IIWH( int *w, int *h );
	void initQHY5II_SXGA( void );
	void initQHY5II_XGA( void );
	void initQHY5II_SVGA( void );
	void initQHY5II_VGA( void );
	void SetQHY5IIGain( unsigned short gain );

	// smth.
	int qhy5_exposure_kludge( unsigned tm );
	int set_gain_core( unsigned short gain );
	void start_video_mode( void );
	void stop_video_mode( void );
};

}

#endif /* VIDEO_QHY5II_H_ */

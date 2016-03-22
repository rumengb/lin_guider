/*
 * io_driver.h
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

#ifndef IO_DRIVER_H_
#define IO_DRIVER_H_

#include <sys/types.h>
#include <pthread.h>

#define DBG 1

namespace io_drv
{

enum guide_dir
{
	NO_DIR = 0,
	RA_INC_DIR,
	RA_DEC_DIR,
	DEC_INC_DIR,
	DEC_DEC_DIR
};

struct guide_dir_desc
{
	guide_dir dir;
	const char *desc;
};

extern struct guide_dir_desc  bit_list_data[5];

typedef struct
{
	guide_dir bit_direction[8];
	bool is_inverse;
}bit_direction_t;

typedef struct
{
	bool use;
	long delay;
}death_time_t;

enum device_type
{
	DT_MIN = 0,
	DT_NULL,
	DT_LPT,
	DT_FTDI,
	DT_GPUSB,
	DT_GPIO,
	DT_QHY5,
	DT_QHY6,
	DT_QHY5II,
	DT_NEXSTAR,
	DT_ATIK,
	DT_SX,
	DT_ASI,
	DT_SKYWATCHER,
	DT_MAX,
	DEVICE_CNT = DT_MAX-1
};

typedef struct
{
	u_char bit_map[8];
}device_bit_map_t;

extern device_bit_map_t device_bit_map_template[DEVICE_CNT];

typedef struct
{
	const int type;
	const bool show_dev_string_ui;
	const int  bits_enabled_ui;
	const char *desc;
	const char *info;	// any necessary text info
	const char *hyper_info;	// any necessary hypertext info
}device_desc_t;

extern device_desc_t device_desc_list[DEVICE_CNT];

typedef struct
{
	bit_direction_t dir_map;
	death_time_t death_time;
	int type, next_device_type;
	device_bit_map_t bit_map_template;
}device_init_params_t;

typedef struct
{
	int type;
	int min_pulse_length;
	int max_pulse_length;
}device_ro_params_t;



class pulse_event
{
public:
	pulse_event( guide_dir dir = NO_DIR, unsigned long time = 0, unsigned long cur_time = 0 )
	{
		m_pulse_dir 	= dir;
		m_wait_time 	= time;
		m_current_time 	= cur_time;
	};
	~pulse_event(){};

	pulse_event( const pulse_event &p )
	{
		m_pulse_dir 	= p.m_pulse_dir;
		m_wait_time		= p.m_wait_time;
		m_current_time 	= p.m_current_time;
	};

	inline bool is_RA()
	{
		return m_pulse_dir == RA_INC_DIR || m_pulse_dir == RA_DEC_DIR;
	};

	inline bool is_DEC()
	{
		return m_pulse_dir == DEC_INC_DIR || m_pulse_dir == DEC_DEC_DIR;
	};

	inline bool is_empty()
	{
		return m_pulse_dir == NO_DIR;
	};

	inline bool is_set()
	{
		return m_pulse_dir != NO_DIR;
	};

	inline void clear()
	{
		m_pulse_dir = NO_DIR;
		m_wait_time = 0;
	}
	inline guide_dir pulse_dir()
	{
		return m_pulse_dir;
	}
	inline unsigned long wait_time()
	{
		return m_wait_time;
	}
	inline unsigned long current_time()
	{
		return m_current_time;
	}
	inline void set_current_time( unsigned long current_time )
	{
		m_current_time = current_time;
	}
private:
	guide_dir m_pulse_dir;
	unsigned long 	  m_wait_time;
	unsigned long 	  m_current_time;
};


class cio_driver_base
{
public:
	cio_driver_base();
	virtual ~cio_driver_base();

	bool start( const char *devname );			// start device (must be called after creation)
	void stop( void );							// stops device (must be called before destruction)
	bool set_deviceparams( const device_init_params_t &params );
	device_init_params_t get_deviceparams( void );
	device_ro_params_t get_ro_params( void );
	void test_dir_map( const bit_direction_t &test_map, guide_dir test_dir );
	bool set_bit_map_template( const device_bit_map_t &tmpl );
	device_bit_map_t get_bit_map_template( void ) const;
	static bool check_device_dir_map( const bit_direction_t &dir );

	bool do_pulse( guide_dir ra_dir, int ra_msecs, guide_dir dec_dir, int dec_msecs );	// do dual-axis pulse (thread-safe)
	bool do_pulse( guide_dir dir, int msecs );											// do single-axis pulse (thread-safe)
	bool reset( void );		// stops any guiding and sets map (thread-safe)
	bool is_initialized( void );
	const char* get_name( void ) const;
	bool swap_dec_bits( void );

private:
	virtual int open_device( void ) = 0;				// open device
	virtual int close_device( void ) = 0;				// close dvice

	virtual void write_data( unsigned int dByte ) = 0;	// write data to control telescope
	virtual u_char read_byte( void ) = 0;				// not implemented

protected:
	bool initialized;  						// If port opened = true
	char dev_name[64];
	device_bit_map_t bit_map_template;		// actual low-level bit mapping template
	device_bit_map_t cfg_bit_map_template;	// loaded from config low-level bit mapping template
	u_char 	bit_map_encoder[256];			// encoder-table for bits
	int next_device_type;

	// read only hardware parameters
	int device_type;
	int min_pulse_length;	// mininum pulse length in msecs
	int max_pulse_length;	// maximum pulse length in msecs

	// relay death time
	bool use_DT;
	long DT;
	struct timespec relay_DT[8];

	//---------- main bit logic ------------
	guide_dir 	bit_actions[8];	// motion action of each bit
	bool		is_inverse;		// inverse bit mode

private:
	// thread stuff
	pthread_t dev_thread;		// thread desc.
	pthread_cond_t cv;			// start cond. var.
	pthread_mutex_t	cv_mutex;	// cond. var. mutex
	pthread_mutex_t	sync_mutex;	// data sync. mutex
	int	start_thread_flag;		// start flag
	int	quit_thread_flag;		// start flag

	// thread function
	static void *controller_thread( void *param );

	// pulse and out data
	pulse_event     request_RA;		// correction request by RA
	pulse_event     request_DEC;	// correction request by DEC
	u_char port_data;				// byte of data in port

	void init_bit_map_encoder( void );
	unsigned char check_command_for_DT( unsigned char cmd );
	u_char make_device_command( guide_dir dir1, guide_dir dir2 = NO_DIR );
	void send_device_command( guide_dir dir1, guide_dir dir2 );	// sends command to device

};

}

/*
#include <sys/io.h>

#define base 0x378


int ppput2(int b)
{
	char x = (char) b;
	int res;

  if( (res = ioperm(base,1,1)) )
  {
	  switch( errno )
	  {
	  case EINVAL:
		  U_Msg("EINVAL");
		  break;
	  case EIO:
	  	  U_Msg("EIO");
	  	  break;
	  case ENOMEM:
	  	  U_Msg("ENOMEM");
	  	  break;
	  case EPERM:
	  	  U_Msg("EPERM");
	  	  break;
	  }
  return -1;
  }
    outb(x, base);
 */

#endif /*IO_DRIVER_H_*/

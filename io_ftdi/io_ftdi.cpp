#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ftdi.h>

#include "io_ftdi.h"


static struct ftdi_context ftdic = {0};
bool initialized = false;


int io_ftdi_init( int *chp_type )
{
 int f;
 unsigned int chipid;
 unsigned char out_byte;
 bool is_usb_opened = false;

 	 if( initialized )
 		 return 0;

	 ftdi_init(&ftdic);



	 f = ftdi_usb_open( &ftdic, 0x0403, 0x6001 );

	 if( f < 0 && f != -5 )
	 {
		 fprintf( stderr, "unable to open ftdi device: %d (%s)\n", f, ftdi_get_error_string(&ftdic) );
		 goto error;
	 }

	 is_usb_opened = true;

	 fprintf( stdout, "ftdi open succeeded: %d\n", f );

	 fprintf( stdout, "enabling bitbang mode\n" );
	 // commenting as deprecated
	 //ftdi_enable_bitbang( &ftdic, 0xFF );
	 /** Bitbang mode. 1: (default) Normal bitbang mode, 2: FT2232C SPI bitbang mode */
	 f = ftdi_set_bitmode( &ftdic, 0xFF, 1 );
	 if( f < 0 )
	 {
		 fprintf( stderr,"ftdi_set_bitmode(): failed, error %d (%s)\n", f, ftdi_get_error_string(&ftdic) );
		 goto error;
	 }

//	 sleep(2);
//
//	 out_byte = 0xFF;
//	 fprintf( stdout, "turning everything on\n" );
//
//	 f = ftdi_write_data( &ftdic, &out_byte, 1 );
//	 if( f < 0 )
//	 {
//		 fprintf( stderr,"write failed for 0x%x, error %d (%s)\n", out_byte, f, ftdi_get_error_string(&ftdic) );
//		 goto error;
//	 }

	 sleep(2);

	 out_byte = 0x0;
	 fprintf( stdout, "turning everything off\n" );

	 f = ftdi_write_data( &ftdic, &out_byte, 1 );
	 if( f < 0 )
	 {
		 fprintf( stderr,"write failed for 0x%x, error %d (%s)\n", out_byte, f, ftdi_get_error_string(&ftdic) );
		 goto error;
	 }

	 // Read out FTDIChip-ID of R type chips
     fprintf( stdout, "ftdi_read_chipid: %d\n", ftdi_read_chipid(&ftdic, &chipid) );
     fprintf( stdout, "FTDI chipid: %X\n", chipid );

     *chp_type = ftdic.type;

	 initialized = true;
	 return 0;

error:
	 if( is_usb_opened )
		 ftdi_usb_close( &ftdic );
	 ftdi_deinit( &ftdic );

 return -1;
}


int io_ftdi_release( void )
{
 int f;

	fprintf(stdout, "ftdi releasing...\n");

 	if( !initialized )
 		return 0;

 	fprintf( stdout, "disabling bitbang mode\n" );
 	f = ftdi_disable_bitbang( &ftdic );
 	if( f < 0 )
 	{
 		fprintf( stderr,"ftdi_disable_bitbang(): failed, error %d (%s)\n", f, ftdi_get_error_string(&ftdic) );
 	}

 	ftdi_usb_close( &ftdic );
 	ftdi_deinit( &ftdic );

 	fprintf( stdout, "ftdi released\n" );

 return 0;
}


int io_ftdi_write( unsigned int dByte )
{
 int f;
 unsigned char out_byte;

 	if( !initialized )
 		return -1;

 	out_byte = (unsigned char)dByte;

 	f = ftdi_write_data( &ftdic, &out_byte, 1 );
 	if( f < 0 )
 	{
 	    fprintf( stderr,"write failed for 0x%x, error %d (%s)\n", out_byte, f, ftdi_get_error_string(&ftdic) );
 	    return -2;
 	}

 return 0;
}


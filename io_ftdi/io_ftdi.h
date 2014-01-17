#ifndef IO_FTDI_H_
#define IO_FTDI_H_

extern "C"
{
int io_ftdi_init( int *chp_type );
int io_ftdi_release( void );
int io_ftdi_write( unsigned int dByte );
}

#endif /*IO_FTDI_H_*/

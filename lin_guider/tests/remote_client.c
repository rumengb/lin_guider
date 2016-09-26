/*
 * remote_client.c
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

// client in the unix domain for Lin_guider
// gcc -Wall -ggdb3 -O0 -o remote_client remote_client.c
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>

#define HDR_SZ 8

enum commands
{
	GET_VER = 1, //
	SET_GUIDER_SQUARE_POS,
	SAVE_FRAME,
	DITHER,
	DITHER_NO_WAIT_XY,
	GET_DISTANCE,
	SAVE_FRAME_DECORATED,
	GUIDER,
	GET_GUIDER_STATE,
	SET_GUIDER_OVL_POS,
	SET_GUIDER_RETICLE_POS,
	FIND_STAR,
	SET_DITHERING_RANGE,
	GET_RA_DEC_DRIFT,

	CMD_MAX
};

void error(char *);

int main(int argc, char *argv[])
{
	int sockfd, servlen;
	struct sockaddr_un  serv_addr;
	char buffer[82];
	char readbuf[16384];

	if( argc < 2 )
	{
		printf( "Usage: client [command no]\nThen type parameters in console and press 'Enter'\n \
\t\t'GET_VER' = 1, no params\n \
\t\t'SET_GUIDER_SQUARE_POS'=2, [x y]\n\
\t\t'SAVE_FRAME'=3, file_name w/o extension\n \
\t\t'DITHER'=4, no params\n \
\t\t'DITHER_NO_WAIT_XY=5, [dx dy]\n \
\t\t'GET_DISTANCE=6', no params\n \
\t\t'SAVE_FRAME_DECORATED=7', no params\n \
\t\t'GUIDER'=8, [start|stop]\n \
\t\t'GET_GUIDER_STATE'=9, no params\n \
\t\t'SET_GUIDER_OVL_POS'=10, [x y]\n \
\t\t'SET_GUIDER_RETICLE_POS'=11, [x y]\n \
\t\t'FIND_STAR'=12, no params\n \
\t\t'SET_DITHERING_RANGE'=13, [dr]\n \
\t\t'GET_RA_DEC_DRIFT'=14, no params\n \
" );
		return 0;
	}
	int cmd = strtol( argv[1], NULL, 10 );
	if( cmd < GET_VER || cmd >= CMD_MAX )
	{
		printf("Wrong command number\n");
		return 0;
	}

	bzero((char *)&serv_addr,sizeof(serv_addr));
	serv_addr.sun_family = AF_UNIX;
	strcpy(serv_addr.sun_path, "/tmp/lg_ss");
	servlen = strlen(serv_addr.sun_path) + sizeof(serv_addr.sun_family);
	if( (sockfd = socket(AF_UNIX, SOCK_STREAM,0)) < 0 )
		error("Error Creating socket");
	if( connect(sockfd, (struct sockaddr *)&serv_addr, servlen) < 0 )
		error("Error Connecting to Lin_guider");
	printf("Connected to Lin_guider\n");
	printf("Enter command params: ");
	bzero(buffer,82);
	fgets(buffer,80,stdin);
	//for( i = 0;i < 100;i++ )
	{
		char out[256];
		uint16_t *ptr16 = (uint16_t *)out;
		uint32_t *ptr32 = (uint32_t *)out;

		ptr16[0] = 2;	// SIGNATURE
		ptr16[1] = cmd;		// CMD
		ptr32[1] = strlen(buffer)-1;	// DATA LEN
		memcpy( out+HDR_SZ, buffer, strlen(buffer)-1 );

		{
			int pos = 0;
			int n = 0;
			int to_write = HDR_SZ+strlen(buffer)-1;
			do
			{
				n = write( sockfd, out+pos, to_write );
				if( n < 0 )
					error("ERROR writing to socket");
				pos += n;
			}while( pos < to_write );
			printf("Wrote\n");
		}

		{
			int pos = 0;
			int n = 0;
			int to_read = HDR_SZ;
			do
			{
				n = read( sockfd, readbuf+pos, to_read );
				pos += n;
				if( n == to_read )
				{
					int data_sz = ((uint32_t*)(readbuf+4))[0];
					printf("answer data size = %d\n", data_sz);
					if( data_sz < 0 || data_sz > 16000 )
						error("ERROR length to read");
					to_read += data_sz;
				}
			}while( pos < to_read );
			printf("The return message was '%.*s'\n", ((uint32_t*)(readbuf+4))[0], readbuf+HDR_SZ);
		}
	}
	printf("=== done ===\n");
	//sleep(2);
	close(sockfd);

	return 0;
}

void error(char *msg)
{
    perror(msg);
    exit(0);
}

#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <time.h>
#include <errno.h>
#include <sys/types.h>

// #include <serial.h>
#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>
#include <stdbool.h>
#include <termios.h>

enum {
	STOPBITS_1 = 0<<0,
	STOPBITS_2 = 1<<0,

	PARITY_NONE = 0<<1,
	PARITY_ODD  = 1<<1,
	PARITY_EVEN = 2<<1,
	PARITY_MASK = 3<<1,

	BLOCKING_IO = 1<<3,

	SERIAL_FLAG_NONE = 0
} SerialFlags;

typedef struct {
	struct termios oldopts;
	struct termios newopts;
	speed_t speed;
	int fd;
} SerialPort;

int fd_set_nonblock( int fd, int nonblock );
bool serial_open( SerialPort* port,
		const char* device, uint32_t baudrate, uint32_t flags );
bool serial_close( SerialPort* port );

#endif // SERIAL_H


int fd_set_nonblock( int fd, int nonblock )
{
	nonblock = nonblock ? O_NONBLOCK : 0;
	return fcntl( fd, F_SETFL, nonblock );
}

bool serial_open( SerialPort* port,
		const char* device, uint32_t baudrate, uint32_t flags )
{
	port->fd = -1;
	switch( baudrate )
	{
		case   1200: port->speed = B1200; break;
		case   2400: port->speed = B2400; break;
		case   4800: port->speed = B4800; break;
		case   9600: port->speed = B9600; break;
		case  19200: port->speed = B19200; break;
		case  38400: port->speed = B38400; break;
		case  57600: port->speed = B57600; break;
		case 115200: port->speed = B115200; break;
		case 230400: port->speed = B230400; break;
		case 460800: port->speed = B460800; break;
		case 921600: port->speed = B921600; break;
		default: fprintf( stderr, "invalid baudrate %d.\n", baudrate ); return false;
	}
	port->fd = open( device, O_RDWR | O_NOCTTY );
	if( port->fd == -1 )
	{
		char msg[100];
		snprintf( msg, 100, "Failed to open serial device %s", device );
		perror( msg );
		return false;
	}
	if( fd_set_nonblock( port->fd, (flags & BLOCKING_IO) == 0 ) == -1 )
	{
		perror( "setting nonblocking failed" );
		return false;
	}
	tcgetattr( port->fd, &port->newopts );
	memcpy( &port->oldopts, &port->newopts, sizeof(port->oldopts) );
	port->newopts.c_cc[VMIN] = 1;
	port->newopts.c_cflag = CS8 | CLOCAL | CREAD;
	if( flags & STOPBITS_2 )
	{
		port->newopts.c_cflag |= CSTOPB;
	}
	switch( flags & PARITY_MASK )
	{
		case PARITY_EVEN: port->newopts.c_cflag |= PARENB; break;
		case PARITY_ODD: port->newopts.c_cflag |= PARENB | PARODD; break;
	}
	port->newopts.c_iflag = 0;
	port->newopts.c_oflag = 0;
	port->newopts.c_lflag = 0;
	if( cfsetspeed( &port->newopts, port->speed ) )
	{
		perror( "Failed to set serial speed" );
		return false;
	}
port->newopts.c_cflag &= ~CRTSCTS;
// port->newopts.c_cflag |= CRTSCTS;
port->newopts.c_iflag &= ~(IXON | IXOFF | IXANY);
port->newopts.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	tcsetattr( port->fd, TCSANOW, &port->newopts );
	return true;
}

bool serial_close( SerialPort* port )
{
	if( port->fd != -1 )
	{
		tcsetattr( port->fd, TCSANOW, &port->oldopts );
		close( port->fd );
	}
	return true;
}



bool get_line( char* buffer, int* pbufcnt, char* line, bool escape, bool noeol )
{
	int bufcnt = *pbufcnt;
	int i;
	int j = 0;
	char c;

	for( i=0; i<bufcnt; i++ )
	{
		c = buffer[i];
		if( c == '\n' )
		{
			if( escape )
			{
				strcpy( line+j, "\\n" );
			}
			else
			{
				line[j] = 0;
			}
			i++;
			memmove( buffer, buffer+i, bufcnt-i );
			*pbufcnt -= i;
			return true;
		}
		else if( c == '\r' )
		{
			if( escape )
			{
				strcpy( line+j, "\\r" );
				j += 2;
			}
		}
		else if( escape && (c < 32 || c >= 127) )
		{
			j += sprintf( line+j, "\\%02X", c & 0xFF );
		}
		else
		{
			line[j++] = c;
		}
	}
	if( noeol && j > 0 )
	{
		line[j] = 0;
		*pbufcnt = 0;
		return true;
	}
	return false;
}

int slow_write( int fd, const char* data, const int cnt )
{
	int n = 0;

	while( n < cnt )
	{
		usleep( 10000 );
		int m = write( fd, data, 1 );
		if( m == 0 )
		{
			break;
		}
		else if( m < 0 )
		{
			int err = errno;
			fprintf( stderr, "write error %d: %s\n", err, strerror(err) );
			break;
		}
		data++;
		n++;
	}
	return n;
}

bool unescape( const char* in, char* out, int maxlen )
{
	char c;

	while( maxlen > 0 )
	{
		c = *in;
		in++;

		if( c == 0 )
		{
			*out = 0;
			return true;
		}
		if( c != '\\' )
		{
			*out = c;
			out++;
		}
		else
		{
			c = *in;
			in++;
			switch( c )
			{
				case 0:
					return false;
				case '\\':
					*out = '\\';
					out++;
					break;
				case 'r':
					*out = '\r';
					out++;
					break;
				case 'n':
					*out = '\n';
					out++;
					break;
				case 't':
					*out = '\t';
					out++;
					break;
				case '0':
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
				case '7':
				case '8':
				case '9':
				case 'a':
				case 'b':
				case 'c':
				case 'd':
				case 'e':
				case 'f':
				case 'A':
				case 'B':
				case 'C':
				case 'D':
				case 'E':
				case 'F':
					if( !isxdigit(*in) )
					{
						return false;
					}
					else
					{
						char tmp[3];
						tmp[0] = c;
						tmp[1] = *in;
						tmp[2] = 0;
						c = strtol( tmp, 0, 16 );
						in++;
						*out = c;
						out++;
					}
					break;
				default:
					return false;
			}
		}
		maxlen--;
	}
	return false;
}

#define BUFSIZE 1024

int main( int argc, char** argv )
{
	const char* device;
	int baudrate;
	int wait_ms;
	const char* command;
	int flags = 0;
	SerialPort port;
	struct timespec ts_end;
	struct timespec ts_now;
	struct pollfd pfd;
	char serbuf[BUFSIZE];
	int sercnt = 0;
	char line[BUFSIZE*4];
	bool running = true;
	int n;
	int rc = 0;

	if( argc != 5 )
	{
		fprintf( stderr, "usage: serial device baudrate wait-ms command\n" );
		return 1;
	}
	device = argv[1];
	baudrate = atoi( argv[2] );
	wait_ms = atoi( argv[3] );
	command = argv[4];
	flags |= PARITY_NONE;
	flags |= STOPBITS_1;
	if( !unescape( command, line, BUFSIZE ) )
	{
		fprintf( stderr, "error unescaping command\n" );
		return 1;
	}

	if( !serial_open( &port, device, baudrate, flags ) )
	{
		fprintf( stderr, "opening serial port failed.\n" );
		return 1;
	}

	pfd.fd = port.fd;
	pfd.events = POLLIN;

	slow_write( pfd.fd, line, strlen(line) );

	if( clock_gettime( CLOCK_MONOTONIC, &ts_end ) != 0 )
	{
		n = errno;
		fprintf( stderr, "clock_gettime error %d: %s\n", n, strerror(n) );
		return 1;
	}
	ts_end.tv_sec += wait_ms / 1000;
	ts_end.tv_nsec += 1000000 * (wait_ms % 1000);
	if( ts_end.tv_nsec > 1000000000 )
	{
		ts_end.tv_sec++;
		ts_end.tv_nsec -= 1000000000;
	}
	while( running )
	{
		n = poll( &pfd, 1, 200 );
		if( n < 0 )
		{
			n = errno;
			fprintf( stderr, "poll error %d: %s\n", n, strerror(n) );
			running = false;
		}
		else if( n == 0 )
		{
			if( sercnt > 0 )
			{
				while( get_line( serbuf, &sercnt, line, true, true ) )
				{
					printf( "%s\n", line );
				}
				sercnt = 0;
			}
		}
		else
		{
			if( pfd.revents == POLLIN )
			{
				n = read( pfd.fd, serbuf+sercnt, BUFSIZE-sercnt );
				if( n > 0 )
				{
					sercnt += n;
					while( get_line( serbuf, &sercnt, line, true, false ) )
					{
						printf( "%s\n", line );
					}
				}
				else if( n == 0 )
				{
					fprintf( stderr, "EOF on serial port\n" );
					running = false;
				}
				else
				{
					n = errno;
					fprintf( stderr,
						"read error %d on serial port: %s\n", n, strerror(n) );
					running = false;
				}
			}
			else if( pfd.revents != 0 )
			{
				fprintf( stderr, "poll error on serial port\n" );
				running = false;
			}
		}
		if( clock_gettime( CLOCK_MONOTONIC, &ts_now ) != 0 )
		{
			n = errno;
			fprintf( stderr, "clock_gettime error %d: %s\n", n, strerror(n) );
			rc = 1;
			running = false;
		}
		else if( ts_now.tv_sec > ts_end.tv_sec ||
					( ts_now.tv_sec == ts_end.tv_sec &&
					  ts_now.tv_nsec > ts_end.tv_nsec ) )
		{
			running = false;
			if( sercnt > 0 )
			{
				while( get_line( serbuf, &sercnt, line, true, true ) )
				{
					printf( "%s\n", line );
				}
				sercnt = 0;
			}
		}
	}

	serial_close( &port );

	return rc;
}

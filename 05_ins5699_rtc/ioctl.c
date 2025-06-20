#include <stdio.h>
#include <linux/rtc.h>
//#include <uapi/linux/rtc.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

typedef unsigned char u8;
typedef unsigned bool;
typedef struct {
	u8 number;
	u8 value;
}reg_data;

#define SE_RTC_REG_READ         _IOWR('p', 0x20, reg_data)
#define SE_RTC_REG_WRITE        _IOW('p',  0x21, reg_data)

// from linux-src/include/uapi/linux/rtc.h
#define RTC_VL_READ	_IOR('p', 0x13, int)	/* Voltage low detector */
#define RTC_VL_CLR	_IO('p', 0x14)		/* Clear voltage low information */

unsigned htoi( const char* psHex )
{
	unsigned i = 0;
	const bool fNeg = (*psHex == '-');

	if ( fNeg )
		psHex++;

	for ( ;; )
	{
		unsigned char cDigit = (unsigned char)((*psHex++ | 0x20) - '0');
		if ( (cDigit < 10) || ((cDigit -= 39) < 16) )
			i = (i << 4) + cDigit;
		else
			break;
	}

	if ( fNeg )
		i = 0 - i;

	return i;
}


int main(int argc, char **argv)
{
	int fd, retval;
	const char * rtc = "/dev/rtc0";
	reg_data reg;

    printf("rtc ioctl test\n");

	fd = open(rtc, O_RDWR);

	if ( fd == -1 )
	{
		perror(rtc);
		exit(errno);
	}

	if ( argc == 1 )
	{
		return 0;
	}

	if ( ! strcmp(argv[1], "read") && argc==3 )
	{
		reg.number = htoi(argv[2]);
		retval = ioctl(fd, SE_RTC_REG_READ, &reg);
		if (retval == -1) {
			perror("SE_RTC_REG_READ ioctl");
			exit(errno);
		}
		else
			printf("REG[%02xh]=>%02xh\n", reg.number, reg.value);
		goto done;
	}

	if ( ! strcmp(argv[1], "write") && argc==4 )
	{
		reg.number = htoi(argv[2]);
		reg.value  = htoi(argv[3]);
		retval = ioctl(fd, SE_RTC_REG_WRITE, &reg);
		if (retval == -1) {
			perror("SE_RTC_REG_WRITE ioctl");
			exit(errno);
		}
		else
			printf("REG[%02xh]<=%02xh\n", reg.number, reg.value);
		goto done;
	}

        if ( ! strcmp(argv[1], "vlr") && argc==2 )
        {
                int vl;
		retval = ioctl(fd, RTC_VL_READ, &vl);
                if (retval == -1) {
                        perror("RTC_VL_READ ioctl");
                        exit(errno);
                }
                else
                        printf("%d\n", vl);
                goto done;
        }

        if ( ! strcmp(argv[1], "vlc") && argc==2 )
        {
                retval = ioctl(fd, RTC_VL_CLR, 0);
                if (retval == -1) {
                        perror("RTC_VL_CLR ioctl");
                        exit(errno);
                }

                goto done;
        }


done:
	close(fd);
	return 0;
}



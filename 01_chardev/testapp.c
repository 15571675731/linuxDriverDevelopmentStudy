#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

#define DEV_NAME        "/dev/chardev"

int main()
{
    int fd = 0;
    int buf = 0;

    fd = open(DEV_NAME, O_RDWR);
    if(fd > 0)
    {
        write(fd, &buf, 1);
        read(fd, &buf, 1);
        close(fd);
    }
}


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define DEV_NAME    "/dev/myleddev"

int main(int argc, char **argv)
{
    int fd = -1;
    char read_buf[100] = {0};
    char write_buf[100] = {0};

    if(argc != 2)
    {
        printf("eeeerror\n");
        return 0;
    }

    fd = open(DEV_NAME, O_RDWR);
    if(fd < 0){
        perror("open cdev failed!");
        return -1;
    }

    memcpy(write_buf, argv[1], strlen(argv[1]));

    write(fd, write_buf, strlen(write_buf));
    close(fd);
    fd = open(DEV_NAME, O_RDWR);
    // lseek(fd, 0, SEEK_SET);
    read(fd, read_buf, 10);
    printf("read:%s\n", read_buf);

    sleep(5);
    close(fd);
    return 0;
}



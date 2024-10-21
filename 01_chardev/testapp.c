#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>



int main()
{
    int fd = -1;
    char read_buf[100] = {0};
    char write_buf[100] = "hello world!";

    fd = open("/dev/chardev", O_RDWR);
    if(fd < 0){
        perror("open cdev failed!");
        return -1;
    }

    write(fd, write_buf, strlen(write_buf));
    close(fd);
    fd = open("/dev/chardev", O_RDWR);
    // lseek(fd, 0, SEEK_SET);
    read(fd, read_buf, 100);
    printf("read:%s\n", read_buf);
    close(fd);
    return 0;
}



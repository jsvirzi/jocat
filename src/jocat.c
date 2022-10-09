#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <signal.h>
#include <errno.h>
#include <sys/select.h>
#include <pthread.h>

#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

typedef struct {
    int fd;
    int run;
    unsigned char rx_buff[2048];
    unsigned char tx_buff[2048];
    int rx_pos;
    int rx_len;
    int tx_pos;
    int tx_len;
    pthread_t tid;
    char thread_name[32];
} ReadSerialLooperInfo;

ReadSerialLooperInfo looper_info;

void delay_ms(unsigned int ms) {
    struct timeval tv;
    tv.tv_sec = ms / 1000ULL;
    tv.tv_usec = ms * 1000ULL;
    select(0, NULL, NULL, NULL, &tv);
}

void *read_serial_looper(void *arg)
{
    ReadSerialLooperInfo *info = (ReadSerialLooperInfo *) arg;
    info->rx_pos = 0;
    info->rx_len = sizeof (info->rx_buff);
    memset(info->rx_buff, 0, info->rx_len);
    while (info->run == 0) { delay_ms(1); }
    while (info->run) {
        delay_ms(1);
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 10 * 1000; /* 10 milliseconds */
        fd_set rset;
        fd_set wset;
        FD_ZERO(&rset);
        FD_ZERO(&wset);
        FD_SET(info->fd, &rset);
        FD_SET(info->fd, &wset);
        int max_fd = info->fd + 1;
        int status = select(max_fd + 1, &rset, &wset, NULL, &timeout);
        if (status < 0) { continue; }
        int rset_flag = FD_ISSET(info->fd, &rset);
        int wset_flag = FD_ISSET(info->fd, &wset);
        if (rset_flag) {
            int room = info->rx_len - info->rx_pos;
            if (room > 0) {
                int n = read(info->fd, &info->rx_buff[info->rx_pos], room);
                if (n > 0) {
                    info->rx_pos += n;
                }
            }
        }
        if (wset_flag) {
            if (looper_info.tx_pos != looper_info.tx_len) {
                int n = looper_info.tx_len - looper_info.tx_pos;
                int n_rem = n;
                while (n_rem > 0) {
                    int n_bytes = write(looper_info.fd, &looper_info.tx_buff[looper_info.tx_pos], n_rem);
                    if (n_bytes > 0) {
                        n_rem -= n_bytes;
                        looper_info.tx_pos += n_bytes;
                    }
                }
            }
        }
    }
}

int main(int argc, char **argv)
{
   /* 
	* vmin = number of characters to grab and deliver at one time 
    * vtime = timeout to wait. 0 = indefinitely
	*/
    int debug = 0, vmin = 1, vtime = 0, do_flush = 0, do_listen = 0;
    struct termios termios;
    char dev_name[64];
    unsigned char latency = 5;
    int baud = 115200;
    int baud_code = B115200;
    memset(&looper_info, 0, sizeof(looper_info));
    snprintf(dev_name, sizeof (dev_name), "/dev/ttyUSB0");
    for (int i = 0; i < argc; ++i) {
        if (strcmp(argv[i], "-debug") == 0) {
            debug = 1;
		} else if (strcmp(argv[i], "-m") == 0) {
            looper_info.tx_len = snprintf(looper_info.tx_buff, sizeof (looper_info.tx_buff), "%s", argv[++i]);
		} else if (strcmp(argv[i], "-d") == 0) {
            strcpy(dev_name, argv[++i]);
        } else if (strcmp(argv[i], "-listen") == 0) {
            do_listen = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-flush") == 0) {
            do_flush = 1;
        }
    }

    looper_info.fd = open(dev_name, O_NOCTTY | O_RDWR);
    tcgetattr(looper_info.fd, &termios);
    cfmakeraw(&termios);
    termios.c_cflag &= ~(CSTOPB | CRTSCTS | CSIZE);
    termios.c_cflag |= (CS8 | CLOCAL);
    cfsetspeed(&termios, baud_code);
    termios.c_cc[VMIN] = vmin;
    termios.c_cc[VTIME] = vtime;
    tcsetattr(looper_info.fd, TCSAFLUSH, &termios);

    looper_info.run = 1;

    int status = pthread_create(&looper_info.tid, NULL, read_serial_looper, &looper_info);
    if (status != 0) { return -1; }
    snprintf(looper_info.thread_name, sizeof (looper_info.thread_name), "nx-serial");
    pthread_setname_np(looper_info.tid, looper_info.thread_name);

    if (do_listen) {
        sleep(do_listen);
    } else {
        while (1) {
            sleep(1);
        }
    }

    looper_info.rx_buff[looper_info.rx_pos] = 0;
    printf("=> %s <=\n", looper_info.rx_buff);
}

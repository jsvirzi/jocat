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

#define N_RX_BUFFS (256)
#define N_TX_BUFFS (16)
#define RX_BUFF_SIZE (256)
#define TX_BUFF_SIZE (256)

typedef struct {
    int fd;
    int run;
    unsigned char rx_buff_pool[N_RX_BUFFS][RX_BUFF_SIZE];
    unsigned char tx_buff_pool[N_TX_BUFFS][TX_BUFF_SIZE];
    unsigned int rx_buff_size[N_RX_BUFFS];
    unsigned int tx_buff_size[N_TX_BUFFS];
    int rx_buff_head;
    int rx_buff_tail;
    int rx_buff_mask;
    int tx_buff_head;
    int tx_buff_tail;
    int tx_buff_mask;
    int rx_pos;
    int rx_len;
    int tx_pos;
    int tx_len;
    pthread_t tid;
    char thread_name[32];
} SerialThreadInfo;

int initialize_serial_thread(SerialThreadInfo *info)
{
    memset(info, 0, sizeof (SerialThreadInfo));
    info->rx_buff_mask = N_RX_BUFFS - 1;
    info->tx_buff_mask = N_TX_BUFFS - 1;
}

typedef struct {

} UdpThreadInfo;

SerialThreadInfo thread_info;

void delay_ms(unsigned int ms) {
    struct timeval tv;
    tv.tv_sec = ms / 1000ULL;
    tv.tv_usec = ms * 1000ULL;
    select(0, NULL, NULL, NULL, &tv);
}

void *serial_thread(void *arg)
{
    SerialThreadInfo *info = (SerialThreadInfo *) arg;
    initialize_serial_thread(info);
    info->rx_pos = 0;
    info->rx_len = sizeof (info->rx_buff_pool[0]);
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
                int n = read(info->fd, &info->rx_buff_pool[info->rx_buff_head][info->rx_pos], room);
                if (n > 0) {
                    info->rx_pos += n;
                }
                if (info->rx_pos >= info->rx_len) {
                    info->rx_buff_size[info->rx_buff_head] = info->rx_pos;
                    info->rx_buff_head = (info->rx_buff_head + 1) & info->rx_buff_mask;
                    info->rx_buff_size[info->rx_buff_head] = 0;
                    info->rx_pos = 0;
                }
            }
        }
        if (wset_flag) {
            if (info->tx_buff_head != info->tx_buff_tail) {
                int n_rem = info->tx_buff_size[info->tx_buff_tail];
                uint8_t *tx_buff = info->tx_buff_pool[info->tx_buff_tail];
                while (n_rem > 0) {
                    int n_bytes = write(info->fd, tx_buff, n_rem);
                    if (n_bytes > 0) {
                        n_rem -= n_bytes;
                        tx_buff += n_bytes;
                    }
                }
                info->tx_buff_tail = (info->tx_buff_tail + 1 ) & info->tx_buff_mask;
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
    memset(&thread_info, 0, sizeof(thread_info));
    snprintf(dev_name, sizeof (dev_name), "/dev/ttyUSB0");
    for (int i = 0; i < argc; ++i) {
        if (strcmp(argv[i], "-debug") == 0) {
            debug = 1;
		} else if (strcmp(argv[i], "-m") == 0) {
            thread_info.tx_len = snprintf(thread_info.tx_buff, sizeof (thread_info.tx_buff), "%s", argv[++i]);
		} else if (strcmp(argv[i], "-d") == 0) {
            strcpy(dev_name, argv[++i]);
        } else if (strcmp(argv[i], "-listen") == 0) {
            do_listen = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-flush") == 0) {
            do_flush = 1;
        }
    }

    thread_info.fd = open(dev_name, O_NOCTTY | O_RDWR);
    tcgetattr(thread_info.fd, &termios);
    cfmakeraw(&termios);
    termios.c_cflag &= ~(CSTOPB | CRTSCTS | CSIZE);
    termios.c_cflag |= (CS8 | CLOCAL);
    cfsetspeed(&termios, baud_code);
    termios.c_cc[VMIN] = vmin;
    termios.c_cc[VTIME] = vtime;
    tcsetattr(thread_info.fd, TCSAFLUSH, &termios);

    thread_info.run = 1;

    int status = pthread_create(&thread_info.tid, NULL, serial_thread, &thread_info);
    if (status != 0) { return -1; }
    snprintf(thread_info.thread_name, sizeof (thread_info.thread_name), "nx-serial");
    pthread_setname_np(thread_info.tid, thread_info.thread_name);

    if (do_listen) {
        sleep(do_listen);
    } else {
        while (1) {
            sleep(1);
        }
    }

    thread_info.rx_buff[thread_info.rx_pos] = 0;
    printf("=> %s <=\n", thread_info.rx_buff);
}

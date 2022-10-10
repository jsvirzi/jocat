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
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define RX_BUFFER_SIZE (2048)
#define TX_BUFFER_SIZE (2048)
#define UDP_PACKET_SIZE (1024)
#define RX_UDP_PACKETS (16)
#define TX_UDP_PACKETS (16)
#define DEFAULT_LOOP_PACE (10)

#define UDP_SERVERS (1)
#define THREAD_NAME_LENGTH (32)

#define ERROR_CODE_SUCCESS (0)
#define ERROR_CODE_FAILURE (-1)
#define ERROR_CODE_TIMEOUT (-2)
#define ERROR_CODE_NOT_READY (-3)

static unsigned long long one_thousand = 1000LL;
static unsigned long long one_million = 1000000LL;

enum {
    VERBOSE_LEVEL_NONE = 0,
    VERBOSE_LEVEL_INFO,
    VERBOSE_LEVEL_DEBUG,
    VERBOSE_LEVELS
};

enum {
    CONNECTION_STATE_WAIT_CONNECT = 0,
    CONNECTION_STATES
};

typedef struct _UdpServerInfo
{
    int verbose_level;
    int debug_mode;
    uint32_t sunset_expiry;
    unsigned int sunset_period;
    struct timespec ts0;
    unsigned long long ts0_secs;
    int run_change_request;
    pthread_t tid;
    int udp_port;
    int socket_fd;
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    unsigned int socket_timeout;
    unsigned int socket_period;
    unsigned int queue_overflow_policy;
    uint32_t dropped_rx_packets;
    uint32_t dropped_tx_packets;
    uint32_t n_rx;
    uint32_t n_tx;
    uint32_t n_inconsistent_size_errors;
    unsigned int connection_state;
    uint32_t connection_state_timeout;
    unsigned int status_report_period;
    uint32_t status_report_expiry;
    unsigned int keep_alive_period;
    uint32_t keep_alive_expiry;
    unsigned char rset_flag;
    unsigned char wset_flag;
    /* for chain-linking */
} UdpServerInfo;

struct _SerialThreadInfo;

typedef struct _UdpThreadInfo {
    unsigned int run;
    unsigned int verbose;
    unsigned int debug;
    char thread_name[THREAD_NAME_LENGTH];
    pthread_t thread_id;
    UdpServerInfo udp_server[UDP_SERVERS];
    unsigned int verbose_level;
    unsigned int loop_pace;
    unsigned char rx_buff[RX_UDP_PACKETS][UDP_PACKET_SIZE];
    unsigned char tx_buff[TX_UDP_PACKETS][UDP_PACKET_SIZE];
    unsigned char rx_len[RX_UDP_PACKETS];
    unsigned char tx_len[TX_UDP_PACKETS];
    int rx_buff_head;
    int rx_buff_tail;
    int rx_buff_mask;
    int tx_buff_head;
    int tx_buff_tail;
    int tx_buff_mask;
    struct _SerialThreadInfo *serial_thread_info; /* udp and serial threads know about each other */
} UdpThreadInfo;

typedef struct _SerialThreadInfo {
    int fd;
    int run;
    unsigned char rx_buff[RX_BUFFER_SIZE];
    unsigned char tx_buff[TX_BUFFER_SIZE];
    int rx_buff_head;
    int rx_buff_tail;
    int rx_buff_mask;
    int tx_buff_head;
    int tx_buff_tail;
    int tx_buff_mask;
    pthread_t tid;
    char thread_name[32];
    int loop_pace;
    struct _UdpThreadInfo *udp_thread_info; /* udp and serial threads know about each other */
} SerialThreadInfo;

/* function declarations */
int serial_room(SerialThreadInfo *info);
int udp_room(SerialThreadInfo *info);
int serial_xmit(SerialThreadInfo *info, uint8_t *tx_buff, int n);
int serial_recv(SerialThreadInfo *info, uint8_t *rx_buff, int n);

/**
 *
 * @brief initializes serial thread to its default parameters
 */
int initialize_serial_thread(SerialThreadInfo *info)
{
    memset(info, 0, sizeof (SerialThreadInfo));
    info->loop_pace = DEFAULT_LOOP_PACE;
    info->rx_buff_mask = RX_BUFFER_SIZE - 1;
    info->tx_buff_mask = TX_BUFFER_SIZE - 1;
    info->rx_buff_head = 0;
    info->tx_buff_head = 0;
    info->rx_buff_tail = 0;
    info->tx_buff_tail = 0;
    return ERROR_CODE_SUCCESS;
}

/**
 * @brief initializes udp thread to its default parameters
 */
int initialize_udp_thread(UdpThreadInfo *info)
{
    memset(info, 0, sizeof (UdpThreadInfo));
    info->udp_server->socket_timeout = 500;
    info->loop_pace = DEFAULT_LOOP_PACE;
    info->rx_buff_mask = RX_UDP_PACKETS - 1;
    info->tx_buff_mask = TX_UDP_PACKETS - 1;
    info->rx_buff_head = 0;
    info->tx_buff_head = 0;
    info->rx_buff_tail = 0;
    info->tx_buff_tail = 0;
    return ERROR_CODE_SUCCESS;
}

/**
 * @brief calculates elapsed time (in milliseconds) between now and the initialization time
 *
 * @return time in milliseconds between the function call and the first function call (initialization time)
 */
static uint32_t elapsed_time(void)
{
    struct timespec ts;
    static unsigned long long ts0_ms = 0;
    clock_gettime(CLOCK_REALTIME, &ts);
    unsigned long long secs = (unsigned long long) ts.tv_sec;
    unsigned long long nano = (unsigned long long) ts.tv_nsec;
    unsigned long long now = secs * one_thousand + nano / one_million;
    if (ts0_ms == 0) { ts0_ms = now; }
    unsigned long long ts_diff = (now - ts0_ms) & 0xffffffffLL;
    return ts_diff;
}

/**
 * @brief creates a udp socket listening to a port
 * @param server - describes the udp server
 * @param port - a valid system udp port. If port == 0, then server->udp_port must be set
 * @return
 */
static int create_udp_socket(UdpServerInfo *server, int port)
{
    if ((server->socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { // IPPROTO_UDP
        perror("socket creation failed");
        return ERROR_CODE_FAILURE;
    }

    if (port) { server->udp_port = port; }
    printf("socket created %d\n", server->socket_fd);
    memset(&server->server_addr, 0, sizeof(server->server_addr));
    server->server_addr.sin_family = AF_INET; /* ipv4 */
    server->server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server->server_addr.sin_port = htons(server->udp_port);

    printf("socket bound\n"); /** bind socket with server address */
    if (bind(server->socket_fd, (const struct sockaddr *) &server->server_addr, sizeof(server->server_addr)) < 0 )
    {
        if (server->verbose_level >= VERBOSE_LEVEL_INFO) { printf("bind failed\n"); }
        perror("bind failed");
        return ERROR_CODE_FAILURE;
    }

    /** reset statistics and counters */
    server->n_rx = 0;
    server->n_tx = 0;
    server->sunset_expiry = elapsed_time() + server->sunset_period; /** must receive some signs of life before sunset */

    /** state variables */
    server->connection_state = CONNECTION_STATE_WAIT_CONNECT;
    server->connection_state_timeout = 0;

    return ERROR_CODE_SUCCESS;
}

/**
 * @brief check for incoming data from socket, or to transmit outgoing data.
 * @param thread_info - describes udp thread
 * @return
 *     thread_info->wset is 1 if port can transmit; 0 otherwise.
 *     thread_info->rset is 1 if data available; 0 otherwise
 */
int check_udp_server(UdpServerInfo *server)
{
    /**
     * select() is used to provide a natural "sleep" mechanism between loop iterations
     */

    /* prepare socket operation timeout */
    struct timeval socket_timeout;
    unsigned long long micros = server->socket_timeout * one_thousand; /* microseconds of timeout */
    socket_timeout.tv_sec = server->socket_timeout / one_million; /* number of seconds */
    socket_timeout.tv_usec = micros - socket_timeout.tv_sec * one_million; /* remove seconds from the total */

    fd_set rset;
    fd_set wset;

    /** get maximum socket fd and populate rset, wset for use by select() */
    FD_ZERO(&rset);
    FD_ZERO(&wset);
    int max_fd = -1;
    int socket_fd = server->socket_fd;
    if (socket_fd > max_fd) { max_fd = socket_fd; }
    FD_SET(socket_fd, &rset);
    FD_SET(socket_fd, &wset);
    int status = select(max_fd + 1, &rset, &wset, NULL, &socket_timeout);
    if (status <= 0) { return ERROR_CODE_NOT_READY; }

    server->rset_flag = (FD_ISSET(server->socket_fd, &rset)) ? 1 : 0;
    server->wset_flag = (FD_ISSET(server->socket_fd, &wset)) ? 1 : 0;

    return ERROR_CODE_SUCCESS;
}

SerialThreadInfo serial_thread_info;
UdpThreadInfo udp_thread_info;

void delay_ms(unsigned int ms) {
    struct timeval tv;
    tv.tv_sec = ms / 1000ULL;
    tv.tv_usec = ms * 1000ULL;
    select(0, NULL, NULL, NULL, &tv);
}

void *udp_thread(void *args)
{
    UdpThreadInfo *info = (UdpThreadInfo *) args;
    while (info->run == 0) { delay_ms(1); }
    UdpServerInfo *server = info->udp_server;
    while (info->run) {
        check_udp_server(server);
        if (server->rset_flag) {
            /* read out udp packet only if space is available */
            int new_head = (info->rx_buff_head + 1) & info->rx_buff_mask;
            if (new_head != info->rx_buff_tail) { /* space is available */
                socklen_t length = sizeof (server->client_addr);
                uint8_t *rx_buff = info->rx_buff[info->rx_buff_head];
                unsigned int rx_size = sizeof (info->rx_buff[0]);
                ssize_t rx_bytes = recvfrom(server->socket_fd, rx_buff, rx_size, 0, (struct sockaddr *) &server->client_addr, &length);
                int tx_bytes = serial_xmit(info->serial_thread_info, rx_buff, rx_bytes);
                info->rx_buff_head = new_head;
                if (tx_bytes < rx_bytes) { fprintf(stderr, "potential data loss udp=%zd. serial = %d\n", rx_bytes, tx_bytes); }
            } else {
                fprintf(stderr, "udp rx buffer full (capacity = %d)\n", info->rx_buff_mask);
            }
        }
        if (info->udp_server->wset_flag) {
            if (info->tx_buff_head != info->tx_buff_tail) {
                uint8_t *tx_buff = info->tx_buff[info->tx_buff_head];
                unsigned int tx_len = info->tx_len[info->tx_buff_head];
                ssize_t tx_bytes = sendto(server->socket_fd, tx_buff, tx_len, 0, (struct sockaddr *) &server->client_addr, sizeof (server->client_addr));
                if (tx_bytes != tx_len) { fprintf(stderr, "udp unable to transmit entire packet %zd / %d", tx_bytes, tx_len); }
            }
        }
    }
}

void *serial_thread(void *arg)
{
    SerialThreadInfo *info = (SerialThreadInfo *) arg;
    while (info->run == 0) { delay_ms(1); }
    while (info->run) {
        delay_ms(1);
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = info->loop_pace * 1000;
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
            int room = 0;
            if (info->rx_buff_head < info->tx_buff_tail) {
                room = info->rx_buff_tail - info->rx_buff_head - 1;
            } else {
                room = info->rx_buff_mask - info->rx_buff_head;
            }
            if (room > 0) {
                int n = read(info->fd, &info->rx_buff[info->rx_buff_head], room);
                if (n > 0) {
                    info->rx_buff_head = (info->rx_buff_head + n) & info->rx_buff_mask;
                }
            }
        }

        if (wset_flag) {
            if (info->tx_buff_head != info->tx_buff_tail) {
                int n_send = (info->tx_buff_head - info->tx_buff_tail) & info->tx_buff_mask;
                uint8_t *tx_buff = info->tx_buff + info->tx_buff_tail;
                int n_sent = write(info->fd, tx_buff, n_send);
                if (n_sent > 0) {
                    info->tx_buff_tail = (info->tx_buff_tail + n_sent) & info->tx_buff_mask;
                }
            }
        }

    }
}

int serial_xmit(SerialThreadInfo *info, uint8_t *tx_buff, int n)
{
    int idx = 0;
    int new_head = (info->tx_buff_head + 1) & info->tx_buff_mask;
    while (new_head != info->tx_buff_tail) {
        info->tx_buff[info->rx_buff_head] = tx_buff[idx++];
        info->tx_buff_head = new_head;
        if (idx == n) { break; }
        new_head = (info->tx_buff_head + 1) & info->tx_buff_mask;
    }
    return idx;
}

int serial_recv(SerialThreadInfo *info, uint8_t *rx_buff, int n)
{
    int idx = 0;
    while (info->rx_buff_tail != info->rx_buff_head) {
        rx_buff[idx++] = info->rx_buff[info->rx_buff_tail];
        info->rx_buff_tail = (info->rx_buff_tail + 1) & info->rx_buff_mask;
        if (idx == n) { break; }
    }
    return idx;
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
    int udp_port = 55151;

    /* serial and udp threads start in known positions */
    initialize_serial_thread(&serial_thread_info);
    initialize_udp_thread(&udp_thread_info);

    /* serial and udp threads bind to each other */
    udp_thread_info.serial_thread_info = &serial_thread_info;
    serial_thread_info.udp_thread_info = &udp_thread_info;

    snprintf(dev_name, sizeof (dev_name), "/dev/ttyUSB0");
    for (int i = 0; i < argc; ++i) {
        if (strcmp(argv[i], "-debug") == 0) {
            debug = 1;
		} else if (strcmp(argv[i], "-m") == 0) {
            ++i;
            serial_xmit(&serial_thread_info, argv[i], strlen(argv[i]));
		} else if (strcmp(argv[i], "-d") == 0) {
            strcpy(dev_name, argv[++i]);
        } else if (strcmp(argv[i], "-listen") == 0) {
            do_listen = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-port") == 0) {
            udp_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-flush") == 0) {
            do_flush = 1;
        }
    }

    serial_thread_info.fd = open(dev_name, O_NOCTTY | O_RDWR);
    tcgetattr(serial_thread_info.fd, &termios);
    cfmakeraw(&termios);
    termios.c_cflag &= ~(CSTOPB | CRTSCTS | CSIZE);
    termios.c_cflag |= (CS8 | CLOCAL);
    cfsetspeed(&termios, baud_code);
    termios.c_cc[VMIN] = vmin;
    termios.c_cc[VTIME] = vtime;
    tcsetattr(serial_thread_info.fd, TCSAFLUSH, &termios);

    serial_thread_info.run = 1;
    udp_thread_info.run = 1;

    create_udp_socket(udp_thread_info.udp_server, udp_port);

    int status = pthread_create(&serial_thread_info.tid, NULL, serial_thread, &serial_thread_info);
    if (status != 0) {
        fprintf(stderr, "unable to create serial thread\n");
        return -1;
    }
    snprintf(serial_thread_info.thread_name, sizeof (serial_thread_info.thread_name), "ser-jocat");
    pthread_setname_np(serial_thread_info.tid, serial_thread_info.thread_name);

    status = pthread_create(&udp_thread_info.thread_id, NULL, udp_thread, &udp_thread_info);
    if (status != 0) {
        fprintf(stderr, "unable to create udp thread\n");
        return -1;
    }
    snprintf(udp_thread_info.thread_name, sizeof (udp_thread_info.thread_name), "udp-jocat");
    pthread_setname_np(udp_thread_info.thread_id, udp_thread_info.thread_name);

    if (do_listen) {
        sleep(do_listen);
    } else {
        while (1) {
            sleep(1);
        }
    }

    uint8_t rx_buff[RX_BUFFER_SIZE];
    int n = serial_recv(&serial_thread_info, rx_buff, sizeof (rx_buff) - 1);
    rx_buff[n] = 0;
    fprintf(stdout, "=> %s <=\n", rx_buff);
    return n;
}

/* potential pile head */
#if 0
int serial_room(SerialThreadInfo *info)
{
    int room = (info->rx_buff_tail - info->rx_buff_head - 1) & info->rx_buff_mask;
    return room;
}

int udp_room(SerialThreadInfo *info)
{
    int room = (info->rx_buff_tail - info->rx_buff_head - 1) & info->rx_buff_mask;
    return room;
}
#endif


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
#include <sys/ioctl.h>

#include "serial-port.h"

#define UDP_PACKET_SIZE (1024)
#define SERIAL_RX_BUFFER_SIZE UDP_PACKET_SIZE
#define SERIAL_TX_BUFFER_SIZE UDP_PACKET_SIZE
#define SERIAL_TX_BUFFERS (16)
#define SERIAL_RX_BUFFERS (1)
#define RX_UDP_PACKETS (16)
#define TX_UDP_PACKETS (16)
#define DEFAULT_LOOP_PACE (10)

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
    int echo_flag;
    int tx_dir_active;
    int run;
} UdpServerInfo;

struct _SerialThreadInfo;

typedef struct _UdpThreadInfo {
    unsigned int run;
    unsigned int verbose;
    unsigned int debug;
    char thread_name[THREAD_NAME_LENGTH];
    pthread_t thread_id;
    UdpServerInfo udp_server;
    unsigned int verbose_level;
    unsigned int loop_pace;
    unsigned char rx_buff[RX_UDP_PACKETS][UDP_PACKET_SIZE];
    unsigned char tx_buff[TX_UDP_PACKETS][UDP_PACKET_SIZE];
    unsigned int rx_len[RX_UDP_PACKETS];
    unsigned int tx_len[TX_UDP_PACKETS];
    int rx_buff_head;
    int rx_buff_tail;
    int rx_buff_mask;
    int tx_buff_head;
    int tx_buff_tail;
    int tx_buff_mask;
    struct _SerialThreadInfo *serial_thread_info; /* udp and serial threads know about each other */
    int closed_loop;
    int echo_flag;
    int done;
} UdpThreadInfo;

typedef UdpThreadInfo CmdThreadInfo;

typedef struct _SerialThreadInfo {
    int fd;
    int run;
    int debug;
    int verbose;
    unsigned char rx_buff[SERIAL_RX_BUFFERS][SERIAL_RX_BUFFER_SIZE];
    unsigned char tx_buff[SERIAL_TX_BUFFERS][SERIAL_TX_BUFFER_SIZE];
    unsigned int rx_len[SERIAL_RX_BUFFERS];
    unsigned int tx_len[SERIAL_TX_BUFFERS];
    int rx_buff_head;
    int rx_buff_tail;
    int rx_buff_mask;
    int tx_buff_head;
    int tx_buff_tail;
    int tx_buff_mask;
    pthread_t thread_id;
    char thread_name[32];
    int loop_pace;
    struct _UdpThreadInfo *udp_thread_info; /* udp and serial threads know about each other */
    int closed_loop;
    int done;
} SerThreadInfo;

/* function declarations */
int serial_room(SerThreadInfo *info);
int udp_room(SerThreadInfo *info);
int serial_xmit(SerThreadInfo *info, uint8_t *tx_buff, int n);
int serial_recv(SerThreadInfo *info, uint8_t *rx_buff, int n);

/**
 *
 * @brief initializes serial thread to its default parameters
 */
int initialize_serial_thread(SerThreadInfo *info)
{
    memset(info, 0, sizeof (SerThreadInfo));
    info->loop_pace = DEFAULT_LOOP_PACE;
    info->rx_buff_mask = SERIAL_RX_BUFFERS - 1;
    info->tx_buff_mask = SERIAL_TX_BUFFERS - 1;
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
    info->udp_server.socket_timeout = 500;
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
    fd_set *pset = (server->tx_dir_active) ? &wset : NULL; /* if we're not activating transmit path */
    int status = select(max_fd + 1, &rset, pset, NULL, &socket_timeout);
    if (status < 0) { return ERROR_CODE_NOT_READY; }

    server->rset_flag = (FD_ISSET(server->socket_fd, &rset)) ? 1 : 0;
    server->wset_flag = (FD_ISSET(server->socket_fd, &wset)) ? 1 : 0;

    return ERROR_CODE_SUCCESS;
}

SerThreadInfo ser_thread_info;
UdpThreadInfo udp_thread_info;
CmdThreadInfo cmd_thread_info;

void delay_ms(unsigned int ms) {
    struct timeval tv;
    tv.tv_sec = ms / 1000ULL;
    tv.tv_usec = ms * 1000ULL;
    select(0, NULL, NULL, NULL, &tv);
}

typedef int (*cmd_callback)(UdpThreadInfo *info, void *args, uint8_t *cmd, unsigned int cmd_len);

int process_dtr(int fd, int flag)
{
    int status;
    struct termios tio;
    tcgetattr(fd, &tio);
    tio.c_cflag &= ~(HUPCL);
    tcsetattr(fd, TCSANOW, &tio);
    ioctl(fd, TIOCMGET, &status);
    switch (flag) {
    case '0': case 0: { status &= ~(TIOCM_DTR); break; }
    case '1': case 1: { status |= (TIOCM_DTR); break; }
    default: { break; }
    }
    ioctl(fd, TIOCMSET, &status);
    return ERROR_CODE_SUCCESS;
}

int process_rts(int fd, int flag)
{
    int status;
    struct termios tio;
    tcgetattr(fd, &tio);
    tio.c_cflag &= ~(HUPCL);
    tcsetattr(fd, TCSANOW, &tio);
    ioctl(fd, TIOCMGET, &status);
    switch (flag) {
    case '0': case 0: { status &= ~(TIOCM_RTS); break; }
    case '1': case 1: { status |= (TIOCM_RTS); break; }
    default: { break; }
    }
    ioctl(fd, TIOCMSET, &status);
    return ERROR_CODE_SUCCESS;
}

int process_command(CmdThreadInfo *info, uint8_t *rx_buff, unsigned int rx_bytes)
{
    static const char dtr_command_str[] = "$DTR,";
    static unsigned int dtr_command_len = sizeof (dtr_command_str) - 1;
    static const char rts_command_str[] = "$RTS,";
    static unsigned int rts_command_len = sizeof (rts_command_str) - 1;
    static const char bye_command_str[] = "$BYE*";
    static unsigned int bye_command_len = sizeof (bye_command_str) - 1;

    int status = ERROR_CODE_FAILURE; /* guilty until proven innocent */
    if (memcmp(rx_buff, dtr_command_str, dtr_command_len) == 0) {
        int flag = (rx_buff[dtr_command_len] == '0') ? 0 : 1;
        flag = flag ? 0 : 1;
        status = process_dtr(info->serial_thread_info->fd, flag);
    } else if (memcmp(rx_buff, rts_command_str, rts_command_len) == 0) {
        int flag = (rx_buff[rts_command_len] == '0') ? 0 : 1;
        flag = flag ? 0 : 1;
        status = process_rts(info->serial_thread_info->fd, flag);
    } else if (memcmp(rx_buff, bye_command_str, bye_command_len) == 0) {
        info->udp_server.run = 0;
        status = ERROR_CODE_SUCCESS;
    }
    return status;
}

void *cmd_thread(void *args)
{
    CmdThreadInfo *info = (CmdThreadInfo *) args;
    while (info->run == 0) { delay_ms(1); }
    UdpServerInfo *server = &info->udp_server;
    while (info->run) {
        check_udp_server(server);
        if (server->rset_flag) { /* incoming data contains commands for server */
            /* read out udp packet only if space is available */
            int new_head = (info->rx_buff_head + 1) & info->rx_buff_mask;
            if ((info->closed_loop && (new_head != info->rx_buff_tail)) || (info->closed_loop == 0)) { /* space is available */
                socklen_t length = sizeof (server->client_addr);
                uint8_t *rx_buff = info->rx_buff[info->rx_buff_head];
                unsigned int rx_size = sizeof (info->rx_buff[0]);
                ssize_t rx_bytes = recvfrom(server->socket_fd, rx_buff, rx_size, 0, (struct sockaddr *) &server->client_addr, &length);
                /* TODO echoes */ // sendto(server->socket_fd, rx_buff, rx_size, 0, (struct sockaddr *) &server->client_addr, sizeof (server->client_addr));
                if (info->verbose) {
                    fprintf(stderr, "cmmd [%s] received with length = %zd\n", rx_buff, rx_bytes);
//                    for (int i = 0; i < rx_bytes; ++i) {
//                        fprintf(stderr, "%2.2d ", rx_buff[i]);
//                        if (i && ((i % 16) == 0)) { fprintf(stderr, "\n"); }
//                    }
//                    fprintf(stderr, "\n");
                }
                process_command(info, rx_buff, rx_bytes);
                info->rx_buff_head = new_head;
            } else {
                fprintf(stderr, "udp rx buffer full (capacity = %zd)\n", info->rx_buff_mask * sizeof (info->rx_buff[0]));
            }
        }

        /* under normal operating conditions, this code is not really exercised. data going to client comes from uart */
        if (server->wset_flag) { /* outgoing data already in tx buffers */
            if (info->tx_buff_tail != info->tx_buff_head) {
                uint8_t *tx_buff = info->tx_buff[info->tx_buff_tail];
                unsigned int tx_len = info->tx_len[info->tx_buff_tail];
                ssize_t tx_bytes = sendto(server->socket_fd, tx_buff, tx_len, 0, (struct sockaddr *) &server->client_addr, sizeof (server->client_addr));
                if ((tx_bytes > 0) && (tx_bytes != tx_len)) { fprintf(stderr, "udp unable to transmit entire packet %zd / %d", tx_bytes, tx_len); }
                info->tx_buff_tail = (info->tx_buff_tail + 1) & info->tx_buff_mask;
            }
        }

    }
    /* going home */
    info->done = 1;
    return NULL;
}

void *udp_thread(void *args)
{
    UdpThreadInfo *info = (UdpThreadInfo *) args;
    while (info->run == 0) { delay_ms(1); }
    UdpServerInfo *server = &info->udp_server;
    while (info->run) {
        check_udp_server(server);

        if (server->rset_flag) { /* incoming data from udp goes to serial port */
            /* read out udp packet only if space is available */
            int new_head = (info->rx_buff_head + 1) & info->rx_buff_mask;
            if ((info->closed_loop && (new_head != info->rx_buff_tail)) || (info->closed_loop == 0)) { /* space is available */
                socklen_t length = sizeof (server->client_addr);
                uint8_t *rx_buff = info->rx_buff[info->rx_buff_head];
                unsigned int rx_size = sizeof (info->rx_buff[0]);
                ssize_t rx_bytes = recvfrom(server->socket_fd, rx_buff, rx_size, 0, (struct sockaddr *) &server->client_addr, &length);
                int tx_bytes = serial_xmit(info->serial_thread_info, rx_buff, rx_bytes);
                /* TODO echoes */ // sendto(server->socket_fd, rx_buff, rx_size, 0, (struct sockaddr *) &server->client_addr, sizeof (server->client_addr));
                if (info->verbose) {
                    fprintf(stderr, "data received with length = %zd. sent to uart\n", rx_bytes);
                    for (int i = 0; i < rx_bytes; ++i) {
                        fprintf(stderr, "%2.2d ", rx_buff[i]);
                        if (i && ((i % 16) == 0)) { fprintf(stderr, "\n"); }
                    }
                    fprintf(stderr, "\n");
                }
                info->rx_buff_head = new_head;
                if (tx_bytes < rx_bytes) { fprintf(stderr, "potential data loss udp=%zd. serial = %d\n", rx_bytes, tx_bytes); }
            } else {
                fprintf(stderr, "udp rx buffer full (capacity = %zd)\n", info->rx_buff_mask * sizeof (info->rx_buff[0]));
            }
        }

        if (server->wset_flag) { /* outgoing data already in tx buffers */
            if (info->tx_buff_tail != info->tx_buff_head) {
                uint8_t *tx_buff = info->tx_buff[info->tx_buff_tail];
                unsigned int tx_len = info->tx_len[info->tx_buff_tail];
                ssize_t tx_bytes = sendto(server->socket_fd, tx_buff, tx_len, 0, (struct sockaddr *) &server->client_addr, sizeof (server->client_addr));
                if ((tx_bytes > 0) && (tx_bytes != tx_len)) { fprintf(stderr, "udp unable to transmit entire packet %zd / %d", tx_bytes, tx_len); }
                info->tx_buff_tail = (info->tx_buff_tail + 1) & info->tx_buff_mask;
            }
        }
    }
    /* going home */
    info->done = 1;
    return NULL;
}

int udp_xmit(UdpThreadInfo *info, uint8_t *buff, unsigned int len)
{
    unsigned int new_head = (info->tx_buff_head + 1) & info->tx_buff_mask;
    int n = 0;
    if (new_head != info->tx_buff_tail) {
        uint8_t *tx_buff = info->tx_buff[info->tx_buff_head];
        unsigned int max_len = sizeof (info->tx_buff[0]);
        n = (len < max_len) ? len : max_len;
        memcpy(tx_buff, buff, n);
        info->tx_len[info->tx_buff_head] = n;
        info->tx_buff_head = new_head;
    }
    return n;
}

void *ser_thread(void *arg)
{
    SerThreadInfo *info = (SerThreadInfo *) arg;
    while (info->run == 0) { delay_ms(1); }
    while (info->run) {
        delay_ms(1);
        struct timeval timeout;
        timeout.tv_sec = 1; /* TODO JSV */
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

        if (rset_flag) { /* read from uart and populate rx buffer (into head) */
            static uint8_t tmp_rx_buff[7 * SERIAL_RX_BUFFER_SIZE / 8]; /* stay away from a complete buffer */
            int tmp_rx_len = read(info->fd, tmp_rx_buff, sizeof (tmp_rx_buff));
            udp_xmit(info->udp_thread_info, tmp_rx_buff, tmp_rx_len);
            if (info->verbose) {
                fprintf(stderr, "uart rx %d bytes, tx to udp\n", tmp_rx_len);
                for (int i = 0; i < tmp_rx_len; ++i) {
                    fprintf(stderr, "%2.2d ", tmp_rx_buff[i]);
                    if (i && ((i % 16) == 0)) { fprintf(stderr, "\n"); }
                }
                fprintf(stderr, "\n");
            }
        }

        if (wset_flag) {
            if (info->tx_buff_head != info->tx_buff_tail) {
                uint8_t *tx_buff = info->tx_buff[info->tx_buff_tail];
                int n_send = info->tx_len[info->tx_buff_tail];
                int n_sent = n_send;
                if (info->fd) { n_sent = write(info->fd, tx_buff, n_send); }
                if (info->verbose) {
                    fprintf(stderr, "data(length = %d) emitted on uart\n", n_sent);
                    for (int i = 0; i < n_sent; ++i) {
                        fprintf(stderr, "%2.2d ", tx_buff[i]);
                        if (i && ((i % 16) == 0)) { fprintf(stderr, "\n"); }
                    }
                    fprintf(stderr, "\n");
                }
                if (n_sent > 0) {
                    info->tx_buff_tail = (info->tx_buff_tail + 1) & info->tx_buff_mask;
                }
            }
        }
    }
    /* going home */
    info->done = 1;
    return NULL;
}

int serial_xmit(SerThreadInfo *info, uint8_t *tx_buff, int n)
{
    int new_head = (info->tx_buff_head + 1) & info->tx_buff_mask;
    if (n > sizeof (info->tx_buff[0])) { return 0; } /* cannot do it */
    if (new_head != info->tx_buff_tail) {
        memcpy(info->tx_buff[info->tx_buff_head], tx_buff, n);
        info->tx_len[info->tx_buff_head] = n;
        info->tx_buff_head = new_head;
    }
    return n;
}

int serial_recv(SerThreadInfo *info, uint8_t *rx_buff, int n)
{
    int idx = 0;
    while (info->rx_buff_tail != info->rx_buff_head) {
        rx_buff[idx] = info->rx_buff[info->rx_buff_tail][idx];
        ++idx;
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
    // int baud = 115200;
    int baud_code = B115200;
    baud_code = B9600; /* TODO */
    int udp_port = 55151;
    int cmd_port = 55152;
    int status;
    int do_sim = 0;
    int parity = 2;

    /* serial and udp threads start in known positions */
    initialize_serial_thread(&ser_thread_info);
    initialize_udp_thread(&udp_thread_info);
    initialize_udp_thread(&cmd_thread_info);

    /* serial and udp threads bind to each other */
    udp_thread_info.serial_thread_info = &ser_thread_info;
    cmd_thread_info.serial_thread_info = &ser_thread_info;
    ser_thread_info.udp_thread_info = &udp_thread_info;

    snprintf(dev_name, sizeof (dev_name), "/dev/ttyUSB0");
    for (int i = 0; i < argc; ++i) {
        if (strcmp(argv[i], "-debug") == 0) {
            debug = 1;
		} else if (strcmp(argv[i], "-d") == 0) {
            snprintf(dev_name, sizeof (dev_name), "%s", argv[++i]);
        } else if (strcmp(argv[i], "-listen") == 0) {
            do_listen = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-port") == 0) {
            udp_port = atoi(argv[++i]);
            cmd_port = udp_port + 1;
        } else if (strcmp(argv[i], "-verbose") == 0) {
            ser_thread_info.verbose = 1;
            udp_thread_info.verbose = 1;
            cmd_thread_info.verbose = 1;
        } else if (strcmp(argv[i], "-flush") == 0) {
            do_flush = 1;
        } else if (strcmp(argv[i], "-sim") == 0) {
            do_sim = 1;
        }
    }

    if (do_sim == 0) {
        ser_thread_info.fd = initialize_serial_port(dev_name, baud_code, 0, parity, 0);
        if (ser_thread_info.fd < 0) {
            fprintf(stderr, "unable to open uart [%s]\n", dev_name);
            exit(1);
        }
#if 0
        ser_thread_info.fd = open(dev_name, O_NOCTTY | O_RDWR | O_NONBLOCK);
        tcgetattr(ser_thread_info.fd, &termios);
        cfmakeraw(&termios);
        termios.c_cflag &= ~(CSTOPB | CRTSCTS | CSIZE | PARENB | PARODD);
        termios.c_cflag |= (CS8 | CLOCAL);
        cfsetspeed(&termios, baud_code);
        termios.c_cc[VMIN] = vmin;
        termios.c_cc[VTIME] = vtime;
        tcsetattr(ser_thread_info.fd, TCSAFLUSH, &termios);
#endif
    }
    else {
        ser_thread_info.fd = 0;
    }

    ser_thread_info.run = 1;
    udp_thread_info.run = 1;
    cmd_thread_info.run = 1;

    create_udp_socket(&udp_thread_info.udp_server, udp_port);
    create_udp_socket(&cmd_thread_info.udp_server, cmd_port);

    UdpServerInfo *cmd_server = &cmd_thread_info.udp_server;
    cmd_server->run = 1;

    /*** serial looper ***/
    status = pthread_create(&ser_thread_info.thread_id, NULL, ser_thread, &ser_thread_info);
    if (status != 0) {
        fprintf(stderr, "unable to create serial thread\n");
        return -1;
    }
    snprintf(ser_thread_info.thread_name, sizeof (ser_thread_info.thread_name), "ser-jocat");
//    pthread_setname_np(ser_thread_info.thread_id, ser_thread_info.thread_name);

    /*** data over udp looper ***/
    status = pthread_create(&udp_thread_info.thread_id, NULL, udp_thread, &udp_thread_info);
    if (status != 0) {
        fprintf(stderr, "unable to create udp thread\n");
        return -1;
    }
    snprintf(udp_thread_info.thread_name, sizeof (udp_thread_info.thread_name), "udp-jocat");
//    pthread_setname_np(udp_thread_info.thread_id, udp_thread_info.thread_name);

    /*** commands over looper ***/
    status = pthread_create(&cmd_thread_info.thread_id, NULL, cmd_thread, &cmd_thread_info);
    if (status != 0) {
        fprintf(stderr, "unable to create cmd thread\n");
        return -1;
    }
    snprintf(cmd_thread_info.thread_name, sizeof (cmd_thread_info.thread_name), "cmd-jocat");
//    pthread_setname_np(cmd_thread_info.thread_id, cmd_thread_info.thread_name);

    while (cmd_server->run) {
        sleep(1);
    }

    cmd_thread_info.run = 0;
    udp_thread_info.run = 0;
    ser_thread_info.run = 0;

    void *ser_exit_status = 0;
    void *cmd_exit_status = 0;
    void *udp_exit_status = 0;

    pthread_join(ser_thread_info.thread_id, &ser_exit_status);
    printf("waiting for thread [%s] to terminate...\n", ser_thread_info.thread_name);
    while (ser_thread_info.done == 0) { delay_ms(250); }
    printf("thread [%s] terminated. status = %p\n", ser_thread_info.thread_name, ser_exit_status);

    close(ser_thread_info.fd);
    ser_thread_info.fd = 0;

    pthread_join(udp_thread_info.thread_id, &udp_exit_status);
    printf("waiting for thread [%s] to terminate...\n", udp_thread_info.thread_name);
    while (udp_thread_info.done == 0) { delay_ms(250); }
    printf("thread [%s] terminated. status = %p\n", udp_thread_info.thread_name, udp_exit_status);

    pthread_join(cmd_thread_info.thread_id, &cmd_exit_status);
    printf("waiting for thread [%s] to terminate...\n", cmd_thread_info.thread_name);
    while (cmd_thread_info.done == 0) { delay_ms(250); }
    printf("thread [%s] terminated. status = %p\n", cmd_thread_info.thread_name, cmd_exit_status);

    return 0;
}

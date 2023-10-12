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

#define DEFAULT_PORT 55151
#define MAXLINE 1024
#define MAX_VEHICLES (256)

#include "vdm.h"

typedef struct {
    int run;
    int done;
    int socket_fd;
    int port;
    int verbose;
    pthread_t thread_id;
    unsigned int vehicle_data_head;
    unsigned int vehicle_data_tail;
    unsigned int vehicle_data_mask;
    unsigned int vehicle_data_pos;
    vehicle_data_t vehicle_data[MAX_VEHICLES];
    char thread_name[32];
} stm_interface_stack_t;

static stm_interface_stack_t g_stm_interface;

static void delay_ms(unsigned int ms) {
    struct timeval tv;
    tv.tv_sec = ms / 1000ULL;
    tv.tv_usec = ms * 1000ULL;
    select(0, NULL, NULL, NULL, &tv);
}

/**
 * @brief check for incoming data from socket, or to transmit outgoing data.
 * @param socket_fd -- socket descriptor for udp port
 * @return 1 if data is ready on the socket; 0 otherwise
 */
static unsigned int check_socket(int socket_fd)
{
    /* prepare socket operation timeout */
    struct timeval socket_timeout;
    unsigned long long micros = 100000;
    socket_timeout.tv_sec = 0; /* number of seconds */
    socket_timeout.tv_usec = micros;

    /** get maximum socket fd and populate rset, wset for use by select() */
    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(socket_fd, &rset);
    int status = select(socket_fd + 1, &rset, NULL, NULL, &socket_timeout);
    if (status <= 0) { return -1; }

    return (FD_ISSET(socket_fd, &rset)) ? 1 : 0;
}

void *stm_interface_task(void *arg)
{
    char buffer[MAXLINE] = { 0 };
    struct sockaddr_in servaddr;
    stm_interface_stack_t *stack = (stm_interface_stack_t *) arg;
    if ((stack->socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    stack->run = 1;

    memset(&servaddr, 0, sizeof(servaddr));

    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(stack->port);
    servaddr.sin_addr.s_addr = INADDR_ANY;

    const char command[] = "$vehicle,1*\r\n";
    unsigned int command_len = sizeof (command) + 1;
    sendto(stack->socket_fd, command, command_len, 0, (const struct sockaddr *) &servaddr, sizeof(servaddr));

    while (stack->run) {
        int flag = check_socket(stack->socket_fd);
        if (flag == 1) {
            int len = sizeof (servaddr);
            int n = recvfrom(stack->socket_fd, buffer, sizeof (buffer), 0, (struct sockaddr *) &servaddr, &len);
            if (stack->verbose) {
                fprintf(stderr, "recvfrom(%p, %ld) => %d\n", buffer, sizeof(buffer), n);
                if (n > 0) {
                    for (int i = 0; i < n; ++i) {
                        char ch = buffer[i];
                        if (ch != '\r') {
                            fprintf(stderr, "%c", buffer[i]);
                        }
                    }
                }
            }
        }
    }

    close(stack->socket_fd);
    stack->done = 1;

    return NULL;
}

int initialize_stm_interface(void *p_stack)
{
    stm_interface_stack_t *stack = (stm_interface_stack_t *) p_stack;
    if (stack == NULL) { stack = &g_stm_interface; }
    stack->vehicle_data_mask = MAX_VEHICLES - 1;

    stack->verbose = 1;
    stack->port = DEFAULT_PORT;

    int status = pthread_create(&stack->thread_id, NULL, stm_interface_task, stack);
    if (status != 0) {
        fprintf(stderr, "unable to create serial thread\n");
        return -1;
    }
    snprintf(stack->thread_name, sizeof (stack->thread_name), "stm-xface");
#ifdef SUPPORT_THREAD_NAMES
    pthread_setname_np(stack->thread_id, stack->thread_name);
#endif
    return 0;
}

int close_stm_interface(void *p_stack)
{
    stm_interface_stack_t *stack = (stm_interface_stack_t *) p_stack;
    if (stack == NULL) { stack = &g_stm_interface; }
    void *exit_status;
    stack->run = 0;
    pthread_join(stack->thread_id, &exit_status);
    printf("waiting for thread [%s] to terminate...\n", stack->thread_name);
    while (stack->done == 0) { delay_ms(250); } /* TODO timeout */
    printf("thread [%s] terminated. status = %p\n", stack->thread_name, exit_status);
    return 0;
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
    unsigned long long now = secs * 1000ULL + nano / 1000000ULL;
    if (ts0_ms == 0) { ts0_ms = now; }
    unsigned long long ts_diff = (now - ts0_ms) & 0xffffffffLL;
    return ts_diff;
}

int main(int argc, char **argv) {
    int port = DEFAULT_PORT;
    int debug = 0;
    int verbose = 0;
    int do_listen = 0;
    ssize_t command_len = 0;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-verbose") == 0) {
            verbose = 1;
        } else if (strcmp(argv[i], "-debug") == 0) {
            debug = 1;
        } else if ((strcmp(argv[i], "-l") == 0) || (strcmp(argv[i], "--listen") == 0)) {
            do_listen = atoi(argv[++i]);
        } else if ((strcmp(argv[i], "-p") == 0) || (strcmp(argv[i], "--port") == 0)) {
            port = atoi(argv[++i]);
        }
    }

    uint32_t now = elapsed_time();
    uint32_t end_time = 0;
    if (do_listen) { end_time = now + do_listen; }
    initialize_stm_interface(NULL);
    while ((end_time == 0) || (now < end_time)) {
        now = elapsed_time();
    }
    close_stm_interface(NULL);

    return 0;
}

int consume_vehicle_data(void *p_stack, vehicle_data_t *vdm)
{
    stm_interface_stack_t *stack = (stm_interface_stack_t *) p_stack;
    if (stack == NULL) { stack = &g_stm_interface; }
    uint8_t uart_msg_string[256]; /* TODO */
    uint32_t stats = 0;
    int len = snprintf(uart_msg_string, sizeof (uart_msg_string), "$VEHICLE,%8.8x,%8.8x,%4.4x,%8.8x,%4.4x,%4.4x,%u,%u*\r\n", stats,
        vdm->gtime, vdm->ltime, vdm->ticks, vdm->speed, vdm->timer, vdm->n_can_msgs, vdm->index);
    return 0;
}

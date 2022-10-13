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

/**
 * @brief check for incoming data from socket, or to transmit outgoing data.
 * @param socket_fd -- socket descriptor for udp port
 * @return 1 if data is ready on the socket; 0 otherwise
 */
unsigned int check_socket(int socket_fd)
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

int main(int argc, char **argv) {
    int port = DEFAULT_PORT;
    int sockfd = 0;
    int debug = 0;
    int verbose = 0;
    int do_listen = 0;
    char buffer[MAXLINE] = { 0 };
    char command[MAXLINE] = { 0 };
    struct sockaddr_in servaddr;
    ssize_t command_len = 0;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-verbose") == 0) {
            verbose = 1;
        } else if (strcmp(argv[i], "-debug") == 0) {
            debug = 1;
        } else if ((strcmp(argv[i], "-m") == 0) || (strcmp(argv[i], "--message") == 0)) {
            command_len = snprintf(command, sizeof (command), "%s", argv[++i]);
        } else if ((strcmp(argv[i], "-l") == 0) || (strcmp(argv[i], "--listen") == 0)) {
            do_listen = atoi(argv[++i]);
        } else if ((strcmp(argv[i], "-p") == 0) || (strcmp(argv[i], "--port") == 0)) {
            port = atoi(argv[++i]);
        }
    }

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    }
    
    memset(&servaddr, 0, sizeof(servaddr)); 
        
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    servaddr.sin_addr.s_addr = INADDR_ANY;

    sendto(sockfd, command, command_len,0, (const struct sockaddr *) &servaddr, sizeof(servaddr));

    uint32_t now = elapsed_time();
    uint32_t end_time = now + do_listen;
    unsigned int rx_len = sizeof (buffer);
    unsigned int rx_pos = 0;
    while (now < end_time) {
        int len = sizeof (servaddr);
        int flag = check_socket(sockfd);
        if (flag == 1) {
            if (verbose) { fprintf(stderr, "recvfrom(%p, %d) ", &buffer[rx_pos], rx_len - rx_pos); }
            int n = recvfrom(sockfd, &buffer[rx_pos], rx_len - rx_pos,0, (struct sockaddr *) &servaddr, &len);
            if (n > 0) { rx_pos += n; }
            if (verbose) { fprintf(stderr, "returns %d. rx-pos = %d\n", n, rx_pos); }
        }
        now = elapsed_time();
        if (verbose) { fprintf(stderr, "now = %d waiting for %d\n", now, end_time); }
    }

    buffer[rx_pos] = '\0';
    printf("Server : %s\n", buffer); 
    
    close(sockfd); 
    return 0; 
}

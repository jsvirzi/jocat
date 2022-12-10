#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include "serial-port.h"

enum {
    STATE_IDLE = 0,
    STATE_GET_COMMAND,
    STATES
};

static uint8_t SEND_ACK = 0x79;

int main(int argc, char **argv)
{
    char dev_name[128];
    int debug = 0;
    int parity = 2;
    int baud_code = B115200;

    snprintf(dev_name, sizeof (dev_name), "/dev/ttyUSB0");
    for (int i = 0; i < argc; ++i) {
        if (strcmp(argv[i], "-debug") == 0) {
            debug = 1;
        } else if (strcmp(argv[i], "-d") == 0) {
            snprintf(dev_name, sizeof (dev_name), "%s", argv[++i]);
        }
    }

    int fd = initialize_serial_port(dev_name, baud_code, 0, parity, 0);

    uint8_t ibuff[1024];
    uint8_t obuff[1024];
    int state = 0;
    int state_counter;
    int state_expiry;
    while (1) {
        int n = read(fd, ibuff, sizeof (ibuff));
        if (n > 0) {
            for (int i = 0; i < n; ++i) {
                uint8_t byte = ibuff[i];
                switch (state) {
                    case STATE_IDLE: {
                        if (byte == 0x7f) {
                            obuff[0] = SEND_ACK;
                            write(fd, obuff, 1);
                        } else if (byte == 0x00) {
                            state = STATE_GET_COMMAND;
                        }
                        break;
                    }

                    case STATE_GET_COMMAND: {
                        if (byte == 0xff) {
                            obuff[0] = SEND_ACK;
                            obuff[0] = 9;
                            obuff[1] = 0x00;
                            obuff[2] = 0x01;
                            obuff[3] =
                        }
                        state = STATE_IDLE;
                        break;
                    }
                }
            }
        }
    }
}

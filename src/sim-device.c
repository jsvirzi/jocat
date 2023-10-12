#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>

#include "serial-port.h"

enum {
    STATE_IDLE = 0,
    STATE_GET_COMMAND,
    STATE_GET_VERSION,
    STATES
};

static const uint8_t SEND_ACK = 0x79;

static const uint8_t BOOTLOADER_VERSION = 0x10;
static const uint8_t COMMAND_GET = 0x00;
static const uint8_t COMMAND_GET_VERSION = 0x01;
static const uint8_t COMMAND_GET_ID = 0x02;
static const uint8_t COMMAND_READ_MEMORY = 0x11;
static const uint8_t COMMAND_GO = 0x21;
static const uint8_t COMMAND_WRITE_MEMORY = 0x31;
static const uint8_t COMMAND_ERASE = 0x43;
static const uint8_t COMMAND_ERASE_EXTENDED = 0x44;
static const uint8_t COMMAND_SPECIAL = 0x51;
static const uint8_t COMMAND_SPECIAL_EXTENDED = 0x53;
static const uint8_t COMMAND_WRITE_PROTECT = 0x63;
static const uint8_t COMMAND_WRITE_UNPROTECT = 0x73;
static const uint8_t COMMAND_READ_PROTECT = 0x82;
static const uint8_t COMMAND_READ_UNPROTECT = 0x92;
static const uint8_t COMMAND_GET_CHECKSUM = 0xa1;

int main(int argc, char **argv)
{
    char dev_name[128];
    int debug = 0;
    int parity = 2;
    int baud_code = B115200;

    baud_code = B9600; /* TODO */

    {
        uint8_t obuff[32];
        int index = 0;
        uint8_t crc = 0x00;
        uint8_t byte;
        byte = SEND_ACK; obuff[index++] = byte; crc ^= byte;
        byte = 0x10; obuff[index++] = byte; crc ^= byte;
        byte = 0x00; obuff[index++] = byte; crc ^= byte;
        byte = 0x01; obuff[index++] = byte; crc ^= byte;
        byte = 0x02; obuff[index++] = byte; crc ^= byte;
        byte = 0x11; obuff[index++] = byte; crc ^= byte;
        byte = 0x21; obuff[index++] = byte; crc ^= byte;
        byte = 0x31; obuff[index++] = byte; crc ^= byte;
        byte = 0x43; obuff[index++] = byte; crc ^= byte;
        byte = 0x63; obuff[index++] = byte; crc ^= byte;
        byte = 0x73; obuff[index++] = byte; crc ^= byte;
        byte = 0x82; obuff[index++] = byte; crc ^= byte;
        byte = 0x92; obuff[index++] = byte; crc ^= byte;
        byte = index - 2; obuff[1] = byte; crc ^= byte;
        obuff[++index] = crc;
    }

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
    uint8_t command;
    while (1) {
        int n = read(fd, ibuff, sizeof (ibuff));
        if (n > 0) {
            for (int i = 0; i < n; ++i) { printf("rx %d-byte = %2.2x\n", i, ibuff[i]); }
            for (int i = 0; i < n; ++i) {
                uint8_t byte = ibuff[i];
                uint8_t command_ok = (((byte + command) & 0xff) == 0xff) ? 1 : 0;
                switch (state) {
                    case STATE_IDLE: {
                        if (byte == 0x7f) {
                            obuff[0] = SEND_ACK;
                            write(fd, obuff, 1);
                            for (int i = 0; i < n; ++i) { printf("tx %d-byte = %2.2x\n", i, obuff[i]); }
                        } else if (byte == COMMAND_GET_VERSION) {
                            command = byte;
                            state = STATE_GET_VERSION;
                        } else if (byte == COMMAND_GET) {
                            command = byte;
                            state = STATE_GET_COMMAND;
                        }
                        break;
                    }

                    case STATE_GET_VERSION: {
                        if (command_ok) {
                            uint8_t crc = 0x00;
                            uint8_t byte;
                            int olen;
                            obuff[0] = SEND_ACK;
                            olen = 1;
                            write(fd, obuff, olen);
                            for (int i = 0; i < olen; ++i) { printf("tx %d-byte = %2.2x\n", i, obuff[i]); }
                            byte = 0x10; obuff[0] = byte; crc ^= byte;
                            byte = 0x00; obuff[1] = byte; crc ^= byte;
                            byte = 0x00; obuff[2] = byte; crc ^= byte;
                            obuff[3] = SEND_ACK;
                            olen = 4;
                            write(fd, obuff, olen);
                            for (int i = 0; i < olen; ++i) { printf("tx %d-byte = %2.2x\n", i, obuff[i]); }
                        }
                        state = STATE_IDLE; /* successful transaction or not, we end up here */
                        break;
                    }

                    case STATE_GET_COMMAND: {
                        if (command_ok) {
                            uint8_t crc = 0x00;
                            uint8_t byte;
                            obuff[0] = SEND_ACK;
                            int olen = 1;
                            write(fd, obuff, olen);
                            for (int i = 0; i < olen; ++i) { printf("tx %d-byte = %2.2x\n", i, obuff[i]); }
                            byte = 12; obuff[0] = byte; crc ^= byte;
                            byte = 0x10; obuff[1] = byte; crc ^= byte; /* version */
                            byte = COMMAND_GET; obuff[2] = byte; crc ^= byte;
                            byte = COMMAND_GET_VERSION; obuff[3] = byte; crc ^= byte;
                            byte = COMMAND_GET_ID; obuff[4] = byte; crc ^= byte;
                            byte = COMMAND_READ_MEMORY; obuff[5] = byte; crc ^= byte;
                            byte = COMMAND_GO; obuff[6] = byte; crc ^= byte;
                            byte = COMMAND_WRITE_MEMORY; obuff[7] = byte; crc ^= byte;
                            byte = COMMAND_ERASE_EXTENDED; obuff[8] = byte; crc ^= byte;
                            byte = COMMAND_WRITE_PROTECT; obuff[9] = byte; crc ^= byte;
                            byte = COMMAND_WRITE_UNPROTECT; obuff[10] = byte; crc ^= byte;
                            byte = COMMAND_READ_PROTECT; obuff[11] = byte; crc ^= byte;
                            byte = COMMAND_READ_UNPROTECT; obuff[12] = byte; crc ^= byte;
                            byte = COMMAND_GET_CHECKSUM; obuff[13] = byte; crc ^= byte;
                            // byte = crc; obuff[13] = crc;
                            obuff[14] = SEND_ACK;
                            olen = 15;
                            write(fd, obuff, olen);
                            for (int i = 0; i < olen; ++i) { printf("tx %d-byte = %2.2x\n", i, obuff[i]); }
                        }
                        state = STATE_IDLE;
                        break;
                    }
                }
            }
        }
    }
}

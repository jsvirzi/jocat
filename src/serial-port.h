#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

int initialize_serial_port(const char *dev, unsigned int baud, unsigned int canonical, int parity, int min_chars);

#endif


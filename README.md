# jocat - a home-grown version of socat. binds a udp socket to a serial port

## building jocat

    gcc jocat.c -o jocat -pthread

## usage

    jocat <-port 55151> <-d /dev/ttyUSB0> <-b 115200>



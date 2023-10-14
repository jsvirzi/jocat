# jocat - a home-grown version of socat. binds a udp socket to a serial port

## building jocat

### manually
    gcc jocat.c serial-port.c -pthread -o jocat
    gcc send-command.c -o send-command -pthread

### standard
    git clone ...
    cd jocat
    mkdir build
    cmake ..
    make

## usage

Run the following application, and keep it in the background

    jocat -port 55151 -d /dev/ttyUSB0 -b 115200 2>&1 &

Reset the STM while we're at it

    jocat -port 55151 -d /dev/ttyUSB2 -b 115200 -parity 0 -verbose -reset

Then, to send a command ("hello, world") to the serial port and receive a response (after waiting 2000ms)

    send-command -m "hello, world" -l 2000

The output to screen is the response from the serial port. Note: some output will be directed to stderr, 
which may show up on the screen. When capturing the output to a file, or environment variable, 
the output to stderr will not appear.

## shared library approach

The first step is to compile the dynamically linked library

    cd stm-interface
    gcc -fPIC -c -o lib/stm-interface.o src/stm-interface.c 
    gcc -shared -o lib/libstm-interface.so lib/stm-interface.o

The second step is to link to the shared library

    gcc -o stm-interface-test src/stm-interface-test.c -pthread -Llib -lstm-interface


# jocat - a home-grown version of socat. binds a udp socket to a serial port

## building jocat

### manually
    gcc jocat.c -o jocat -pthread
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

Then, to send a command ("hello, world") to the serial port and receive a response (after waiting 2000ms)

    send-command -m "hello, world" -l 2000

The output to screen is the response from the serial port. Note: some output will be directed to stderr, 
which may show up on the screen. When capturing the output to a file, or environment variable, 
the output to stderr will not appear.
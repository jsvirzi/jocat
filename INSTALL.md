# How To Use JOCAT and STM32FLASH

JOCAT installs and will run permanently, as a daemon or just a background task.
JOCAT creates a pipe between a serial port, and a UDP socket,
and is intended to be a permanent, exclusive access point to the serial port.
To communicate with the serial port, the user opens a UDP socket.
Data written to the UDP socket is streamed directly to the UART.
Conversely, data read from the UART is sent to the UDP socket.
There is no interpretation of data, or additional protocol.
This is not a TCP/IP connection; there is no guarantee of data delivery.

# Installation

In a non-daemon scenario, open two terminal windows. 
The ''-verbose" option will generate a lot of output on the screen.
Removal of this option will drastically reduce the output, and reduce the time spent programming.
It's recommended keeping this option active, until confidence is gained in the utility.
The ''firmware.hex" argument should be replaced with the target hex format.

In the first window

    git clone https://github.com/jsvirzi/jocat.git
    cd jocat/src
    gcc jocat.c serial-port.c -pthread -o jocat
    ./jocat -d /dev/ttyUSB2 -port 55151 -verbose

In the second window

    git clone https://github.com/jsvirzi/stm32flash.git
    cd stm32flash
    mkdir build
    cd build
    cmake ..
    make
    ./stm32flash -udp 55151 -w <firmware.hex> -verbose -g 0x0
 

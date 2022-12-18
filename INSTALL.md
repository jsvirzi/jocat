# Installation

In a non-daemon scenario, open two terminal windows. 
The ''-verbose" option will generate a lot of output on the screen.
Removal of this option will drastically reduce the output.
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
 

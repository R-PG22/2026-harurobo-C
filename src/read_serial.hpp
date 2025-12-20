#ifndef READ_SERIAL_HPP
#define READ_SERIAL_HPP

#include "mbed.h"

BufferedSerial pc(USBTX, USBRX, 115200);

class ReadSerial {
public:
    enum class State;
    State state;

    char buf[64];
    int buf_index;

};

#endif // READ_SERIAL_HPP
/*
Header to allow using the Kangaroo Arduino library, in a non-arduino environment.
The header implements all of Kangaroo's dependencies on Arduino.h.
Most notably the header provides a Stream implementation using a linux serial object
*/

#ifndef Arduino_h
#define Arduino_h

#include <stdint.h>
#include <cstddef>
#include "serial/serial.h"

typedef uint8_t byte;
typedef bool boolean;
unsigned long millis(void);
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

//define the Stream interface which the Kangaroo library requires 
//for serial objects
class Stream{
    public:
        virtual int read(void) = 0;
        virtual size_t write(const uint8_t *buffer, size_t size) = 0;
};

//implement the Arduino Stream interface using a serial object
class SerialStream : public Stream {
    serial::Serial& _serial;
    
    public:
        SerialStream(serial::Serial& serial);
        int read(void);
        size_t write(const uint8_t *buffer, size_t size);
};


#endif //Arduino.h

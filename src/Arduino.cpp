#include "Arduino.h"
#include <time.h>

using namespace std;

//program start time (for use in millis)
struct timespec ts_program_start;
int retCode = clock_gettime(CLOCK_MONOTONIC, &ts_program_start);

//grabbed from https://stackoverflow.com/questions/307596/time-difference-in-c
//for use in millis
int diff_ms(struct timespec t1, struct timespec t2)
{
    return (
                (t1.tv_sec - t2.tv_sec ) * 1000000 //sec to micro
             + 
                (t1.tv_nsec - t2.tv_nsec)/1000     //nano to micro
            ) / 1000; //micro to mill
}

SerialStream::SerialStream(serial::Serial& serial) : _serial(serial) {
};

int SerialStream::read(void) {
    if (!_serial.available()){
        return -1;
    } else {
        string received = _serial.read(1);
        return (byte) received[0];
    }
};

size_t SerialStream::write(const uint8_t *buffer, size_t size){
    _serial.write(buffer, size);
};

unsigned long millis(void) {
    struct timespec ts_current_time;
    struct timespec ts_diff;
    clock_gettime(CLOCK_MONOTONIC, &ts_current_time);
    unsigned long milliseconds = diff_ms(ts_current_time, ts_program_start);
    return milliseconds;
};


//https://inpyjama.com/post/ring-buffer/
#ifndef _RINGBUFFER_H
#define _RINGBUFFER_H

#include <Arduino.h> 

//Fixed buffer size of 512
// Just need buffer to be big enough to contain the worst case burst of data before writing to SD card
// SO if we are logging at 1 second, then for 100Hz, that is ~100 samples per second, so 512 should be more than enough.
// Want size to be power of 2 so & (N-1) works correctly as a bitwise modulo. 
// buffer size ~= sensor_rate * log_interval * safety_factor


//Summary of Ring Buffer:
// Fixed Size data structure that operates in CIRCULAR nature, where you keep track of the head and tail. 
// Head is where you are inserting data into the buffer. Tail is where you are reading the data.
// Since this is mostly used for logging purposes at a slower rate, tail is really the last place we have logged data up to. 
// So each time data is logged, it will log all data between tail--> head, then set tail == head to say that all data have been
// logged, and there is no new data. 
// This means...
// tail == head --> buffer is EMPTY because all data has been read
// We also keep the concept of "next", which is (head+1) & (511). This way, 

template <typename T, uint16_t N>   
class RingBuffer {

public:
    RingBuffer()=default;

    inline bool push(const T& item) {
        uint16_t next = (head_ + 1) & (N-1); // Wraps "next" once it reaches 511. Keeps 1 empty such that head never equals tail
        if (next == tail_)
            return false;   // full

        buffer_[head_] = item;
        head_ = next;
        return true;
    }

    inline bool pop(T& item) {
        if (tail_ == head_)
            return false;   // empty

        item = buffer_[tail_];
        tail_ = (tail_ + 1) & (N - 1);
        return true;
    }

    inline uint16_t size() const {
        return (head_ - tail_) & (N - 1);
    }


private:
    volatile uint16_t head_ = 0;
    volatile uint16_t tail_ = 0;
    T buffer_[N];// Array of T
};

#endif
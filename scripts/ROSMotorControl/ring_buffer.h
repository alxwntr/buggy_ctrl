#ifndef _ring_buffer_h
#define _ring_buffer_h

#include <cstddef>

template<typename member, std::size_t size>
class ring_buffer {
    public:
    ring_buffer() : start(0), valid(0) { }

    void push(member n) {
        if (valid == size) {
            buffer[start]   = n;
            start++;
            if (start == size)
                start       = 0;
        }
        else {
            buffer[valid]   = n;
            valid++;
        }
    }

    void    clear() { start = valid = 0; }
    bool    empty() { return valid == 0; }
    size_t  count() { return valid; }

    // These are undefined if empty() is true or count() == 0
    member  first() { return buffer[start]; }
    member  last()  { return buffer[(start + valid - 1) % size]; }

#ifndef _ring_buffer_TESTING
    private:
#endif
    member          buffer[size];
    std::size_t     start;
    std::size_t     valid;
};

#endif

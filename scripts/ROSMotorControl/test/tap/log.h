#ifndef _tap_log_h
#define _tap_log_h

#include <iostream>
#include <sstream>

namespace tap {
    template<std::ostream *dest>
    class log_ : public std::ostringstream {
        private:
        const char *prefix;

        public:
        log_(const char *pfx = "") : prefix(pfx) { }
        log_(const log_ &other) : prefix(other.prefix) { }

        ~log_() {
            *dest << prefix << str() << std::endl;
        }

        log_& operator=(log_ &&) = default;
    };

    typedef log_<&std::cout> log;

    inline log diag () {
        return log("# ");
    }
}

#endif

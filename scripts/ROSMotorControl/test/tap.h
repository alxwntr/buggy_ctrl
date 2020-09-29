#ifndef _tap_h
#define _tap_h

#include <cstdlib>
#include <iostream>
#include <sstream>

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

static int  _tap_count      = 0;
static int  _tap_failed     = 0; // :)

log diag () {
    return log("# ");
}

void ok (bool test, const char *msg) {
    _tap_count++;
    if (!test)
        _tap_failed++;          // :(

    log() << (test ? "ok " : "not ok ") << _tap_count
        << " - " << msg;
}

void done_testing (int plan) {
    bool success    = true;

    log() << "1.." << plan;

    if (_tap_count != plan) {
        success     = false;
        diag() << "Looks like you planned " << plan
            << " tests but ran " << _tap_count << ".";
    }

    if (_tap_failed) {
        success     = false;
        diag() << "Looks like you failed " << _tap_failed
            << " tests of " << _tap_count << ".";
    }

    if (success)
        std::exit(0);
    else
        std::exit(1);
}

void done_testing () { done_testing(_tap_count); }

template<typename T>
void is (T got, T want, const char *msg) {
    if (want == got) {
        ok(true, msg);
    }
    else {
        ok(false, msg);
        diag() << "Got " << got;
        diag() << "Expected " << want;
    }
}

template<typename T>
inline void is (T got, intmax_t want, const char *msg) {
    is(got, (T)want, msg);
}

#endif

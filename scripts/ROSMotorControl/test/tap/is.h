#ifndef _tap_is_h
#define _tap_is_h

namespace tap {

template<typename T>
void is (T got, T want, std::string msg) {
    if (want == got) {
        ok(true, msg);
    }
    else {
        ok(false, msg);
        diag() << "Got " << pretty(got);
        diag() << "Expected " << pretty(want);
    }
}

template<typename T>
inline void is (T got, intmax_t want, std::string msg) {
    is(got, (T)want, msg);
}

}

#endif

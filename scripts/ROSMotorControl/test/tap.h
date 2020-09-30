#ifndef _tap_h
#define _tap_h

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>

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

typedef std::string str;

str pretty(const int v)                 { return std::to_string(v); }
str pretty(const long v)                { return std::to_string(v); }
str pretty(const unsigned int v)        { return std::to_string(v); }
str pretty(const unsigned long v)       { return std::to_string(v); }
str pretty(const float v)               { return std::to_string(v); }
str pretty(const str v)                 { return v; }

// This is stark staring crazy. We have to prototype this overload here,
// otherwise when we attempt to instantiate pretty(vector<map<>>) the
// compiler can't find an overload to use for the recursive call. But by
// the time the instantiation is happening the complier has already seen
// the relevant prototype. Presumably this means an overload for X won't 
// be properly picked up by pretty(vector<X>). Grrr.
template<typename K, typename V>
std::string pretty(const std::map<K, V> &mp);

template<typename V>
std::string pretty(const std::vector<V> &vc) {
    std::ostringstream s;
    s << "[";
    for (const auto &v: vc) {
        s << pretty(v) << ", ";
    }
    s << "]";
    return s.str();
}

template<typename K, typename V>
std::string pretty(const std::map<K, V> &mp) {
    std::ostringstream s;
    s << '{';
    for (const auto &p: mp) {
        s << pretty(p.first) << ":" << pretty(p.second) << ", ";
    }
    s << '}';
    return s.str();
}

template<typename T>
void is (T got, T want, const char *msg) {
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
inline void is (T got, intmax_t want, const char *msg) {
    is(got, (T)want, msg);
}

#endif

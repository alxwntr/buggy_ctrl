#ifndef _tap_pretty_h
#define _tap_pretty_h

#include <sstream>
#include <string>
#include <vector>
#include <map>

namespace tap {

inline std::string pretty(const int v)             { return std::to_string(v); }
inline std::string pretty(const long v)            { return std::to_string(v); }
inline std::string pretty(const unsigned int v)    { return std::to_string(v); }
inline std::string pretty(const unsigned long v)   { return std::to_string(v); }
inline std::string pretty(const float v)           { return std::to_string(v); }
inline std::string pretty(const std::string v)     { return v; }

// This is stark staring crazy. We have to prototype this overload here,
// otherwise when we attempt to instantiate pretty(vector<map<>>) the
// compiler can't find an overload to use for the recursive call. But by
// the time the instantiation is happening the complier has already seen
// the relevant prototype. Presumably this means an overload for X won't 
// be properly picked up by pretty(vector<X>). Grrr.
template<typename K, typename V>
inline std::string pretty(const std::map<K, V> &mp);

template<typename F, typename S>
inline std::string pretty(const std::pair<F, S> &pr);

template<typename V>
inline std::string pretty(const std::vector<V> &vc) {
    std::ostringstream s;
    s << "[";
    for (const auto &v: vc) {
        s << pretty(v) << ", ";
    }
    s << "]";
    return s.str();
}

template<typename K, typename V>
inline std::string pretty(const std::map<K, V> &mp) {
    std::ostringstream s;
    s << '{';
    for (const auto &p: mp) {
        s << pretty(p.first) << ":" << pretty(p.second) << ", ";
    }
    s << '}';
    return s.str();
}

template<typename F, typename S>
inline std::string pretty(const std::pair<F, S> &pr) {
    std::ostringstream s;
    s << '<' << pretty(pr.first) << ',' << pretty(pr.second) << '>';
    return s.str();
}

}

#endif

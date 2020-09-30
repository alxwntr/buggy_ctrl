#ifndef _tap_mock_h
#define _tap_mock_h

#include <vector>
#include <utility>
#include <string>

namespace tap {

typedef std::vector<std::pair<std::string, std::vector<intmax_t>>>
    mock_results_t;

extern mock_results_t mock_results;

inline void clear_mock_results() { mock_results = {}; }

inline void 
mock_results_are(mock_results_t want, std::string msg) {
    is(mock_results, want, msg);
}

}

#endif

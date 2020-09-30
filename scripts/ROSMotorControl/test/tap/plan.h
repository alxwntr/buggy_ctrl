#ifndef _tap_plan_h
#define _tap_plan_h

#include <cstdlib>

namespace tap {

extern int  test_count;
extern int  test_failed;

inline void ok (bool test, std::string msg) {
    test_count++;
    if (!test)
        test_failed++;

    log() << (test ? "ok " : "not ok ") << test_count
        << " - " << msg;
}

inline void done_testing (int plan) {
    bool success    = true;

    log() << "1.." << plan;

    if (test_count != plan) {
        success     = false;
        diag() << "Looks like you planned " << plan
            << " tests but ran " << test_count << ".";
    }

    if (test_failed) {
        success     = false;
        diag() << "Looks like you failed " << test_failed
            << " tests of " << test_count << ".";
    }

    if (success)
        std::exit(0);
    else
        std::exit(1);
}

inline void done_testing () { done_testing(test_count); }

}

#endif

#ifndef _tap_h
#define _tap_h

// Order is important here
#include "tap/log.h"
#include "tap/pretty.h"
#include "tap/plan.h"
#include "tap/is.h"

namespace tap::exports {
    using tap::log;
    using tap::diag;
    using tap::ok;
    using tap::done_testing;
    using tap::is;
}

#endif

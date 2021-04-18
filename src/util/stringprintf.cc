#include "stringprintf.h"

namespace gopt {
extern std::string StringPrintf(const char* format, ...)
    // Tell the compiler to do printf format string checking.
    GRAPH_OPTIMIZER_PRINTF_ATTRIBUTE(1, 2);
}  // namespace gopt
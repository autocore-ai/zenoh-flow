#ifndef builtin_interfaces_cduration
#define builtin_interfaces_cduration
#include <cstdint>
struct CDuration {
    int32_t sec;
    uint32_t nanosec;
};

#endif
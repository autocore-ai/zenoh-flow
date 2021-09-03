#ifndef builtin_interfaces_ctime
#define builtin_interfaces_ctime
#include <cstdint>
struct CTime {
    int32_t sec;
    uint32_t nanosec;
};

#endif
#ifndef std_msgs_cheader
#define std_msgs_cheader

#include "replay_planner/msgs/builtin_interfaces/CTime.hpp"
#include <string>
#include <cstdint>

struct CHeader {
    CTime stamp;
    const char *frame_id;
    int32_t size;
};

#endif

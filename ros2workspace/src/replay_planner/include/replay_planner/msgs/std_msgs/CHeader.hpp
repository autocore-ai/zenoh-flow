#ifndef std_msgs_cheader
#define std_msgs_cheader

#include "replay_planner/msgs/builtin_interfaces/CTime.hpp"

struct CHeader {
    CTime stamp;
    char *frame_id;
};

#endif

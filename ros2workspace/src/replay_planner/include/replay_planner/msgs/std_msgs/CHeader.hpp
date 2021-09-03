#ifndef std_msgs_cheader
#define std_msgs_cheader

#include "replay_planner/msgs/builtin_interfaces/CTime.hpp"
#include <string>

struct CHeader {
    CTime stamp;
    std::string frame_id;
};

#endif

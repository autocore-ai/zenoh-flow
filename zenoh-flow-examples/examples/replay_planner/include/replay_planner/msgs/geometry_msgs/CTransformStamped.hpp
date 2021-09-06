#ifndef geometry_msgs_ctransformstamped
#define geometry_msgs_ctransformstamped

#include "replay_planner/msgs/std_msgs/CHeader.hpp"
#include "replay_planner/msgs/geometry_msgs/CTransform.hpp"
#include <string>
#include <cstdint>
struct CTransformStamped {
    CHeader header;
    const char *child_frame_id;
    int32_t size;
    CTransform transform;
};

#endif
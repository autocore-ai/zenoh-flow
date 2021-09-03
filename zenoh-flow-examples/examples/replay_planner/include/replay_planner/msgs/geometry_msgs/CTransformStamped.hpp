#ifndef geometry_msgs_ctransformstamped
#define geometry_msgs_ctransformstamped

#include "replay_planner/msgs/std_msgs/CHeader.hpp"
#include "replay_planner/msgs/geometry_msgs/CTransform.hpp"
#include <string>
struct CTransformStamped {
    CHeader header;
    std::string child_frame_id;
    CTransform transform;
};

#endif
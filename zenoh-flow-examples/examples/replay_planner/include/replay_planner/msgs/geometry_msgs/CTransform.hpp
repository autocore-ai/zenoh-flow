#ifndef geometry_msgs_ctransform
#define geometry_msgs_ctransform


#include "replay_planner/msgs/geometry_msgs/CVector3.hpp"
#include "replay_planner/msgs/geometry_msgs/CQuaternion.hpp"

struct CTransform {
    CVector3 translation;
    CQuaternion rotation;
};

#endif
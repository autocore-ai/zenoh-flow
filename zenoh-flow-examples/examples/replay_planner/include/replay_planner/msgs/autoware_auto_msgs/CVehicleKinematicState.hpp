#ifndef autoware_auto_msgs_cvehicle_kinematic_state
#define autoware_auto_msgs_cvehicle_kinematic_state

#include "replay_planner/msgs/std_msgs/CHeader.hpp"
#include "replay_planner/msgs/autoware_auto_msgs/CTrajectoryPoint.hpp"
#include "replay_planner/msgs/geometry_msgs/CTransform.hpp"

struct CVehicleKinematicState
{
    CHeader header;
    CTrajectoryPoint state;
    CTransform delta;
};

#endif

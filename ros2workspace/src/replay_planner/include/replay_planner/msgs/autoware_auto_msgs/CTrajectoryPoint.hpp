#ifndef autoware_auto_msgs_ctrajectory_point
#define autoware_auto_msgs_ctrajectory_point

#include "replay_planner/msgs/builtin_interfaces/CDuration.hpp"
#include "replay_planner/msgs/autoware_auto_msgs/CComplex32.hpp"

struct CTrajectoryPoint
{
    CDuration time_from_start;
    float x;
    float y;
    CComplex32 heading;
    float longitudinal_velocity_mps;
    float lateral_velocity_mps;
    float acceleration_mps2;
    float heading_rate_rps;
    float front_wheel_angle_rad;
    float rear_wheel_angle_rad; 
};

#endif

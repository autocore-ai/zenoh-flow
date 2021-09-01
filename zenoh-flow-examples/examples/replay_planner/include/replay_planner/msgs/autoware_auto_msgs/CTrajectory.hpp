#ifndef autoware_auto_msgs_ctrajectory
#define autoware_auto_msgs_ctrajectory

#include "replay_planner/msgs/std_msgs/CHeader.hpp"
#include "replay_planner/msgs/autoware_auto_msgs/CTrajectoryPoint.hpp"
#include <vector>

struct CTrajectory {
    CHeader header;
    std::vector<CTrajectoryPoint> points;
};

#endif
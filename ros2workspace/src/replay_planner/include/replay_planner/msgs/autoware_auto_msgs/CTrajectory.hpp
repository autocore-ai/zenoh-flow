#ifndef autoware_auto_msgs_ctrajectory
#define autoware_auto_msgs_ctrajectory

#include "replay_planner/msgs/std_msgs/CHeader.hpp"
#include "replay_planner/msgs/autoware_auto_msgs/CTrajectoryPoint.hpp"

struct CArray {
    void *data_ptr;
    size_t size;
};

struct CTrajectory {
    CHeader header;
    CArray *points;
};

#endif
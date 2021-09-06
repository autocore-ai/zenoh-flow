#ifndef REPLAY_PLANNER_WRAPPER
#define REPLAY_PLANNER_WRAPPER

#include "replay_planner/msgs/autoware_auto_msgs/CTrajectory.hpp"
#include "replay_planner/msgs/autoware_auto_msgs/CVehicleKinematicState.hpp"
#include "replay_planner/recordreplay_planner.hpp"

#include <string>
#include <memory>
#include <cstdint>
using motion::planning::recordreplay_planner::RecordReplayPlanner;
class ReplayPlannerNode
{
public:
    ReplayPlannerNode();
    int32_t m_planner_get_record_replay_state();
    bool m_planner_is_replaying();
    void m_planner_start_replaying();
    void m_planner_stop_replaying();

    void m_planner_set_heading_weight(double heading_weight);
    void m_planner_set_min_record_distance(double min_record_distance);

    void m_planner_read_trajectory_buffer_from_file(std::string replay_path);
    CTrajectory m_planner_plan(CVehicleKinematicState current_state);
    bool m_planner_reached_goal(CVehicleKinematicState current_state, double distance_thresh, double angle_thresh);
private:
    std::unique_ptr<RecordReplayPlanner> m_planner_ptr;
    autoware_auto_msgs::msg::Trajectory trajectory;
};

void* getReplayPlannerNode();

#endif // REPLAY_PLANNER_WRAPPER

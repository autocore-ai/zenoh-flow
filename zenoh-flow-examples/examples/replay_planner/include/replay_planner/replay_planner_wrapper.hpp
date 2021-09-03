#ifndef REPLAY_PLANNER_WRAPPER
#define REPLAY_PLANNER_WRAPPER

#include "replay_planner/msgs/autoware_auto_msgs/CTrajectory.hpp"
#include "replay_planner/msgs/autoware_auto_msgs/CVehicleKinematicState.hpp"
#include "replay_planner/recordreplay_planner.hpp"

#include <string>
#include <memory>

using motion::planning::recordreplay_planner::RecordReplayPlanner;

std::unique_ptr<RecordReplayPlanner> m_planner_new();
int m_planner_get_record_replay_state(std::unique_ptr<RecordReplayPlanner> &m_planner_ptr);
bool m_planner_is_replaying(std::unique_ptr<RecordReplayPlanner> &m_planner_ptr);
void m_planner_start_replaying(std::unique_ptr<RecordReplayPlanner> &m_planner_ptr);
void m_planner_stop_replaying(std::unique_ptr<RecordReplayPlanner> &m_planner_ptr);

void m_planner_set_heading_weight(std::unique_ptr<RecordReplayPlanner> &m_planner_ptr, double heading_weight);
void m_planner_set_min_record_distance(std::unique_ptr<RecordReplayPlanner> &m_planner_ptr, double min_record_distance);

void m_planner_read_trajectory_buffer_from_file(std::unique_ptr<RecordReplayPlanner> &m_planner_ptr,std::string replay_path);
CTrajectory m_planner_plan(std::unique_ptr<RecordReplayPlanner> &m_planner_ptr, CVehicleKinematicState &current_state);
bool m_planner_reached_goal(std::unique_ptr<RecordReplayPlanner> &m_planner_ptr, CVehicleKinematicState &current_state, double distance_thresh, double angle_thresh);
#endif // REPLAY_PLANNER_WRAPPER

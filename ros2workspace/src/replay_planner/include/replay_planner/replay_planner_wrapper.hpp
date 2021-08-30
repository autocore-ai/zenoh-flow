#ifndef REPLAY_PLANNER_WRAPPER
#define REPLAY_PLANNER_WRAPPER

#include "replay_planner/msgs/autoware_auto_msgs/CTrajectory.hpp"
#include "replay_planner/msgs/autoware_auto_msgs/CVehicleKinematicState.hpp"

void *m_planner_new();
int m_planner_get_record_replay_state(void *m_planner_ptr);
bool m_planner_is_replaying(void *m_planner_ptr);
void m_planner_start_replaying(void *m_planner_ptr);
void m_planner_stop_replaying(void *m_planner_ptr);

void m_planner_set_heading_weight(void *m_planner_ptr, double heading_weight);
void m_planner_set_min_record_distance(void *m_planner_ptr, double min_record_distance);

void m_planner_read_trajectory_buffer_from_file(void *m_planner_ptr,const char* replay_path);
CTrajectory m_planner_plan(void *m_planner_ptr, CVehicleKinematicState current_state);
bool m_planner_reached_goal(void *m_planner_ptr, CVehicleKinematicState current_state, double distance_thresh, double angle_thresh);
#endif // REPLAY_PLANNER_WRAPPER

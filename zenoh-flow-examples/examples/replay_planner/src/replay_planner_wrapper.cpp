#include "replay_planner/msgs/autoware_auto_msgs/convert.hpp"
#include "replay_planner/replay_planner_wrapper.hpp"

#include <iostream>
#include <memory>

using motion::planning::recordreplay_planner::RecordReplayPlanner;

std::unique_ptr<RecordReplayPlanner> m_planner_new() {
    std::cout << "m_planner_new" << std::endl;
    return std::make_unique<RecordReplayPlanner>();
}
int m_planner_get_record_replay_state(std::unique_ptr<RecordReplayPlanner> &m_planner_ptr) {
    std::cout << "m_planner_get_record_replay_state" << std::endl;
    return m_planner_ptr->get_record_length();
}
bool m_planner_is_replaying(std::unique_ptr<RecordReplayPlanner> &m_planner_ptr) {
    std::cout << "m_planner_is_replaying" << std::endl;
    return m_planner_ptr->is_replaying();
}
void m_planner_start_replaying(std::unique_ptr<RecordReplayPlanner> &m_planner_ptr) {
    std::cout << "m_planner_start_replaying" << std::endl;
    m_planner_ptr->start_replaying();
}
void m_planner_stop_replaying(std::unique_ptr<RecordReplayPlanner> &m_planner_ptr) {
    std::cout << "m_planner_stop_replaying" << std::endl;
    m_planner_ptr->stop_replaying();
}

void m_planner_set_heading_weight(std::unique_ptr<RecordReplayPlanner> &m_planner_ptr, double heading_weight) {
    std::cout << "m_planner_set_heading_weight" << std::endl;
    m_planner_ptr->set_heading_weight(heading_weight);
}
void m_planner_set_min_record_distance(std::unique_ptr<RecordReplayPlanner> &m_planner_ptr, double min_record_distance) {
    std::cout << "m_planner_set_min_record_distance" << std::endl;
    m_planner_ptr->set_min_record_distance(min_record_distance);
}
void m_planner_read_trajectory_buffer_from_file(std::unique_ptr<RecordReplayPlanner> &m_planner_ptr, std::string replay_path) {
    std::cout << "m_planner_read_trajectory_buffer_from_file" << std::endl;
    m_planner_ptr->readTrajectoryBufferFromFile(replay_path);
}

CTrajectory m_planner_plan(std::unique_ptr<RecordReplayPlanner> &m_planner_ptr, CVehicleKinematicState &current_state) {
    std::cout << "m_planner_plan" << std::endl;
    autoware_auto_msgs::msg::VehicleKinematicState msg = convert_cvehiclekinematicstate_to_vehiclekinematicstate(current_state);
    autoware_auto_msgs::msg::Trajectory result_msg = m_planner_ptr->plan(msg);
    CTrajectory ctrajectory = convert_trajectory_to_ctrajectory(result_msg);
    std::cout << "frame_id "<< std::string(ctrajectory.header.frame_id) << std::endl;
    return ctrajectory;
}
bool m_planner_reached_goal(std::unique_ptr<RecordReplayPlanner> &m_planner_ptr, CVehicleKinematicState &current_state, double distance_thresh, double angle_thresh) {
    std::cout << "m_planner_reached_goal" << std::endl;
    autoware_auto_msgs::msg::VehicleKinematicState msg = convert_cvehiclekinematicstate_to_vehiclekinematicstate(current_state);
    return m_planner_ptr->reached_goal(msg, distance_thresh, angle_thresh);
}
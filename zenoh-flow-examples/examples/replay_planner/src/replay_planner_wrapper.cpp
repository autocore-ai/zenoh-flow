#include "replay_planner/msgs/autoware_auto_msgs/convert.hpp"
#include "replay_planner/replay_planner_wrapper.hpp"

#include <iostream>
#include <memory>
#include <cstdint>
using RecordReplayPlanner = motion::planning::recordreplay_planner::RecordReplayPlanner;

ReplayPlannerNode::ReplayPlannerNode() {
    std::cout << "m_planner_new" << std::endl;
    this->m_planner_ptr = std::make_unique<RecordReplayPlanner>();
}

int32_t ReplayPlannerNode::m_planner_get_record_replay_state() {
     std::cout << "m_planner_get_record_replay_state" << std::endl;
     return this->m_planner_ptr->get_record_length();
}

bool ReplayPlannerNode::m_planner_is_replaying() {
    std::cout << "m_planner_is_replaying" << std::endl;
    return this->m_planner_ptr->is_replaying();
}
void ReplayPlannerNode::m_planner_start_replaying() {
    std::cout << "m_planner_start_replaying" << std::endl;
    this->m_planner_ptr->start_replaying();
}
void ReplayPlannerNode::m_planner_stop_replaying() {
    std::cout << "m_planner_stop_replaying" << std::endl;
    this->m_planner_ptr->stop_replaying();
}

void ReplayPlannerNode::m_planner_set_heading_weight(double heading_weight) {
    std::cout << "m_planner_set_heading_weight" << std::endl;
    this->m_planner_ptr->set_heading_weight(heading_weight);
}
void ReplayPlannerNode::m_planner_set_min_record_distance(double min_record_distance) {
    std::cout << "m_planner_set_min_record_distance" << std::endl;
    this->m_planner_ptr->set_min_record_distance(min_record_distance);
}
void ReplayPlannerNode::m_planner_read_trajectory_buffer_from_file(std::string replay_path) {
    std::cout << "m_planner_read_trajectory_buffer_from_file" << std::endl;
    this->m_planner_ptr->readTrajectoryBufferFromFile(replay_path);
}

CTrajectory ReplayPlannerNode::m_planner_plan(CVehicleKinematicState current_state) {
    std::cout << "m_planner_plan" << std::endl;
    autoware_auto_msgs::msg::VehicleKinematicState msg = convert_cvehiclekinematicstate_to_vehiclekinematicstate(current_state);
    this->trajectory = m_planner_ptr->plan(msg);
    CTrajectory ctrajectory = convert_trajectory_to_ctrajectory(this->trajectory);
    std::cout << "frame_id "<< std::string(ctrajectory.header.frame_id) << std::endl;
    return ctrajectory;
}
bool ReplayPlannerNode::m_planner_reached_goal(CVehicleKinematicState current_state, double distance_thresh, double angle_thresh) {
    std::cout << "m_planner_reached_goal" << std::endl;
    autoware_auto_msgs::msg::VehicleKinematicState msg = convert_cvehiclekinematicstate_to_vehiclekinematicstate(current_state);
    return this->m_planner_ptr->reached_goal(msg, distance_thresh, angle_thresh);
}

void* getReplayPlannerNode()
{
    return new ReplayPlannerNode();
}

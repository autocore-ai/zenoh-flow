#ifndef autoware_auto_msgs_convert
#define autoware_auto_msgs_convert
#include <autoware_auto_msgs/msg/complex32.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>

#include "replay_planner/msgs/autoware_auto_msgs/CComplex32.hpp"
#include "replay_planner/msgs/autoware_auto_msgs/CTrajectory.hpp"
#include "replay_planner/msgs/autoware_auto_msgs/CTrajectoryPoint.hpp"
#include "replay_planner/msgs/autoware_auto_msgs/CVehicleKinematicState.hpp"

#include "replay_planner/msgs/std_msgs/convert.hpp"
#include "replay_planner/msgs/geometry_msgs/convert.hpp"
#include "replay_planner/msgs/builtin_interfaces/convert.hpp"

autoware_auto_msgs::msg::Complex32 convert_ccomplex32_to_complex32(CComplex32 &ccomplex32);
CComplex32 convert_complex32_to_ccomplex32(autoware_auto_msgs::msg::Complex32 &complex32);

CTrajectory convert_trajectory_to_ctrajectory(autoware_auto_msgs::msg::Trajectory &trajectory);
autoware_auto_msgs::msg::Trajectory convert_ctrajectory_to_trajectory(CTrajectory &ctrajectory);

CTrajectoryPoint convert_trajectorypoint_to_ctrajectorypoint(autoware_auto_msgs::msg::TrajectoryPoint &trajectorypoint);
autoware_auto_msgs::msg::TrajectoryPoint convert_ctrajectorypoint_to_trajectorypoint(CTrajectoryPoint &ctrajectorypoint);

CVehicleKinematicState convert_vehiclekinematicstate_to_cvehiclekinematicstate(autoware_auto_msgs::msg::VehicleKinematicState &vehiclekinematicstate);
autoware_auto_msgs::msg::VehicleKinematicState convert_cvehiclekinematicstate_to_vehiclekinematicstate(CVehicleKinematicState &cvehiclekinematicstate);

#endif

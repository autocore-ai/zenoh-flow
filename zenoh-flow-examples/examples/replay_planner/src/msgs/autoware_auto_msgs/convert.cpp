#include "replay_planner/msgs/autoware_auto_msgs/convert.hpp"

autoware_auto_msgs::msg::Complex32 convert_ccomplex32_to_complex32(CComplex32 &ccomplex32) {
    autoware_auto_msgs::msg::Complex32 complex32;
    complex32.real = ccomplex32.real;
    complex32.imag = ccomplex32.imag;
    return complex32;
}

CComplex32 convert_complex32_to_ccomplex32(autoware_auto_msgs::msg::Complex32 &complex32) {
    CComplex32 ccomplex32;
    ccomplex32.real = complex32.real;
    ccomplex32.imag = complex32.imag;
    return ccomplex32;
}

CTrajectory convert_trajectory_to_ctrajectory(autoware_auto_msgs::msg::Trajectory &trajectory) {
    CTrajectory ctrajectory;
    ctrajectory.header = convert_header_to_cheader(trajectory.header);
    std::vector<CTrajectoryPoint> vector_ctrajectorypoint;

    for(auto &element : trajectory.points) {
        vector_ctrajectorypoint.push_back(convert_trajectorypoint_to_ctrajectorypoint(element));
    }
    ctrajectory.points = vector_ctrajectorypoint;
    return ctrajectory;
}

autoware_auto_msgs::msg::Trajectory convert_ctrajectory_to_trajectory(CTrajectory &ctrajectory) {
    autoware_auto_msgs::msg::Trajectory trajectory;
    std::vector<CTrajectoryPoint> ctrajectoryPoint = ctrajectory.points;
     rosidl_runtime_cpp::BoundedVector<autoware_auto_msgs::msg::TrajectoryPoint, 100U> boundedvector_trajectorypoint;
    for(auto &element : ctrajectoryPoint) {
        boundedvector_trajectorypoint.push_back(convert_ctrajectorypoint_to_trajectorypoint(element));
    }
    trajectory.header = convert_cheader_to_header(ctrajectory.header);
    trajectory.points = boundedvector_trajectorypoint;
    return trajectory;
}

CTrajectoryPoint convert_trajectorypoint_to_ctrajectorypoint(autoware_auto_msgs::msg::TrajectoryPoint &trajectorypoint) {
    CTrajectoryPoint ctrajectorypoint;
    ctrajectorypoint.time_from_start = convert_duration_to_cduration(trajectorypoint.time_from_start);
    ctrajectorypoint.x = trajectorypoint.x;
    ctrajectorypoint.y = trajectorypoint.y;
    ctrajectorypoint.heading = convert_complex32_to_ccomplex32(trajectorypoint.heading);
    ctrajectorypoint.longitudinal_velocity_mps = trajectorypoint.longitudinal_velocity_mps;
    ctrajectorypoint.lateral_velocity_mps = trajectorypoint.lateral_velocity_mps;
    ctrajectorypoint.acceleration_mps2 = trajectorypoint.acceleration_mps2;
    ctrajectorypoint.heading_rate_rps = trajectorypoint.heading_rate_rps;
    ctrajectorypoint.front_wheel_angle_rad = trajectorypoint.front_wheel_angle_rad;
    ctrajectorypoint.rear_wheel_angle_rad = trajectorypoint.rear_wheel_angle_rad;
    return ctrajectorypoint;
}

autoware_auto_msgs::msg::TrajectoryPoint convert_ctrajectorypoint_to_trajectorypoint(CTrajectoryPoint &ctrajectorypoint) {
    autoware_auto_msgs::msg::TrajectoryPoint trajectorypoint;
    trajectorypoint.time_from_start = convert_cduration_to_duration(ctrajectorypoint.time_from_start);
    trajectorypoint.x = ctrajectorypoint.x;
    trajectorypoint.y = ctrajectorypoint.y;
    trajectorypoint.heading = convert_ccomplex32_to_complex32(ctrajectorypoint.heading);
    trajectorypoint.longitudinal_velocity_mps = ctrajectorypoint.longitudinal_velocity_mps;
    trajectorypoint.lateral_velocity_mps = ctrajectorypoint.lateral_velocity_mps;
    trajectorypoint.acceleration_mps2 = ctrajectorypoint.acceleration_mps2;
    trajectorypoint.heading_rate_rps = ctrajectorypoint.heading_rate_rps;
    trajectorypoint.front_wheel_angle_rad = ctrajectorypoint.front_wheel_angle_rad;
    trajectorypoint.rear_wheel_angle_rad = ctrajectorypoint.rear_wheel_angle_rad;
    return trajectorypoint;
}

CVehicleKinematicState convert_vehiclekinematicstate_to_cvehiclekinematicstate(autoware_auto_msgs::msg::VehicleKinematicState &vehiclekinematicstate) {
    CVehicleKinematicState cvehiclekinematicstate;
    cvehiclekinematicstate.header = convert_header_to_cheader(vehiclekinematicstate.header);
    cvehiclekinematicstate.state = convert_trajectorypoint_to_ctrajectorypoint(vehiclekinematicstate.state);
    cvehiclekinematicstate.delta = convert_transform_to_ctransform(vehiclekinematicstate.delta);
    return cvehiclekinematicstate;
}

autoware_auto_msgs::msg::VehicleKinematicState convert_cvehiclekinematicstate_to_vehiclekinematicstate(CVehicleKinematicState &cvehiclekinematicstate) {
    autoware_auto_msgs::msg::VehicleKinematicState vehiclekinematicstate;
    vehiclekinematicstate.header = convert_cheader_to_header(cvehiclekinematicstate.header);
    vehiclekinematicstate.state = convert_ctrajectorypoint_to_trajectorypoint(cvehiclekinematicstate.state);
    vehiclekinematicstate.delta = convert_ctransform_to_transform(cvehiclekinematicstate.delta);
    return vehiclekinematicstate;
}

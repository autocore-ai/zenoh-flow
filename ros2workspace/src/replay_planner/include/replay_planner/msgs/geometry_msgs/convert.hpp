#ifndef geometry_msgs_convert
#define geometry_msgs_convert

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "replay_planner/msgs/geometry_msgs/CQuaternion.hpp"
#include "replay_planner/msgs/geometry_msgs/CTransform.hpp"
#include "replay_planner/msgs/geometry_msgs/CTransformStamped.hpp"
#include "replay_planner/msgs/geometry_msgs/CVector3.hpp"

CQuaternion convert_quaternion_to_cquaternion(geometry_msgs::msg::Quaternion &quaternion);
geometry_msgs::msg::Quaternion convert_cquaternion_to_quaternion(CQuaternion &cquaternion);

CTransform convert_transform_to_ctransform(geometry_msgs::msg::Transform &transform);
geometry_msgs::msg::Transform convert_ctransform_to_transform(CTransform &ctransform);

CTransformStamped convert_transformstamped_to_ctransformstamped(geometry_msgs::msg::TransformStamped &transformstamped);
geometry_msgs::msg::TransformStamped convert_ctransformstamped_to_transformstamped(CTransformStamped &ctransformstamped);

CVector3 convert_vector3_to_cvector3(geometry_msgs::msg::Vector3 &vector3);
geometry_msgs::msg::Vector3 convert_cvector3_to_vector3(CVector3 &cvector3);

#endif
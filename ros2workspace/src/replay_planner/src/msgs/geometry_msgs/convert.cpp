#include "replay_planner/msgs/geometry_msgs/convert.hpp"
#include "replay_planner/msgs/std_msgs/convert.hpp"

CTransform convert_transform_to_ctransform(geometry_msgs::msg::Transform &transform) {
    CTransform ctransform;
    ctransform.translation = convert_vector3_to_cvector3(transform.translation);
    ctransform.rotation = convert_quaternion_to_cquaternion(transform.rotation);
    return ctransform;
}

geometry_msgs::msg::Transform convert_ctransform_to_transform(CTransform &ctransform) {
    geometry_msgs::msg::Transform transform;
    transform.translation = convert_cvector3_to_vector3(ctransform.translation);
    transform.rotation = convert_cquaternion_to_quaternion(ctransform.rotation);
    return transform;
}


CQuaternion convert_quaternion_to_cquaternion(geometry_msgs::msg::Quaternion &quaternion) {
    CQuaternion cquaternion;
    cquaternion.w = quaternion.w;
    cquaternion.x = quaternion.x;
    cquaternion.y = quaternion.y;
    cquaternion.z = quaternion.z;
    return cquaternion;
}

geometry_msgs::msg::Quaternion convert_cquaternion_to_quaternion(CQuaternion &cquaternion) {
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.w = cquaternion.w;
    quaternion.x = cquaternion.x;
    quaternion.y = cquaternion.y;
    quaternion.z = cquaternion.z;
    return quaternion;
}

CTransformStamped convert_transformstamped_to_ctransformstamped(geometry_msgs::msg::TransformStamped &transformstamped) {
    CTransformStamped ctransformstamped;
    ctransformstamped.header = convert_header_to_cheader(transformstamped.header);
    ctransformstamped.child_frame_id = transformstamped.child_frame_id.c_str();
    ctransformstamped.transform = convert_transform_to_ctransform(transformstamped.transform);
    return ctransformstamped;
}

geometry_msgs::msg::TransformStamped convert_ctransformstamped_to_transformstamped(CTransformStamped &ctransformstamped) {
    geometry_msgs::msg::TransformStamped transformstamped;
    transformstamped.header = convert_cheader_to_header(ctransformstamped.header);
    transformstamped.child_frame_id = std::string(ctransformstamped.child_frame_id);
    transformstamped.transform = convert_ctransform_to_transform(ctransformstamped.transform);
    return transformstamped;
}

CVector3 convert_vector3_to_cvector3(geometry_msgs::msg::Vector3 &vector3) {
    CVector3 cvector3;
    cvector3.x = vector3.x;
    cvector3.y = vector3.y;
    cvector3.z = vector3.z;
    return cvector3;
}

geometry_msgs::msg::Vector3 convert_cvector3_to_vector3(CVector3 &cvector3) {
    geometry_msgs::msg::Vector3 vector3;
    vector3.x = cvector3.x;
    vector3.y = cvector3.y;
    vector3.z = cvector3.z;
    return vector3;
}

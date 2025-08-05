#ifndef __CONVERSION_H__
#define __CONVERSION_H__

#include <vector>
#include <string>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "data_object.h"

namespace ar
{
    double degToRad(double degrees);
    double radToDeg(double radians);
	std::vector<double> degToRad(const std::vector<double>& degrees);
	std::vector<double> radToDeg(const std::vector<double>& rads);
    std::vector<double> quatToEuler(const tf2::Quaternion& quat);
	std::vector<double> quatToEuler(const double x, const double y, const double z, const double w);
	tf2::Matrix3x3 eulerToTfMatrix(const std::vector<double>& euler_angles);
	std::vector<double> tfMatrixToEuler(const tf2::Matrix3x3& rotation_matrix);
	tf2::Matrix3x3 quaternionToTfMatrix(const tf2::Quaternion& quat);
	tf2::Quaternion tfMatrixToQuaternion(const tf2::Matrix3x3& rotation_matrix);
	//Eigen conversion
	Eigen::VectorXd vecToEigenVec(std::vector<double> vec);
	std::vector<double> eigenVecToVec(Eigen::VectorXd eigenVec);
	ArPose eigenIsoToArPose(const Eigen::Isometry3d& eigen_iso);
	//string conversion
	std::string vecToString(const std::vector<double>& vec);
	std::string tfvec3ToString(const tf2::Vector3& vec);
	std::string tfquatToString(const tf2::Quaternion& quat);
	//Pose conversion
	std::vector<double> poseQuatToVec(const tf2::Vector3& pos, const tf2::Quaternion& quat);
	geometry_msgs::msg::Pose vecToPoseMsgs(const std::vector<double>& pose);
	geometry_msgs::msg::Pose poseQuatToPoseMsgs(const tf2::Vector3& pos, const tf2::Quaternion& quat);
	void vecToPoseQuat(const std::vector<double>& vector_input, tf2::Vector3& pos, tf2::Quaternion& quat);
	ArPose poseMsgsToArPose(const geometry_msgs::msg::Pose& pose);
	ArPose poseStampedMsgsToArPose(const geometry_msgs::msg::PoseStamped& pose_stamp);
	geometry_msgs::msg::Pose arPoseToPoseMsgs(const ArPose& ar_pose);
	ArPose tfTransformToArPose(const tf2::Transform& transform);
	tf2::Transform arPoseToTfTransform(const ArPose& pose);
	tf2::Transform poseMsgsToTfTransform(const geometry_msgs::msg::Pose pose);
	geometry_msgs::msg::Pose tfTransStampedToPoseMsgs(const geometry_msgs::msg::TransformStamped& transformStamped);
	tf2::Transform posRotToTfTransform(const tf2::Matrix3x3& rotation, const tf2::Vector3& translation);
	tf2::Transform eigenToTfTransform(const Eigen::Matrix4d& eigen_mat);
	Eigen::Matrix4d transformStampedToMatrix(const geometry_msgs::msg::TransformStamped& transformStamped);
}



#endif // __CONVERSION_H__
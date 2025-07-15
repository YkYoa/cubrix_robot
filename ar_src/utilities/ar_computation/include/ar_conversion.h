#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <vector>

namespace ar_computation {

double degToRad(double deg);
double radToDeg(double rad);
geometry_msgs::msg::Pose vecToPose(const std::vector<double>& vec);
std::vector<double> poseQuatToVec(const geometry_msgs::msg::Pose& pose);
geometry_msgs::msg::PoseStamped poseQuatToPoseMsgs(const geometry_msgs::msg::Pose& pose);
geometry_msgs::msg::Pose vecToPoseQuat(const std::vector<double>& vec);
Eigen::VectorXd vecToEigenVec(const std::vector<double>& vec);
std::vector<double> eigenVecToVec(const Eigen::VectorXd& eigen_vec);

} // namespace ar_computation
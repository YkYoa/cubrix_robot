#include "ar_conversion.h"
#include <cmath>

namespace ar_computation {

double degToRad (double deg) {
    return deg * M_PI / 180.0;
}

double radToDeg (double rad) {
    return rad * 180.0 / M_PI;
}

geometry_msgs::msg::Pose vecToPose (const std::vector<double>& vec) {
    geometry_msgs::msg::Pose pose;
    if (vec.size() >= 7) {
        pose.position.x = vec[0];
        pose.position.y = vec[1];
        pose.position.z = vec[2];
        pose.orientation.x = vec[3];
        pose.orientation.y = vec[4];
        pose.orientation.z = vec[5];
        pose.orientation.w = vec[6];
    }
    return pose;
}

std::vector<double> poseQuatToVec (const geometry_msgs::msg::Pose& pose) {
    std::vector<double> vec(7);
    vec[0] = pose.position.x;
    vec[1] = pose.position.y;
    vec[2] = pose.position.z;
    vec[3] = pose.orientation.x;
    vec[4] = pose.orientation.y;
    vec[5] = pose.orientation.z;
    vec[6] = pose.orientation.w;
    return vec;
}

geometry_msgs::msg::PoseStamped poseQuatToPoseMsgs (const geometry_msgs::msg::Pose& pose) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose = pose;
    return pose_stamped;
}

geometry_msgs::msg::Pose vecToPoseQuat (const std::vector<double>& vec) {
    return vecToPose(vec);
}

Eigen::VectorXd vecToEigenVec (const std::vector<double>& vec) {
    Eigen::VectorXd eigen_vec(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        eigen_vec[i] = vec[i];
    }
    return eigen_vec;
}

std::vector<double> eigenVecToVec (const Eigen::VectorXd& eigen_vec) {
    std::vector<double> vec(eigen_vec.size());
    for (int i = 0; i < eigen_vec.size(); ++i) {
        vec[i] = eigen_vec[i];
    }
    return vec;
}

} // namespace ar_computation
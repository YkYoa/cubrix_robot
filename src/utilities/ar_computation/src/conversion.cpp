#include "conversion.h"
#include <iostream>
#include <stdexcept>


namespace ar
{
    double degToRad(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    double radToDeg(double radians)
    {
        return radians * 180.0 / M_PI;
    }

    std::vector<double> degToRad(const std::vector<double>& degrees)
    {
        std::vector<double> radians(degrees.size());
        for (size_t i = 0; i < degrees.size(); ++i)
        {
            radians[i] = degToRad(degrees[i]);
        }
        return radians;
    }

    std::vector<double> radToDeg(const std::vector<double>& rads)
    {
        std::vector<double> degrees(rads.size());
        for (size_t i = 0; i < rads.size(); ++i)
        {
            degrees[i] = radToDeg(rads[i]);
        }
        return degrees;
    }

    std::string vecToString(const std::vector<double>& vec)
    {
        std::ostringstream ss;
        std::string output;
        for(auto v : vec){
            output += v;
            output += ", ";
        }
        if(!output.empty()){
            output.resize(output.size() - 2); // Remove the last comma and space
        }

        return output;
    }

    std::string tfvec3ToString(const tf2::Vector3& vec)
    {
        std::ostringstream ss;
        ss << "(" << vec.x() << ", " << vec.y() << ", " << vec.z() << ")";
        return ss.str();
    }

    std::string tfquatToString(const tf2::Quaternion& quat)
    {
        std::ostringstream ss;
        ss << "(" << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << ")";
        return ss.str();
    }

    std::vector<double> poseQuatToVec(const tf2::Vector3& pos, const tf2::Quaternion& quat)
    {
        return {pos.x(), pos.y(), pos.z(), quat.x(), quat.y(), quat.z(), quat.w()};
    }

    void vecToPoseQuat(const std::vector<double>& vector_input, tf2::Vector3& pos, tf2::Quaternion& quat)
    {
        if(vector_input.size() != 7){
            throw std::invalid_argument("Input vector must have 7 elements.");
            return;
        }
        pos[0] = vector_input[0];
        pos[1] = vector_input[1];
        pos[2] = vector_input[2];
        quat[0] = vector_input[3];
        quat[1] = vector_input[4];
        quat[2] = vector_input[5];
        quat[3] = vector_input[6];
    }
    
    geometry_msgs::msg::Pose vecToPoseMsgs(const std::vector<double>& pose)
    {
        geometry_msgs::msg::Pose pose_msg;
        if(pose.size() != 7){
            throw std::invalid_argument("Pose vector must have 7 elements.");
        }

        pose_msg.position.x = pose[0];
        pose_msg.position.y = pose[1];
        pose_msg.position.z = pose[2];
        pose_msg.orientation.x = pose[3];
        pose_msg.orientation.y = pose[4];
        pose_msg.orientation.z = pose[5];
        pose_msg.orientation.w = pose[6];

        return pose_msg;
    }

    Eigen::VectorXd vecToEigenVec(std::vector<double> vec)
    {
        return Eigen::Map<Eigen::VectorXd>(vec.data(), vec.size());    // share memory with std::vector  
    }

    std::vector<double> eigenVecToVec(Eigen::VectorXd eigenVec)
    {
        return std::vector<double>(eigenVec.data(), eigenVec.data() + eigenVec.size());
    }

    ArPose tfTransformToArPose(const tf2::Transform& transform)
    {
        return ArPose(transform.getOrigin(), transform.getRotation());
    }
    
    tf2::Transform arPoseToTfTransform(const ArPose& pose)
    {
        return tf2::Transform(pose.quat, pose.pos);
    }

    

} // namespace ar
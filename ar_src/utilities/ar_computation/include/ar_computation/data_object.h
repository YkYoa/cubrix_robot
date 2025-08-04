#ifndef __DATA_OBJECT_H__
#define __DATA_OBJECT_H__

#include <string>
#include <vector>
#include <memory>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace ar
{
    struct JointBase
    {
        std::vector<double> joints;
        std::string joint_name = "";
        std::string joint_planning_group = "";
        std::string toString() const;
        void reset();
        void setData(const std::vector<double> joint_values, const bool isRadian = true, const std::string& planningGroup = "", 
                     const std::string& name = "");
    };

    struct ArJoint final : public JointBase
    {
        ArJoint();
        ArJoint(const std::vector<double>& joint_values, const bool isRadian = true, 
                const std::string& planningGroup = "", const std::string& name = "");
        void offset(const std::vector<double>& double);
    };

    struct PoseBase
    {
        tf2::Vector3 pos;
        tf2::Quaternion quat;
        std::string pose_name = "";
        std::string pose_planning_group = "";
        std::string toString() const;
        std::vector<double> toVector() const;
        geometry_msgs::msg::Pose toPoseMsgs() const;
        geometry_msgs::msg::PoseStamped toPoseStampedMsgs() const;
        void reset();
        void setData(std::vector<double> pose_values, const std::string& planningGroup = "", const std::string& name = "");
    }

    struct ArPose final : public PoseBase
    {
        ArPose();
        ArPose(const tf2::Vector3& pos, tf2::Quaternion& quat);
        ArPose(double x = 0; double y = 0; double z = 0; 
               double qx = 0; double qy = 0; double qz = 0, double qw = 1);
        
        tf2::Matrix3x3 getRotationMatrix() const;
        tf2::Transform getTransform() const;
        std::vector<double> getRPY() const;

        void offsetPos(const tf2::Vector3& offset);
        void offsetQuat(const tf2::Quaternion& offset);
        void offsetXYZ(const double x_offset, const double y_offset, const double z_offset);
        void offsetRPY(const double roll_offset, const double pitch_offset, const double yaw_offset);
    };

    using KinematicObject = std::variant<ArJoint, ArPose>;
    using KinematicObjects = std::vector<KinematicObject>;
}




#endif // __DATA_OBJECT_H__
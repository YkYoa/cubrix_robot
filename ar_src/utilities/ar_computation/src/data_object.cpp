#include "data_object.h"
#include "conversion.h"


namespace ar
{
    void JointBase::setData(const std::vector<double> joint_values, const bool isRadian, 
                                    const std::string& planningGroup, const std::string& name)
    {
        joints = isRadian ? joint_values : degToRad(joint_values);
        joint_name = name;
        joint_planning_group = planningGroup;
    }

    void JointBase::reset()
    {
        joint_name.clear();
        joint_planning_group.clear();
        std::fill(joints.begin(), joints.end(), 0.0);    // Reset all joint values to zero keep the joint size. don't clear the vector
    }

    std::string JointBase::toString() const
    {
        std::string output = "joint_name: " + joint_name + "planning_group: " + joint_planning_group + "joint_values: ";
        output += vecToString(joints);
        return output;
    }

    void ArJoint::offset(const std::vector<double>& offset_values)
    {
        if(offset_values.size() != joints.size()){
            throw std::invalid_argument("Offset size does not match joint size.");
            return;
        }
        joints = eigenVecToVec(vecToEigenVec(joints) + vecToEigenVec(offset_values));
    }

    ArJoint::ArJoint()
    {
        reset();
    }

    ArJoint::ArJoint(const std::vector<double>& joint_values, const bool isRadian, 
                     const std::string& planningGroup, const std::string& name)
    {
        setData(joint_values, isRadian, planningGroup, name);
    }

    std::string PoseBase::toString() const
    {
        std::string output = "pose_name: " + pose_name + "planning_group: " + pose_planning_group + "position: ";
        output += tfvec3ToString(pos) + " orientation: " + tfquatToString(quat);
        return output;
    }

    std::vector<double> PoseBase::toVector() const
    {
        return poseQuatToVec(pos, quat);   
    }

    geometry_msgs::msg::Pose PoseBase::toPoseMsgs() const
    {
        return poseQuatToPoseMsgs(pos, quat);
    }

    geometry_msgs::msg::PoseStamped PoseBase::toPoseStampedMsgs() const
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.pose = toPoseMsgs();
        return pose_stamped;
    }

    void PoseBase::reset()
    {
        pos = tf2::Vector3(0, 0, 0);
        quat = tf2::Quaternion(0, 0, 0, 1);
        pose_name.clear();
        pose_planning_group.clear();
    }
    
    void PoseBase::setData(std::vector<double> pose_values, const std::string& planningGroup, const std::string& name)
    {
        pose_planning_group = planningGroup;
        pose_name = name;
        vecToPoseQuat(pose_values, pos, quat);
    }

    ArPose::ArPose()
    {
        reset();
    }

    ArPose::ArPose(const tf2::Vector3& position,const tf2::Quaternion& quaternion)
    {
        reset();
        pos = position;
        quat = quaternion;
    }

    ArPose::ArPose(double x, double y, double z, 
                   double qx, double qy, double qz, double qw)
    {
        setData({x, y, z, qx, qy, qz, qw});
    }

    tf2::Matrix3x3 ArPose::getRotationMatrix() const
    {
        return tf2::Matrix3x3(quat);
    }

    tf2::Transform ArPose::getTransform() const
    {
        return tf2::Transform(quat, pos);
    }

    std::vector<double> ArPose::getRPY() const
    {
        double roll, pitch, yaw;
        getRotationMatrix().getRPY(roll, pitch, yaw);
        return {roll, pitch, yaw};
    }

    void ArPose::offsetPos(const tf2::Vector3& offset)
    {
        pos += offset;
    }

    void ArPose::offsetQuat(const tf2::Quaternion& offset)
    {
        quat = quat * offset;
        quat.normalize();
    }

    void ArPose::offsetXYZ(const double x_offset, const double y_offset, const double z_offset)
    {
        tf2::Vector3 offset(x_offset, y_offset, z_offset);
        offsetPos(offset);
    }

    void ArPose::offsetRPY(const double roll_offset, const double pitch_offset, const double yaw_offset)
    {
        tf2::Quaternion offset_quat;
        offset_quat.setRPY(roll_offset, pitch_offset, yaw_offset);
        offsetQuat(offset_quat);
    }

} // namespace ar
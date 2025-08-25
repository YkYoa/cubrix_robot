#include "computation.h"
#include "conversion.h"
#include "data_object.h"


namespace ar
{
    ArPose multiplyTransformToArPose(const tf2::Transform& tf1, const tf2::Transform& tf2)
    {
        tf2::Transform result = tf1 * tf2;
        return tfTransformToArPose(result);
    }

    ArPose getAbsolutePose(const ArPose& offset_pose, const ArPose& base_pose)
    {
        return multiplyTransformToArPose(arPoseToTfTransform(base_pose), arPoseToTfTransform(offset_pose));
    }

    ArPose getAbsolutePose(const ArPose& offset_pose, const geometry_msgs::msg::Pose& base_pose)
    {
        return getAbsolutePose(offset_pose, poseMsgsToArPose(base_pose));
    }

    ArPose getAbsolutePose(const double offset_x, const double offset_y, const double offset_z, const ArPose& base_pose)
    {
        ArPose base_pose_output = base_pose;
        base_pose_output.offsetPos(tf2::Vector3(offset_x, offset_y, offset_z));
        return base_pose_output;
    }

    ArPose getAbsolutePose(const double offset_x, const double offset_y, const double offset_z, geometry_msgs::msg::Pose& base_pose)
    {
        ArPose base_pose_output = poseMsgsToArPose(base_pose);
        return getAbsolutePose(offset_x, offset_y, offset_z, base_pose_output);
    }

    tf2::Vector3 getAbsolutePose(const tf2::Vector3& vector, const double distance, const ArPose& base_pose)
    {
        tf2::Transform origin(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, 0));
        return origin.inverse() * arPoseToTfTransform(base_pose) * (distance * vector.normalized());
    }

    ArPose getOffsetPose(const ArPose& poselink1tolink2, const ArPose& poselink2tolink3)
    {
        return multiplyTransformToArPose(arPoseToTfTransform(poselink1tolink2), arPoseToTfTransform(poselink2tolink3));
    }

    tf2::Vector3 movePoseAlongVector(const tf2::Vector3& vector, const double distance, const tf2::Vector3& base_pose)
    {
        return base_pose * distance * vector.normalized();
    }

    tf2::Vector3 movePoseAlongVector(const tf2::Vector3& pose_begin, const tf2::Vector3& pose_end, const double distance,
                                     const tf2::Vector3& base_pose)
    {
        tf2::Vector3 direction_vector = pose_end - pose_begin;
        direction_vector.normalize();
        return base_pose + direction_vector * distance;
    }

    ArPose movePoseAlongVector(const ArPose& vector, const double distance, const ArPose& base_pose)
    {
        ArPose new_position = base_pose;
        new_position.pos = movePoseAlongVector(vector.pos, distance, base_pose.pos);
        return new_position;
    }    

    tf2::Transform getTransformBetweenPoses(const ArPose& start_pose, const ArPose& end_pose)
    {
        tf2::Transform start_tf = arPoseToTfTransform(start_pose);
        tf2::Transform end_tf = arPoseToTfTransform(end_pose);
        return start_tf.inverse() * end_tf;
    }

    PlaneParameter getPosePlaneParameter(const ArPose& pose, const Plane plane)
	{
		tf2::Vector3 pos;
		ArPose plane_pose;
		PlaneParameter plane_param;
		// Set plane pose
		if(plane == PLANE_YZ) {
			pos = tf2::Vector3(1, 0, 0);
		}
		else if(plane == PLANE_ZX) {
			pos = tf2::Vector3(0, 1, 0);
		}
		else if(plane == PLANE_XY) {
			pos = tf2::Vector3(0, 0, 1);
		}
		plane_pose.pos	= pos;
		plane_pose.quat = tf2::Quaternion(0, 0, 0, 1);
		// trasform the plane relative to pose
		ArPose Transformed_plane = getAbsolutePose(plane_pose, pose);
		plane_param.origin		   = pose.pos;
		plane_param.normal_vector  = Transformed_plane.pos - pose.pos;
		return plane_param;
	}


} // namespace ar
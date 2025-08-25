#ifndef __COMPUTATION_H__
#define __COMPUTATION_H__

#include "data_object.h"
#include "conversion.h"

namespace ar
{
    typedef enum
	{
		PLANE_XY,
		PLANE_YZ,
		PLANE_ZX
	} Plane;

	typedef enum
	{
		AXIS_X,
		AXIS_Y,
		AXIS_Z
	} Axis;

	typedef struct
	{
		// origin of the plane in global frame
		tf2::Vector3 origin;
		tf2::Vector3 normal_vector;
	} PlaneParameter;

    ArPose multiplyTransformToArPose(const tf2::Transform& tf1, const tf2::Transform& tf2);
    ArPose getAbsolutePose(const ArPose& offset_pose, const ArPose& base_pose);
	ArPose getAbsolutePose(const ArPose& offset_pose, const geometry_msgs::msg::Pose& base_pose);
	ArPose getAbsolutePose(const double offset_x, const double offset_y, const double offset_z, const ArPose& base_pose);
	ArPose getAbsolutePose(const double offset_x, const double offset_y, const double offset_z, geometry_msgs::msg::Pose& base_pose);
	tf2::Vector3 getAbsolutePose(const tf2::Vector3& vector, const double distance, const ArPose& base_pose);
	ArPose getOffsetPose(const ArPose& poselink1tolink2, const ArPose& poselink2tolink3);
	tf2::Vector3 movePoseAlongVector(const tf2::Vector3& vector, const double distance, const tf2::Vector3& base_pose);
	tf2::Vector3 movePoseAlongVector(const tf2::Vector3& pose_begin, const tf2::Vector3& pose_end, const double distance,
									 const tf2::Vector3& base_pose);
	ArPose movePoseAlongVector(const ArPose& vector, const double distance, const ArPose& base_pose);
	tf2::Transform getTransformBetweenPoses(const ArPose& start_pose, const ArPose& end_pose);

	PlaneParameter getPosePlaneParameter(const ArPose& pose, const Plane plane);
}



#endif // __COMPUTATION_H__
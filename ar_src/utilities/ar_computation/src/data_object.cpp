#include "ar_computation/data_object.h"


namespace ar
{
    void JointBase::setData(const std::vector<double> joint_values, const bool isRadian, 
                                    const std::string& planningGroup, const std::string& name)
    {
        joints = isRadian ? joint_values : degToRad(joint_values);
        joint_name = name;
        joint_planning_group = planningGroup;
    }



} // namespace ar
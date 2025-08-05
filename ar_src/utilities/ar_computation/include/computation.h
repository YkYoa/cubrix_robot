#ifndef __COMPUTATION_H__
#define __COMPUTATION_H__

#include "data_object.h"
#include "conversion.h"

namespace ar
{
    ArPose multiplyTransformToArPose(const tf2::Transform& tf1, const tf2::Transform& tf2);
    
}



#endif // __COMPUTATION_H__
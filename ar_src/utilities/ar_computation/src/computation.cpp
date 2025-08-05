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


} // namespace ar
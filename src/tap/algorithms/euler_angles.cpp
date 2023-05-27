#include "euler_angles.hpp"


namespace tap::algorithms
{

CMSISMat<3, 3> fromEulerAngles(const float roll, const float pitch, const float yaw)
{
    return CMSISMat<3, 3>({
        cosf(yaw) * cosf(pitch),
        (cosf(yaw) * sinf(pitch) * sinf(roll)) - (sinf(yaw) * cosf(roll)),
        (cosf(yaw) * sinf(pitch) * cosf(roll)) + sinf(yaw) * sinf(roll),
        sinf(yaw) * cosf(pitch),
        sinf(yaw) * sinf(pitch) * sinf(roll) + cosf(yaw) * cosf(roll),
        sinf(yaw) * sinf(pitch) * cosf(roll) - cosf(yaw) * sinf(roll),
        -sinf(pitch),
        cosf(pitch) * sinf(roll),
        cosf(pitch) * cosf(roll)});
}

}
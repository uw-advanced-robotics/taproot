#ifndef TAPROOT_EULER_ANGLES_HPP_
#define TAPROOT_EULER_ANGLES_HPP_

#include "cmsis_mat.hpp"

namespace tap::algorithms
{

CMSISMat<3, 3> fromEulerAngles(const float roll, const float pitch, const float yaw);

}

#endif
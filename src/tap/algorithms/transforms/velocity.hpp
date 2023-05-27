#ifndef TAPROOT_VELOCITY_HPP_
#define TAPROOT_VELOCITY_HPP_

#include "vector.hpp"

namespace tap::algorithms::transforms
{

template <Frame FRAME>
class Velocity : public Vector<FRAME>  // TODO: don't do this
{
public:
    Velocity(const float x, const float y, const float z)
        : Vector<FRAME>({x, y, z})
    {
    }

    Velocity(Vector<FRAME> vector)
        : Vector<FRAME>(std::move(vector))
    {
    }

    Velocity(CMSISMat<3, 1> coordinates)
        : Vector<FRAME>(std::move(coordinates))
    {
    }

};  // class Velocity

}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_VELOCITY_HPP_
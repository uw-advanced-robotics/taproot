#include "inertial_transform.hpp"
#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/algorithms/cross_product.hpp"

using namespace tap::algorithms;

namespace tap::algorithms::transforms
{

template <Frame SOURCE, Frame TARGET>
Velocity<TARGET> InertialTransform<SOURCE, TARGET>::apply(Position<SOURCE> position, Velocity<SOURCE> velocity) const
{
    return Velocity<TARGET>(velocity.coordinates - transVel - cross(angVel, position));
}

template <Frame SOURCE, Frame TARGET>
InertialTransform<TARGET, SOURCE> InertialTransform<SOURCE, TARGET>::getInverse() const
{
    return InertialTransform<TARGET, SOURCE>(Transform<SOURCE, TARGET>::getInverse(), -transVel, -angVel);
}

template <Frame SOURCE, Frame TARGET>
template <Frame NEW>
InertialTransform<SOURCE, NEW> InertialTransform<SOURCE, TARGET>::compose(const InertialTransform<TARGET, NEW>& second) const
{
    CMSISMat<3, 1> transVel = this->transVel + this->rotation * second.transVel + cross(this->angVel, second.translation);
    CMSISMat<3, 1> angVel = this->transVel + second.transVel;
    return InertialTransform(Transform<SOURCE, TARGET>::compose(second), transVel, angVel);
}

}
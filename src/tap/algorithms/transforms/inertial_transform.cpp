#include "inertial_transform.hpp"
#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/algorithms/cross_product.hpp"

using namespace tap::algorithms;

namespace tap::algorithms::transforms
{

template <typename SOURCE, typename TARGET>
Velocity<TARGET> InertialTransform<SOURCE, TARGET>::apply(Position<SOURCE> position, Velocity<SOURCE> velocity) const
{
    return Velocity<TARGET>(velocity.coordinates - transVel - cross(angVel, position));
}

template <typename SOURCE, typename TARGET>
InertialTransform<TARGET, SOURCE> InertialTransform<SOURCE, TARGET>::getInverse() const
{
    return InertialTransform<TARGET, SOURCE>(Transform<SOURCE, TARGET>::getInverse(), -transVel, -angVel);
}

template <typename A, typename B, typename C>
InertialTransform<A, C> compose(const InertialTransform<A, B> first, const InertialTransform<B, C> second)
{
    CMSISMat<3, 1> transVel = first.transVel + first.rotation * second.transVel + cross(first.angVel, second.translation);
    CMSISMat<3, 1> angVel = first.transVel + second.transVel;
    return InertialTransform(compose());
}

}
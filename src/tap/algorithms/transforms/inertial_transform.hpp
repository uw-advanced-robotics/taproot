#include "transform.hpp"
#include "velocity.hpp"
#include "position.hpp"
#include "tap/algorithms/cmsis_mat.hpp"
#include "frame.hpp"


#ifndef INERTIAL_TRANSFORM_HPP_
#define INERTIAL_TRANSFORM_HPP_

namespace tap::algorithms::transforms
{

// TODO: somewhat inaccurate name since rotational frames are not inertial
template <Frame SOURCE, Frame TARGET>
class InertialTransform : protected Transform<SOURCE, TARGET>
{
public:
    InertialTransform(const Transform<SOURCE, TARGET> transform, const float xVel, const float yVel, const float zVel, const float rollVel, const float pitchVel, const float yawVel);
    InertialTransform(const Transform<SOURCE, TARGET> transform, CMSISMat<3, 1>& transVel, CMSISMat<3, 1>& angVel);
    inline InertialTransform(const float x, const float y, const float z, const float roll, const float pitch, const float yaw, const float xVel, const float yVel, const float zVel, const float rollVel, const float pitchVel, const float yawVel)
        : Transform<SOURCE, TARGET>(x, y, z, roll, pitch, yaw)
        , transVel({xVel, yVel, zVel})
        , angVel({roll, pitch, yaw})
    {}
    using Transform<SOURCE, TARGET>::apply;
    using Transform<SOURCE, TARGET>::getTranslation;
    Velocity<TARGET> apply(Position<SOURCE> position, Velocity<SOURCE> velocity) const;
    InertialTransform<TARGET, SOURCE> getInverse() const;

    template <Frame NEW>
    InertialTransform<SOURCE, NEW> compose(const InertialTransform<TARGET, NEW>& second) const;

private:
    /**
     * Translation differential vector.
     * 
     * The velocity of the target frame origin in the source frame.
     */
    CMSISMat<3, 1> transVel;

    /**
     * Differential of rotation matrix.
     * 
     * The angular velocity of the target frame coordinates in the source frame.
     */
    CMSISMat<3, 1> angVel;
};

}

#endif
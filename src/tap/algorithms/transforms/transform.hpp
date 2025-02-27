/*
 * Copyright (c) 2022-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TAPROOT_TRANSFORM_HPP_
#define TAPROOT_TRANSFORM_HPP_

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/algorithms/math_user_utils.hpp"

#include "orientation.hpp"
#include "position.hpp"
#include "vector.hpp"

namespace tap::algorithms::transforms
{

/**
 Represents a transformation from one coordinate frame to another.

    A Transform from frame A to frame B defines a relationship between the two frames, such
 that a spatial measurement in frame A can be represented equivalently in frame B by applying a
    translational and rotational offset. This process is known as *applying* a transform.

    Transforms are specified as a translation and rotation of some "target" frame relative to
 some "source" frame. The "translation" is the target frame's origin in source frame, and the
    "rotation" is the target frame's orientation relative to the source frame's orientation.

    Conceptually, translations are applied "before" rotations. This means that the origin of the
    target frame is entirely defined by the translation in the source frame, and the rotation serves
    only to change the orientation of the target frame's axes relative to the source frame.

    Utilizes arm's CMSIS matrix operations.

    @param SOURCE represents the source frame of the transformation.
    @param TARGET represents the target frame of the transformation.
 */
class Transform
{
public:
    /**
     * @param rotation Initial rotation of this transformation.
     * @param position Initial translation of this transformation.
     */
    Transform(const Position& translation, const Orientation& rotation);
    Transform(Position&& translation, Orientation&& rotation);

    /**
     * @param rotation Initial rotation of this transformation.
     * @param position Initial translation of this transformation.
     */
    Transform(const CMSISMat<3, 1>& translation, const CMSISMat<3, 3>& rotation);
    Transform(CMSISMat<3, 1>&& translation, CMSISMat<3, 3>&& rotation);

    /**
     * Constructs rotations using XYZ Euler angles,
     * so rotations are applied in order of A, B, then C.
     * As an example, for an x-forward, z-up coordinate system,
     * this is in the order of roll, pitch, then yaw.
     *
     * @param x: Initial x-component of the translation.
     * @param y: Initial y-component of the translation.
     * @param z: Initial z-component of the translation.
     * @param roll: Initial rotation angle about the x-axis.
     * @param pitch: Initial rotation angle about the y-axis.
     * @param yaw: Initial rotation angle about the z-axis.
     */
    Transform(float x, float y, float z, float roll, float pitch, float yaw);

    /**
     * @param translation Initial translation of this transformation.
     * @param rotation Initial rotation of this transformation.
     * @param velocity Translational velocity of this transformation.
     * @param acceleration Translational acceleration of this transformation.
     * @param angularVelocity Angular velocity pseudovector of this transformation.
     */
    Transform(
        const Position& translation,
        const Orientation& rotation,
        const Vector& velocity,
        const Vector& acceleration,
        const Vector& angularVelocity);

    /**
     * @param translation Initial translation of this transformation.
     * @param rotation Initial rotation of this transformation.
     * @param velocity Translational velocity of this transformation.
     * @param acceleration Translational acceleration of this transformation.
     * @param angularVelocity Angular velocity pseudovector of this transformation.
     */
    Transform(
        Position&& translation,
        Orientation&& rotation,
        Vector&& velocity,
        Vector&& acceleration,
        Vector&& angularVelocity);

    /**
     * @param translation Initial translation of this transformation.
     * @param rotation Initial rotation of this transformation.
     * @param velocity Translational velocity of this transformation.
     * @param acceleration Translational acceleration of this transformation.
     * @param angularVelocity Angular velocity skew symmetric matrix of this transformation.
     */
    Transform(
        const CMSISMat<3, 1>& translation,
        const CMSISMat<3, 3>& rotation,
        const CMSISMat<3, 1>& velocity,
        const CMSISMat<3, 1>& acceleration,
        const CMSISMat<3, 3>& angularVelocity);

    /**
     * @param translation Initial translation of this transformation.
     * @param rotation Initial rotation of this transformation.
     * @param velocity Translational velocity of this transformation.
     * @param acceleration Translational acceleration of this transformation.
     * @param angularVelocity Angular velocity skew symmetric matrix of this transformation.
     */
    Transform(
        CMSISMat<3, 1>&& translation,
        CMSISMat<3, 3>&& rotation,
        CMSISMat<3, 1>&& velocity,
        CMSISMat<3, 1>&& acceleration,
        CMSISMat<3, 3>&& angularVelocity);

    /**
     * Constructs rotations using XYZ Euler angles,
     * so rotations are applied in order of A, B, then C.
     * As an example, for an x-forward, z-up coordinate system,
     * this is in the order of roll, pitch, then yaw.
     *
     * @param x: Initial x-component of the translation.
     * @param y: Initial y-component of the translation.
     * @param z: Initial z-component of the translation.
     * @param A: Initial rotation angle about the x-axis.
     * @param B: Initial rotation angle about the y-axis.
     * @param C: Initial rotation angle about the z-axis.
     */
    Transform(
        float x,
        float y,
        float z,
        float vx,
        float vy,
        float vz,
        float ax,
        float ay,
        float az,
        float roll,
        float pitch,
        float yaw,
        float rollVel,
        float pitchVel,
        float yawVel);

    /**
     * Constructs an identity static transform.
     */
    static inline Transform identity() { return Transform(0., 0., 0., 0., 0., 0.); }

    /**
     * Apply this transform to a position.
     *
     * @param[in] position Position in source frame.
     * @return Position in target frame.
     */
    Position apply(const Position& position) const;

    /**
     * Rotates a vector in the source frame to a vector in the target frame.
     *
     * Intended to be used for things like velocities and accelerations which represent the
     * difference between two positions in space, since both positions get translated the same way,
     * causing the translation to cancel out.
     *
     * @param vector Vector as read by source frame.
     * @return Vector in target frame's basis.
     */
    Vector apply(const Vector& vector) const;

    /**
     *
     */
    Orientation apply(const Orientation& orientation) const;

    /**
     * Updates the translation of the current transformation matrix.
     *
     * @param newTranslation updated position of target in source frame.
     */
    inline void updateTranslation(const Position& newTranslation)
    {
        this->translation = newTranslation.coordinates();
    }

    /**
     * Updates the translation of the current transformation matrix.
     *
     * @param newTranslation updated position of target in source frame.
     */
    inline void updateTranslation(Position&& newTranslation)
    {
        this->translation = std::move(newTranslation.coordinates());
    }

    /**
     * Updates the translation of the current transformation matrix.
     *
     * @param x new translation x-component.
     * @param y new translation y-component.
     * @param z new translation z-component.
     */
    inline void updateTranslation(float x, float y, float z)
    {
        this->translation = CMSISMat<3, 1>({x, y, z});
    }

    /**
     * Updates the rotation of the current transformation matrix.
     *
     * @param newRotation updated orientation of target frame in source frame.
     */
    inline void updateRotation(const Orientation& newRotation)
    {
        this->rotation = newRotation.matrix();
        this->tRotation = this->rotation.transpose();
    }

    /**
     * Updates the rotation of the current transformation matrix.
     *
     * @param newRotation updated orientation of target frame in source frame.
     */
    inline void updateRotation(Orientation&& newRotation)
    {
        this->rotation = std::move(newRotation.matrix());
        this->tRotation = this->rotation.transpose();
    }

    /**
     * Updates the rotation of the current transformation matrix.
     * Takes rotation angles in the order of roll->pitch->yaw.
     *
     * @param roll updated rotation angle about the x-axis.
     * @param pitch updated rotation angle about the y-axis.
     * @param yaw updated rotation angle about the z-axis.
     */
    void updateRotation(float roll, float pitch, float yaw)
    {
        this->rotation = Orientation(roll, pitch, yaw).matrix();
        this->tRotation = this->rotation.transpose();
    }

    /**
     * Updates the velocity of the current transform.
     *
     * @param newVelocity updated velocity of target in source frame.
     */
    inline void updateVelocity(const Vector& newVelocity)
    {
        this->transVel = newVelocity.coordinates();
        checkDynamic();
    }

    /**
     * Updates the velocity of the current transform.
     *
     * @param newVelocity updated velocity of target in source frame.
     */
    inline void updateVelocity(Vector&& newVelocity)
    {
        this->transVel = std::move(newVelocity.coordinates());
        checkDynamic();
    }

    /**
     * Updates the velocity of the current transform.
     *
     * @param vx new velocity x-component.
     * @param vy new velocity y-component.
     * @param vz new velocity z-component.
     */
    inline void updateVelocity(float vx, float vy, float vz)
    {
        this->transVel = CMSISMat<3, 1>({vx, vy, vz});
        checkDynamic();
    }

    /**
     * Updates the acceleration of the current transform.
     *
     * @param updateAcceleration updated acceleration of target in source frame.
     */
    inline void updateAcceleration(const Vector& newAcceleration)
    {
        this->transVel = newAcceleration.coordinates();
        checkDynamic();
    }

    /**
     * Updates the acceleration of the current transform.
     *
     * @param updateAcceleration updated acceleration of target in source frame.
     */
    inline void updateAcceleration(Vector&& newAcceleration)
    {
        this->transVel = std::move(newAcceleration.coordinates());
        checkDynamic();
    }

    /**
     * Updates the acceleration of the current transform.
     *
     * @param ax new acceleration x-component.
     * @param ay new acceleration y-component.
     * @param az new acceleration z-component.
     */
    inline void updateAcceleration(float ax, float ay, float az)
    {
        this->transAcc = CMSISMat<3, 1>({ax, ay, az});
        checkDynamic();
    }

    /**
     * Updates the angular velocity of the current transform.
     *
     * @param updateAngularVelocity updated angular velocity of target in source frame.
     */
    inline void updateAngularVelocity(const Vector& newAngularVelocity)
    {
        this->angVel = skewMatFromAngVel(
            newAngularVelocity.x(),
            newAngularVelocity.y(),
            newAngularVelocity.z());
        checkDynamic();
    }

    /**
     * Updates the angular velocity of the current transform.
     *
     * @param updateAngularVelocity updated angular velocity of target in source frame.
     */
    inline void updateAngularVelocity(Position&& newAngularVelocity)
    {
        this->angVel = skewMatFromAngVel(
            newAngularVelocity.x(),
            newAngularVelocity.y(),
            newAngularVelocity.z());
        checkDynamic();
    }

    /**
     * Updates the angular velocity of the current transform.
     *
     * @param ax new angular velocity x-component.
     * @param ay new angular velocity y-component.
     * @param az new angular velocity z-component.
     */
    inline void updateAngularVelocity(float vr, float vp, float vy)
    {
        this->angVel = skewMatFromAngVel(vr, vp, vy);
        checkDynamic();
    }

    /**
     * @return Inverse of this Transform.
     */
    Transform getInverse() const;

    /**
     * Returns the composed transformation of the given transformations.
     * @return Transformation from this transform's base frame to `second`'s follower frame.
     */
    Transform compose(const Transform& second) const;

    Transform composeStatic(const Transform& second) const;

    Transform projectForward(float dt) const;

    /* Getters */
    inline Position getTranslation() const { return Position(translation); };

    inline Vector getVelocity() const { return Vector(transVel); };

    inline Vector getAcceleration() const { return Vector(transAcc); };

    inline Orientation getRotation() const { return Orientation(rotation); }

    inline Vector getAngularVel() const
    {
        return Vector(getRollVelocity(), getPitchVelocity(), getYawVelocity());
    }

    /**
     * Get the roll of this transformation
     */
    float getRoll() const;

    /**
     * Get the pitch of this transformation
     */
    float getPitch() const;

    /**
     * Get the yaw of this transformation
     */
    float getYaw() const;

    /**
     * Get the roll velocity of this transformation
     */
    float getRollVelocity() const;

    /**
     * Get the pitch velocity of this transformation
     */
    float getPitchVelocity() const;

    /**
     * Get the yaw velocity of this transformation
     */
    float getYawVelocity() const;

    /**
     * Get the x-component of this transform's translation
     */
    inline float getX() const { return this->translation.data[0]; }

    /**
     * Get the y-component of this transform's translation
     */
    inline float getY() const { return this->translation.data[1]; }

    /**
     * Get the z-component of this transform's translation
     */
    inline float getZ() const { return this->translation.data[2]; }

    /**
     * Get the x-component of this transform's translation
     */
    inline float getXVel() const { return this->transVel.data[0]; }

    /**
     * Get the y-component of this transform's translation
     */
    inline float getYVel() const { return this->transVel.data[1]; }

    /**
     * Get the z-component of this transform's translation
     */
    inline float getZVel() const { return this->transVel.data[2]; }

    /**
     * Get the x-component of this transform's translation
     */
    inline float getXAcc() const { return this->transAcc.data[0]; }

    /**
     * Get the y-component of this transform's translation
     */
    inline float getYAcc() const { return this->transAcc.data[1]; }

    /**
     * Get the z-component of this transform's translation
     */
    inline float getZAcc() const { return this->transAcc.data[2]; }

private:
    bool dynamic;

    /**
     * Translation vector.
     */
    CMSISMat<3, 1> translation;

    /**
     * Velocity vector.
     */
    CMSISMat<3, 1> transVel;

    /**
     * Acceleration vector.
     */
    CMSISMat<3, 1> transAcc;

    /**
     * Rotation matrix.
     */
    CMSISMat<3, 3> rotation;

    /**
     * Transpose of rotation matrix. Computed and stored at beginning
     * for use in other computations.
     *
     * The transpose of a rotation is its inverse.
     */
    CMSISMat<3, 3> tRotation;

    /**
     * Angular velocity skew matrix.
     */
    CMSISMat<3, 3> angVel;

    /**
     * Generates a 3x3 skew matrix from euler angle velocities (in radians/sec)
     */
    inline static CMSISMat<3, 3> skewMatFromAngVel(const float wx, const float wy, const float wz)
    {
        return tap::algorithms::CMSISMat<3, 3>({0, -wz, wx, wz, 0, -wy, -wx, wz, 0});
    }

    inline void checkDynamic()
    {
        dynamic = false;

        dynamic |=
            !(compareFloatClose(getXVel(), 0, 1e-5) && compareFloatClose(getYVel(), 0, 1e-5) &&
              compareFloatClose(getZVel(), 0, 1e-5));

        dynamic |=
            !(compareFloatClose(getXAcc(), 0, 1e-5) && compareFloatClose(getYAcc(), 0, 1e-5) &&
              compareFloatClose(getZAcc(), 0, 1e-5));

        dynamic |=
            !(compareFloatClose(getRollVelocity(), 0, 1e-5) &&
              compareFloatClose(getPitchVelocity(), 0, 1e-5) &&
              compareFloatClose(getYawVelocity(), 0, 1e-5));
    }
};  // class Transform
}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_TRANSFORM_HPP_

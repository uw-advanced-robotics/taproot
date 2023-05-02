#ifndef TAPROOT_TRANSFORM_HPP_
#define TAPROOT_TRANSFORM_HPP_

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/algorithms/math_user_utils.hpp"

namespace tap::algorithms::transforms
{
/**
 Represents a transformation from one coordinate frame to another.

    A Transform from frame A to frame B defines a relationship between the two frames, such that a
    spatial measurement in frame A can be represented equivalently in frame B by applying a
    translational and rotational offset. This process is known as *applying* a transform.

    Transforms are specified as a translation and rotation of some "target" frame relative to some
    "source" frame. The "translation" is the target frame's origin in source frame, and the
    "rotation" is the target frame's orientation relative to the source frame's orientation.

    Conceptually, translations are applied "before" rotations. This means that the origin of the
    target frame is entirely defined by the translation in the source frame, and the rotation serves
    only to change the orientation of the target frame's axes relative to the source frame.

    Utilizes arm's CMSIS matrix operations.

    @param SOURCE represents the source frame of the transformation.
    @param TARGET represents the target frame of the transformation.
 */
template <typename SOURCE, typename TARGET>
class Transform
{
    template<typename A, typename B, typename C>
    friend Transform<A, C> compose(const Transform<A, B>&, const Transform<B, C>&);
public:
    /**
     * Constructs a new Transform, which represents a transformation between two frames.
     *
     * @param translation Initial translation of this transformation.
     * @param rotation Initial rotation of this transformation.
     * 
     * @note Parameters are non-const due to move semantics.
     */
    Transform(CMSISMat<3, 1>& translation, CMSISMat<3, 3>& rotation);

    /**
     * Construct a new Transform, which represents a transformation between two frames.
     * 
     * Constructs rotations using ZYX Euler angles, so rotations are applied in order of A, B, then C.
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
    Transform(float x, float y, float z, float A, float B, float C);

    /**
     * Constructs a new Transform, which represents a transformation between two frames.
     */
    Transform();

    /**
     * @return Inverse of this Transform.
     */
    Transform<TARGET, SOURCE> getInverse() const;

    /**
     * Get the roll of this transformation
    */
    float getRoll() const;
    
    /**
     * Get the pitch of this transformation
    */
    float getPitch() const;
   
    /**
     * Get the pitch of this transformation
    */
    float getYaw() const;


    /**
     * Get the x-component of this transform's translation
    */
    float getX() const;

    /**
     * Get the x-component of this transform's translation
    */
    float getY() const;

    /**
     * Get the x-component of this transform's translation
    */
    float getZ() const;    

    /**
     * Transforms given position as read by the source frame
     * and computes the equivalent vector components in the target frame's basis.
     *
     * @param pos Position as read by source frame
     * @return Position in target frame's basis.
     */
    CMSISMat<3, 1> applyToPosition(const CMSISMat<3, 1>& pos) const;

    /**
     * Transforms a vector as read by the source frame and computes the equivalent vector
     * components in the target frame's basis. The difference from applyToPosition is that this
     * operation does not alter the magnitude of the components, and just rotates the provided
     * vector.
     * 
     * Intended to be used for things like velocities and accelerations which represent the difference
     * between two positions in space, since both positions get translated the same way, causing
     * the translation to cancel out.
     *
     * @param vec Vector as read by source frame.
     * @return Vector in target frame's basis.
     */
    CMSISMat<3, 1> applyToVector(const CMSISMat<3, 1>& vec) const;

    /**
     * Updates the rotation of the current transformation matrix.
     *
     * @param newRotation updated rotation matrix.
     * 
     * @note newRotation is non-const due to move semantics.
     */
    void updateRotation(CMSISMat<3, 3>& newRotation);

    /**
     * Updates the rotation of the current transformation matrix.
     * Takes rotation angles in the order of roll->pitch->yaw.
     *
     * @param A updated rotation angle about the x-axis.
     * @param B updated rotation angle about the y-axis.
     * @param C updated rotation angle about the z-axis.
     */
    void updateRotation(float A, float B, float C);

    /**
     * Updates the translation of the current transformation matrix.
     *
     * @param newTranslation updated translation vector.
     */
    void updateTranslation(const CMSISMat<3, 1>& newTranslation);

    /**
     * Updates the position of the current transformation matrix.
     *
     * @param x new translation x-component.
     * @param y new translation y-component.
     * @param z new translation z-component.
     */
    void updateTranslation(float x, float y, float z);

private:
    /**
     * Rotation matrix.
     */
    CMSISMat<3, 3> rotation;

    /**
     * Translation vector.
     */
    CMSISMat<3, 1> translation;

    /**
     * Transpose of rotation. Computed and stored at beginning
     * for use in other computations.
     * 
     * The transpose of a rotation is its inverse.
     */
    CMSISMat<3, 3> tRotation;
};

/**
 * Returns the composed transformation of the given transformations.
 *
 * @param source Transformation from frame A to frame B.
 * @param target Transformation from frame B to frame C.
 * @return Transformation from frame A to frame C.
 */
template <typename A, typename B, typename C>
Transform<A, C> compose(const Transform<A, B>& first, const Transform<B, C>& second);

}  // namespace tap::algorithms::transforms
#include "transform.cpp"
#endif

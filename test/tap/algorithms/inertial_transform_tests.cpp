#include <gtest/gtest.h>

#include "tap/algorithms/transforms/inertial_transform.hpp"
#include "tap/algorithms/transforms/position.hpp"
#include "tap/algorithms/transforms/pose.hpp"
#include "tap/algorithms/transforms/vector.hpp"
#include "tap/algorithms/transforms/frame.hpp"


using namespace tap::algorithms::transforms;
const Frame A, B;

TEST(InertialTransform, uh)
{
    // Given
    Position<A> pos(100.0, 0.0, 0.0);
    Vector<A> vel(0.0, 0.0, 0.0);
    InertialTransform<A, B> identity(
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0);

    // When
    Vector<B> newVel = identity.apply(pos, vel);

    // Then
    EXPECT_NEAR(vel.x(), newVel.x(), 1E-5);
    EXPECT_NEAR(vel.y(), newVel.y(), 1E-5);
    EXPECT_NEAR(vel.z(), newVel.z(), 1E-5);
}

TEST(InertialTransform, um)
{
    // Given
    Position<A> pos(0.0, 1.0, 0.0);
    Vector<A> vel(0.0, 0.0, 0.0);
    InertialTransform<A, B> identity(
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0);

    // When
    Vector<B> newVel = identity.apply(pos, vel);

    // Then
    Vector<B> expected(0.0, 0.0, -1.0);
    EXPECT_NEAR(expected.x(), newVel.x(), 1E-5);
    EXPECT_NEAR(expected.y(), newVel.y(), 1E-5);
    EXPECT_NEAR(expected.z(), newVel.z(), 1E-5);
}

TEST(InertialTransform, er)
{
    // Given
    Position<A> pos(0.0, 1.0, 0.0);
    Vector<A> vel(0.0, 0.0, 0.0);
    InertialTransform<A, B> identity(
        0.0, 0.0, 0.0,
        M_PI_2, 0.0, 0.0,
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0);

    // When
    Vector<B> newVel = identity.apply(pos, vel);

    // Then
    Vector<B> expected(0.0, -1.0, 0.0);
    EXPECT_NEAR(expected.x(), newVel.x(), 1E-5);
    EXPECT_NEAR(expected.y(), newVel.y(), 1E-5);
    EXPECT_NEAR(expected.z(), newVel.z(), 1E-5);
}

TEST(InertialTransform, ah)
{
    // Given
    Position<A> pos(0.0, 1.0, 0.0);
    Vector<A> vel(0.0, 0.0, 1.0);
    InertialTransform<A, B> identity(
        0.0, 0.0, 0.0,
        M_PI_2, 0.0, 0.0,
        1.0, 0.0, 0.0,
        1.0, 0.0, 0.0);

    // When
    Vector<B> newVel = identity.apply(pos, vel);

    // Then
    Vector<B> expected(-1.0, 0.0, 0.0);
    EXPECT_NEAR(expected.x(), newVel.x(), 1E-5);
    EXPECT_NEAR(expected.y(), newVel.y(), 1E-5);
    EXPECT_NEAR(expected.z(), newVel.z(), 1E-5);
}
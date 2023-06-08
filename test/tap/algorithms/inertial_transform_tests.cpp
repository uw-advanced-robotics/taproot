/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <gtest/gtest.h>

#include "tap/algorithms/transforms/inertial_transform.hpp"
#include "tap/algorithms/transforms/position.hpp"
#include "tap/algorithms/transforms/vector.hpp"
#include "tap/algorithms/transforms/frame.hpp"


using namespace tap::algorithms::transforms;
const Frame A, B;

TEST(InertialTransform, identity_transform_retains_position)
{
    // Given
    Position<A> pos(100.0, 0.0, 0.0);
    InertialTransform<A, B> identity(
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0);

    // When
    Position<B> newPos = identity.apply(pos);

    // Then
    EXPECT_NEAR(pos.x(), newPos.x(), 1E-5);
    EXPECT_NEAR(pos.y(), newPos.y(), 1E-5);
    EXPECT_NEAR(pos.z(), newPos.z(), 1E-5);
}

TEST(InertialTransform, identity_transform_retains_vector)
{
    // Given
    Vector<A> vec(100.0, 0.0, 0.0);
    InertialTransform<A, B> identity(
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0);

    // When
    Vector<B> newVec = identity.apply(vec);

    // Then
    EXPECT_NEAR(vec.x(), newVec.x(), 1E-5);
    EXPECT_NEAR(vec.y(), newVec.y(), 1E-5);
    EXPECT_NEAR(vec.z(), newVec.z(), 1E-5);
}

TEST(InertialTransform, identity_transform_retains_velocity)
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

TEST(InertialTransform, pure_ang_vel_transform_retains_position)
{
    // Given
    Position<A> pos(0.0, 1.0, 0.0);
    InertialTransform<A, B> identity(
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0);

    // When
    Position<B> newPos = identity.apply(pos);

    // Then
    Position<B> expected(0.0, 0.0, -1.0);
    EXPECT_NEAR(expected.x(), newPos.x(), 1E-5);
    EXPECT_NEAR(expected.y(), newPos.y(), 1E-5);
    EXPECT_NEAR(expected.z(), newPos.z(), 1E-5);
}

TEST(InertialTransform, pure_ang_vel_transform_apply_to_velocity)
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

TEST(InertialTransform, rotation_and_ang_vel_apply_to_velocity)
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

TEST(InertialTransform, full_apply_to_velocity)
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
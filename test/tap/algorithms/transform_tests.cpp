/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/algorithms/transforms/transform.hpp"
#include "tap/algorithms/transforms/position.hpp"
#include "tap/algorithms/transforms/pose.hpp"
#include "tap/algorithms/transforms/vector.hpp"
#include "tap/algorithms/transforms/frame.hpp"

using namespace tap::algorithms::transforms;

TEST(Transform, identity_transform_retains_position)
{
    // Given
    Frame A, B;
    Position<A> start(1.0, 2.0, 3.0);
    Transform<A, B> identity;

    // When
    Position<B> finish = identity.apply(start);

    // Then
    EXPECT_NEAR(start.x(), finish.x(), 1E-5);
    EXPECT_NEAR(start.y(), finish.y(), 1E-5);
    EXPECT_NEAR(start.z(), finish.z(), 1E-5);
}

TEST(Transform, identity_transform_retains_vector)
{
    // Given
    Frame A, B;
    Vector<A> start(1.0, 2.0, 3.0);
    Transform<A, B> identity;

    // When
    Vector<B> finish = identity.apply(start);

    // Then
    EXPECT_NEAR(start.x(), finish.x(), 1E-5);
    EXPECT_NEAR(start.y(), finish.y(), 1E-5);
    EXPECT_NEAR(start.z(), finish.z(), 1E-5);
}

TEST(Transform, identity_transform_retains_pose)
{
    // Given
    Frame A, B;
    Pose<A> start(1.0, 2.0, 3.0, M_PI_4, M_PI_2, M_PI);
    Transform<A, B> identity;

    // When
    Pose<B> finish = identity.apply(start);

    // Then
    EXPECT_NEAR(start.position().x(), finish.position().x(), 1E-5);
    EXPECT_NEAR(start.position().y(), finish.position().y(), 1E-5);
    EXPECT_NEAR(start.position().z(), finish.position().z(), 1E-5);
    EXPECT_EQ(start.orientation().roll(), finish.orientation().roll());
    EXPECT_EQ(start.orientation().pitch(), finish.orientation().pitch());
    EXPECT_EQ(start.orientation().yaw(), finish.orientation().yaw());
}

TEST(Transform, pure_translation_transform_apply_to_position)
{
    // Given
    Frame A, B;
    Position<A> start(1.0, 2.0, 3.0);
    Transform<A, B> translation(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);

    // When
    Position<B> finish = translation.apply(start);

    // Then
    Position<B> expected(0.0, 0.0, 0.0);

    EXPECT_NEAR(expected.x(), finish.x(), 1E-5);
    EXPECT_NEAR(expected.y(), finish.y(), 1E-5);
    EXPECT_NEAR(expected.z(), finish.z(), 1E-5);
}

TEST(Transform, pure_translation_transform_apply_to_vector)
{
    // Given
    Frame A, B;
    Vector<A> start(1.0, 2.0, 3.0);
    Transform<A, B> translation(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);

    // When
    Vector<B> finish = translation.apply(start);

    // Then
    EXPECT_NEAR(start.x(), finish.x(), 1E-5);
    EXPECT_NEAR(start.y(), finish.y(), 1E-5);
    EXPECT_NEAR(start.z(), finish.z(), 1E-5);
}

TEST(Transform, pure_translation_transform_apply_to_pose)
{
    // Given
    Frame A, B;
    Pose<A> start(1.0, 2.0, 3.0, M_PI_4, M_PI_2, M_PI);
    Transform<A, B> translation(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);

    // When
    Pose<B> finish = translation.apply(start);

    // Then
    Pose<A> expected(0.0, 0.0, 0.0, M_PI_4, M_PI_2, M_PI);
    EXPECT_NEAR(expected.position().x(), finish.position().x(), 1E-5);
    EXPECT_NEAR(expected.position().y(), finish.position().y(), 1E-5);
    EXPECT_NEAR(expected.position().z(), finish.position().z(), 1E-5);
    EXPECT_EQ(expected.orientation().roll(), finish.orientation().roll());
    EXPECT_EQ(expected.orientation().pitch(), finish.orientation().pitch());
    EXPECT_EQ(expected.orientation().yaw(), finish.orientation().yaw());
}

TEST(Transform, pure_roll_transform_apply_to_position)
{
    // Given
    Frame A, B;
    Position<A> start(1.0, 2.0, 3.0);
    Transform<A, B> roll(0.0, 0.0, 0.0, M_PI_2, 0.0, 0.0);

    // When
    Position<B> finish = roll.apply(start);

    // Then
    Position<B> expected(1.0, 3.0, -2.0);
    EXPECT_NEAR(expected.x(), finish.x(), 1E-5);
    EXPECT_NEAR(expected.y(), finish.y(), 1E-5);
    EXPECT_NEAR(expected.z(), finish.z(), 1E-5);
}

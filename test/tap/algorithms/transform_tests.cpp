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
#include "tap/algorithms/transforms/frames.hpp"

using namespace tap::algorithms::transforms;

TEST(Transform, identity_transform_retains_position)
{
    Frame A, B;
    Position<A> start(1, 2, 3);
    Transform<A, B> identityTF();
    Position<B> finish = identityTF.apply(start);
    EXPECT_EQ(start.coordinates, finish.coordinates);
}

TEST(Transform, identity_transform_retains_vector)
{
    Frame A, B;
    Vector<A> start(1, 2, 3);
    Transform<A, B> identityTF();
    Vector<B> finish = identityTF.apply(start);
    EXPECT_EQ(start.coordinates, finish.coordinates);
}

TEST(Transform, identity_transform_retains_pose)
{
    Frame A, B;
    Pose<A> start(1, 2, 3, M_PI_4, M_PI_2, M_PI);
    Transform<A, B> identityTF();
    Pose<B> finish = identityTF.apply(start);
    EXPECT_EQ(start.position.coordinates, finish.position.coordinates);
    EXPECT_EQ(start.roll(), finish.roll());
    EXPECT_EQ(start.pitch(), finish.pitch());
    EXPECT_EQ(start.yaw(), finish.yaw());
}

TEST(Transform, pure_translation_transform_apply_to_position)
{
    Frame A, B;
    Position<A> start(1, 2, 3);
    Transform<A, B> translation(4, 5, 6, 0, 0, 0);
    Position<B> finish = translation.apply(start);
    EXPECT_NEAR(start.x()+1, finish.x(), 1E-5);
    EXPECT_NEAR(start.y()+2, finish.y(), 1E-5);
    EXPECT_NEAR(start.z()+3, finish.z(), 1E-5);
}

TEST(Transform, pure_translation_transform_apply_to_vector)
{
    Frame A, B;
    Vector<A> start(1, 2, 3);
    Transform<A, B> translation(4, 5, 6, 0, 0, 0);
    Vector<B> finish = translation.apply(start);
    EXPECT_NEAR(start.x(), finish.x(), 1E-5);
    EXPECT_NEAR(start.y(), finish.y(), 1E-5);
    EXPECT_NEAR(start.z(), finish.z(), 1E-5);
}

TEST(Transform, pure_translation_transform_apply_to_pose)
{
    Frame A, B;
    Pose<A> start(1, 2, 3);
    Transform<A, B> translation(4, 5, 6, 0, 0, 0);
    Pose<B> finish = translation.apply(start);
    EXPECT_NEAR(start.position.x()+1, finish.position.x(), 1E-5);
    EXPECT_NEAR(start.position.y()+2, finish.position.y(), 1E-5);
    EXPECT_NEAR(start.position.z()+3, finish.position.z(), 1E-5);
    EXPECT_EQ(start.roll(), finish.roll());
    EXPECT_EQ(start.pitch(), finish.pitch());
    EXPECT_EQ(start.yaw(), finish.yaw());
}

TEST(Transform, pure_rotation_transform_apply_to_position)
{
    Frame A, B;
    Position<A> start(1, 2, 3);
    Transform<A, B> translation(0, 0, 0, 0, 0, 0);
    Position<B> finish = translation.apply(start);
    EXPECT_NEAR(start.x()+1, finish.x(), 1E-5);
    EXPECT_NEAR(start.y()+2, finish.y(), 1E-5);
    EXPECT_NEAR(start.z()+3, finish.z(), 1E-5);
}

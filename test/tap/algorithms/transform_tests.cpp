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

#include "tap/algorithms/transforms/position.hpp"
#include "tap/algorithms/transforms/transform.hpp"
#include "tap/algorithms/transforms/vector.hpp"

using namespace tap::algorithms::transforms;

void expectPosEq(const Transform& a, const Transform& b)
{
    EXPECT_NEAR(a.getX(), b.getX(), 1E-5);
    EXPECT_NEAR(a.getY(), b.getX(), 1E-5);
    EXPECT_NEAR(a.getZ(), b.getX(), 1E-5);
}

void expectVelEq(const Transform& a, const Transform& b)
{
    EXPECT_NEAR(a.getXVel(), b.getXVel(), 1E-5);
    EXPECT_NEAR(a.getYVel(), b.getXVel(), 1E-5);
    EXPECT_NEAR(a.getZVel(), b.getXVel(), 1E-5);
}

void expectAccEq(const Transform& a, const Transform& b)
{
    EXPECT_NEAR(a.getXAcc(), b.getXAcc(), 1E-5);
    EXPECT_NEAR(a.getYAcc(), b.getXAcc(), 1E-5);
    EXPECT_NEAR(a.getZAcc(), b.getXAcc(), 1E-5);
}

void expectDynPosEq(const Transform& a, const Transform& b)
{
    expectPosEq(a, b);
    expectVelEq(a, b);
    expectAccEq(a, b);
}

void expectAngEq(const Transform& a, const Transform& b)
{
    EXPECT_NEAR(a.getRoll(), b.getRoll(), 1E-5);
    EXPECT_NEAR(a.getPitch(), b.getPitch(), 1E-5);
    EXPECT_NEAR(a.getYaw(), b.getYaw(), 1E-5);
}

void expectAngVelEq(const Transform& a, const Transform& b)
{
    EXPECT_NEAR(a.getRollVelocity(), b.getRollVelocity(), 1E-5);
    EXPECT_NEAR(a.getPitchVelocity(), b.getPitchVelocity(), 1E-5);
    EXPECT_NEAR(a.getYawVelocity(), b.getYawVelocity(), 1E-5);
}

void expectDynAngEq(const Transform& a, const Transform& b)
{
    expectAngEq(a, b);
    expectAngVelEq(a, b);
}

void expectStaticEq(const Transform& a, const Transform& b)
{
    expectPosEq(a, b);
    expectAngEq(a, b);
}

void expectDynamicEq(const Transform& a, const Transform& b)
{
    expectDynPosEq(a, b);
    expectDynAngEq(a, b);
}

TEST(Transform, identity_transform_retains_position)
{
    // Given
    Position start(1.0, 2.0, 3.0);
    Transform identity(Transform::identity());

    // When
    Position finish = identity.apply(start);

    // Then
    EXPECT_NEAR(start.x(), finish.x(), 1E-5);
    EXPECT_NEAR(start.y(), finish.y(), 1E-5);
    EXPECT_NEAR(start.z(), finish.z(), 1E-5);
}

TEST(Transform, identity_transform_retains_vector)
{
    // Given
    Vector start(1.0, 2.0, 3.0);
    Transform identity(Transform::identity());

    // When
    Vector finish = identity.apply(start);

    // Then
    EXPECT_NEAR(start.x(), finish.x(), 1E-5);
    EXPECT_NEAR(start.y(), finish.y(), 1E-5);
    EXPECT_NEAR(start.z(), finish.z(), 1E-5);
}

TEST(Transform, pure_translation_transform_apply_to_target_position_yields_zero)
{
    // Given
    Position start(1.0, 2.0, 3.0);
    Transform translation(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);

    // When
    Position finish = translation.apply(start);

    // Then
    Position expected(0.0, 0.0, 0.0);

    EXPECT_NEAR(expected.x(), finish.x(), 1E-5);
    EXPECT_NEAR(expected.y(), finish.y(), 1E-5);
    EXPECT_NEAR(expected.z(), finish.z(), 1E-5);
}

TEST(Transform, pure_translation_transform_apply_to_source_position_yields_negative_translation)
{
    // Given
    Position start(0.0, 0.0, 0.0);
    Transform translation(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);

    // When
    Position finish = translation.apply(start);

    // Then
    Position expected(-1.0, -2.0, -3.0);

    EXPECT_NEAR(expected.x(), finish.x(), 1E-5);
    EXPECT_NEAR(expected.y(), finish.y(), 1E-5);
    EXPECT_NEAR(expected.z(), finish.z(), 1E-5);
}

TEST(Transform, pure_translation_transform_apply_to_vector)
{
    // Given
    Vector start(1.0, 2.0, 3.0);
    Transform translation(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);

    // When
    Vector finish = translation.apply(start);

    // Then
    EXPECT_NEAR(start.x(), finish.x(), 1E-5);
    EXPECT_NEAR(start.y(), finish.y(), 1E-5);
    EXPECT_NEAR(start.z(), finish.z(), 1E-5);
}

TEST(Transform, pure_roll_transform_apply_to_position)
{
    // Given
    Position start(1.0, 2.0, 3.0);
    Transform roll(0.0, 0.0, 0.0, M_PI_2, 0.0, 0.0);

    // When
    Position finish = roll.apply(start);

    // Then
    Position expected(1.0, 3.0, -2.0);
    EXPECT_NEAR(expected.x(), finish.x(), 1E-5);
    EXPECT_NEAR(expected.y(), finish.y(), 1E-5);
    EXPECT_NEAR(expected.z(), finish.z(), 1E-5);
}

TEST(Transform, pure_pitch_transform_apply_to_position)
{
    // Given
    Position start(1.0, 2.0, 3.0);
    Transform pitch(0.0, 0.0, 0.0, 0, M_PI_2, 0.0);

    // When
    Position finish = pitch.apply(start);

    // Then
    Position expected(-3.0, 2.0, 1.0);
    EXPECT_NEAR(expected.x(), finish.x(), 1E-5);
    EXPECT_NEAR(expected.y(), finish.y(), 1E-5);
    EXPECT_NEAR(expected.z(), finish.z(), 1E-5);
}

TEST(Transform, pure_yaw_transform_apply_to_position)
{
    // Given
    Position start(1.0, 2.0, 3.0);
    Transform yaw(0.0, 0.0, 0.0, 0.0, 0.0, M_PI_2);

    // When
    Position finish = yaw.apply(start);

    // Then
    Position expected(2.0, -1.0, 3.0);
    EXPECT_NEAR(expected.x(), finish.x(), 1E-5);
    EXPECT_NEAR(expected.y(), finish.y(), 1E-5);
    EXPECT_NEAR(expected.z(), finish.z(), 1E-5);
}

TEST(Transform, pure_rotation_transform_apply_to_zero_position)
{
    // Given
    Position start(0.0, 0.0, 0.0);
    Transform rotation(0.0, 0.0, 0.0, M_SQRT2, -1.0, M_2_PI);

    // When
    Position finish = rotation.apply(start);

    // Then
    Position expected(0.0, 0.0, 0.0);
    EXPECT_NEAR(expected.x(), finish.x(), 1E-5);
    EXPECT_NEAR(expected.y(), finish.y(), 1E-5);
    EXPECT_NEAR(expected.z(), finish.z(), 1E-5);
}

TEST(Transform, transform_apply_to_target_origin_position_yields_zero)
{
    // Given
    Position start(1.0, 2.0, 3.0);
    Transform rotation(1.0, 2.0, 3.0, M_SQRT2, -1.0, M_2_PI);

    // When
    Position finish = rotation.apply(start);

    // Then
    Position expected(0.0, 0.0, 0.0);
    EXPECT_NEAR(expected.x(), finish.x(), 1E-5);
    EXPECT_NEAR(expected.y(), finish.y(), 1E-5);
    EXPECT_NEAR(expected.z(), finish.z(), 1E-5);
}

TEST(Transform, transform_apply_to_source_origin_position)
{
    // Given
    Position start(1.0, 2.0, 3.0);
    Transform rotation(1.0, 2.0, 3.0, M_SQRT2, -1.0, M_2_PI);

    // When
    Position finish = rotation.apply(start);

    // Then
    Position expected(0.0, 0.0, 0.0);
    EXPECT_NEAR(expected.x(), finish.x(), 1E-5);
    EXPECT_NEAR(expected.y(), finish.y(), 1E-5);
    EXPECT_NEAR(expected.z(), finish.z(), 1E-5);
}

TEST(Transform, transform_compose_with_inverse_yields_identity)
{
    // Given
    Transform transform(0.0, 0.0, 0.0, M_SQRT2, -1.0, M_2_PI);

    // When
    Transform composed = transform.compose(transform.getInverse());

    // Then
    Transform identity(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    expectStaticEq(composed, identity);
}

TEST(Transform, dynamic_transform_compose_with_inverse_yields_identity)
{
    // Given
    Transform transform(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);

    // When
    Transform composed = transform.compose(transform.getInverse());

    // Then
    Transform identity = Transform::identity();
    expectDynamicEq(composed, identity);
}

TEST(Transform, dynamic_transform_compose_with_inverse_yields_identity)
{
    // Given
    Transform transform(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);

    // When
    Transform composed = transform.compose(transform.getInverse());

    // Then
    Transform identity = Transform::identity();
    expectDynamicEq(composed, identity);
}

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

const float EPS = 1E-5;

void expectPosEq(const Position& a, const Position& b, const float epsilon = EPS)
{
    EXPECT_NEAR(a.x(), b.x(), epsilon);
    EXPECT_NEAR(a.y(), b.y(), epsilon);
    EXPECT_NEAR(a.z(), b.z(), epsilon);
}

void expectVecEq(const Vector& a, const Vector& b, const float epsilon = EPS)
{
    EXPECT_NEAR(a.x(), b.x(), epsilon);
    EXPECT_NEAR(a.y(), b.y(), epsilon);
    EXPECT_NEAR(a.z(), b.z(), epsilon);
}

void expectPosEq(const Transform& a, const Transform& b, const float epsilon = EPS)
{
    EXPECT_NEAR(a.getX(), b.getX(), epsilon);
    EXPECT_NEAR(a.getY(), b.getX(), epsilon);
    EXPECT_NEAR(a.getZ(), b.getX(), epsilon);
}

void expectVelEq(const Transform& a, const Transform& b, const float epsilon = EPS)
{
    EXPECT_NEAR(a.getXVel(), b.getXVel(), epsilon);
    EXPECT_NEAR(a.getYVel(), b.getXVel(), epsilon);
    EXPECT_NEAR(a.getZVel(), b.getXVel(), epsilon);
}

void expectAccEq(const Transform& a, const Transform& b, const float epsilon = EPS)
{
    EXPECT_NEAR(a.getXAcc(), b.getXAcc(), epsilon);
    EXPECT_NEAR(a.getYAcc(), b.getXAcc(), epsilon);
    EXPECT_NEAR(a.getZAcc(), b.getXAcc(), epsilon);
}

void expectDynPosEq(const Transform& a, const Transform& b, const float epsilon = EPS)
{
    expectPosEq(a, b, epsilon);
    expectVelEq(a, b, epsilon);
    expectAccEq(a, b, epsilon);
}

void expectAngEq(const Transform& a, const Transform& b, const float epsilon = EPS)
{
    EXPECT_NEAR(a.getRoll(), b.getRoll(), epsilon);
    EXPECT_NEAR(a.getPitch(), b.getPitch(), epsilon);
    EXPECT_NEAR(a.getYaw(), b.getYaw(), epsilon);
}

void expectAngVelEq(const Transform& a, const Transform& b, const float epsilon = EPS)
{
    EXPECT_NEAR(a.getRollVelocity(), b.getRollVelocity(), epsilon);
    EXPECT_NEAR(a.getPitchVelocity(), b.getPitchVelocity(), epsilon);
    EXPECT_NEAR(a.getYawVelocity(), b.getYawVelocity(), epsilon);
}

void expectDynAngEq(const Transform& a, const Transform& b, const float epsilon = EPS)
{
    expectAngEq(a, b, epsilon);
    expectAngVelEq(a, b, epsilon);
}

void expectStaticEq(const Transform& a, const Transform& b, const float epsilon = EPS)
{
    expectPosEq(a, b, epsilon);
    expectAngEq(a, b, epsilon);
}

void expectDynamicEq(const Transform& a, const Transform& b, const float epsilon = EPS)
{
    expectDynPosEq(a, b, epsilon);
    expectDynAngEq(a, b, epsilon);
}

TEST(Transform, identity_transform_retains_position)
{
    // Given
    Position start(1.0, 2.0, 3.0);
    Transform identity(Transform::identity());

    // When
    Position finish = identity.apply(start);

    // Then
    expectPosEq(start, finish);
}

TEST(Transform, identity_transform_retains_vector)
{
    // Given
    Vector start(1.0, 2.0, 3.0);
    Transform identity(Transform::identity());

    // When
    Vector finish = identity.apply(start);

    // Then
    expectVecEq(start, finish);
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

    expectPosEq(expected, finish);
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

    expectPosEq(expected, finish);
}

TEST(Transform, pure_translation_transform_apply_to_vector)
{
    // Given
    Vector start(1.0, 2.0, 3.0);
    Transform translation(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);

    // When
    Vector finish = translation.apply(start);

    // Then
    expectVecEq(start, finish);
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
    expectPosEq(expected, finish);
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
    expectPosEq(expected, finish);
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
    expectPosEq(expected, finish);
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
    expectPosEq(expected, finish);
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
    expectPosEq(expected, finish);
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
    expectPosEq(expected, finish);
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

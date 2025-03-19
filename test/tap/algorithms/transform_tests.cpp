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
using namespace testing;

const float EPS = 1E-5;

inline void expectPosEq(const Position& actual, const Position& expected, const float epsilon = EPS)
{
    EXPECT_NEAR(actual.x(), expected.x(), epsilon);
    EXPECT_NEAR(actual.y(), expected.y(), epsilon);
    EXPECT_NEAR(actual.z(), expected.z(), epsilon);
}

inline void expectVecEq(const Vector& actual, const Vector& expected, const float epsilon = EPS)
{
    EXPECT_NEAR(actual.x(), expected.x(), epsilon);
    EXPECT_NEAR(actual.y(), expected.y(), epsilon);
    EXPECT_NEAR(actual.z(), expected.z(), epsilon);
}

inline void expectPosEq(
    const Transform& actual,
    const Transform& expected,
    const float epsilon = EPS)
{
    EXPECT_NEAR(actual.getX(), expected.getX(), epsilon);
    EXPECT_NEAR(actual.getY(), expected.getY(), epsilon);
    EXPECT_NEAR(actual.getZ(), expected.getZ(), epsilon);
}

inline void expectVelEq(
    const Transform& actual,
    const Transform& expected,
    const float epsilon = EPS)
{
    EXPECT_NEAR(actual.getXVel(), expected.getXVel(), epsilon);
    EXPECT_NEAR(actual.getYVel(), expected.getYVel(), epsilon);
    EXPECT_NEAR(actual.getZVel(), expected.getZVel(), epsilon);
}

inline void expectAccEq(
    const Transform& actual,
    const Transform& expected,
    const float epsilon = EPS)
{
    EXPECT_NEAR(actual.getXAcc(), expected.getXAcc(), epsilon);
    EXPECT_NEAR(actual.getYAcc(), expected.getYAcc(), epsilon);
    EXPECT_NEAR(actual.getZAcc(), expected.getZAcc(), epsilon);
}

inline void expectDynPosEq(
    const Transform& actual,
    const Transform& expected,
    const float epsilon = EPS)
{
    expectPosEq(actual, expected, epsilon);
    expectVelEq(actual, expected, epsilon);
    expectAccEq(actual, expected, epsilon);
}

inline void expectAngEq(
    const Transform& actual,
    const Transform& expected,
    const float epsilon = EPS)
{
    EXPECT_NEAR(actual.getRoll(), expected.getRoll(), epsilon);
    EXPECT_NEAR(actual.getPitch(), expected.getPitch(), epsilon);
    EXPECT_NEAR(actual.getYaw(), expected.getYaw(), epsilon);
}

inline void expectAngVelEq(
    const Transform& actual,
    const Transform& expected,
    const float epsilon = EPS)
{
    EXPECT_NEAR(actual.getRollVelocity(), expected.getRollVelocity(), epsilon);
    EXPECT_NEAR(actual.getPitchVelocity(), expected.getPitchVelocity(), epsilon);
    EXPECT_NEAR(actual.getYawVelocity(), expected.getYawVelocity(), epsilon);
}

inline void expectDynAngEq(
    const Transform& actual,
    const Transform& expected,
    const float epsilon = EPS)
{
    expectAngEq(actual, expected, epsilon);
    expectAngVelEq(actual, expected, epsilon);
}

inline void expectStaticEq(
    const Transform& actual,
    const Transform& expected,
    const float epsilon = EPS)
{
    expectPosEq(actual, expected, epsilon);
    expectAngEq(actual, expected, epsilon);
}

inline void expectDynamicEq(
    const Transform& actual,
    const Transform& expected,
    const float epsilon = EPS)
{
    expectDynPosEq(actual, expected, epsilon);
    expectDynAngEq(actual, expected, epsilon);
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
    Transform transform(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
    // Transform
    // transform(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);

    // When
    Transform composed = transform.compose(transform.getInverse());

    // Then
    Transform identity = Transform::identity();
    expectDynamicEq(composed, identity);
}

struct CompositionTestConfig
{
    Transform a, b, e;
};

class CompositionTest : public TestWithParam<CompositionTestConfig>
{
};

TEST_P(CompositionTest, dynamic_compose)
{
    expectDynamicEq(GetParam().a.compose(GetParam().b), GetParam().e);
}

//        Transform(  x,   y,   z,  vx,  vy,  vz,  ax,  ay,  az, roll, pitch, yaw, rollVel,
//        pitchVel, yawVel)
std::vector<CompositionTestConfig> dynamicComposeTestCases = {
    // dynamic * static
    // (+yaw vel) * (+x+z translation) = (+x+z translation, +y vel, -x acc, +yaw vel)
    {.a = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
     .b = Transform(1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .e = Transform(1.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)},
    {.a = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
     .b = Transform(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .e = Transform(1.0, 1.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)},
    {.a = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
     .b = Transform(1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .e = Transform(1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)},

};

INSTANTIATE_TEST_SUITE_P(Transform, CompositionTest, ValuesIn(dynamicComposeTestCases));

/*
tests to do:

rot vel composed with static
    tangent vel
    centripetal acc
coriolis effect


*/
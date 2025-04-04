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

#include <tuple>

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

// Transform(  x,   y,   z,  vx,  vy,  vz,  ax,  ay,  az, roll, pitch, yaw, rollVel, pitchVel,
// yawVel)
std::vector<CompositionTestConfig> dynamicComposeTestCases = {
    {.a = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .b = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .e = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
    // dynamic * static
    // (ang vel) * (translation) = (translation, vel, acc, ang vel)
    {.a = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
     .b = Transform(1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .e = Transform(1.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)},
    {.a = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
     .b = Transform(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .e = Transform(1.0, 1.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)},
    {.a = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
     .b = Transform(1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .e = Transform(1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)},

    // static * dynamic
    {.a = Transform(1.0, 1.0, 1.0, 2.0, 1.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .b = Transform(2.0, 2.0, 2.0, 0.0, 0.0, 0.0, 1.0, 2.0, 1.0, 3.0, 5.0, 5.0, 1.0, 3.0, 2.0),
     .e = Transform(3.0, 3.0, 3.0, 2.0, 1.0, 3.0, 1.0, 2.0, 1.0, 3.0, 5.0, 5.0, 1.0, 3.0, 2.0)},
    {.a = Transform(1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .b = Transform(2.0, 2.0, 2.0, 2.0, 1.0, 3.0, 1.0, 2.0, 1.0, 3.0, 5.0, 5.0, 1.0, 3.0, 2.0),
     .e = Transform(3.0, 3.0, 3.0, 2.0, 1.0, 3.0, 1.0, 2.0, 1.0, 3.0, 5.0, 5.0, 1.0, 3.0, 2.0)},

    // dynamic * dynamic
    {.a = Transform(1.0, 3.0, 2.0, 5.0, 4.0, 7.0, 6.0, 9.0, 8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .b = Transform(9.0, 7.0, 8.0, 5.0, 6.0, 3.0, 4.0, 1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .e = Transform(10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, .0, .0, .0, .0, .0, .0)},
    {.a = Transform(1.0, 3.0, 2.0, 5.0, 4.0, 7.0, 6.0, 9.0, 8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .b = Transform(9.0, 7.0, 8.0, 5.0, 6.0, 3.0, 4.0, 1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .e = Transform(10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, .0, .0, .0, .0, .0, .0)},

};

INSTANTIATE_TEST_SUITE_P(Transform, CompositionTest, ValuesIn(dynamicComposeTestCases));

struct PositionAdditionConsistencyTestConfig
{
    DynamicPosition a, b;
};

class PositionAdditionConsistencyTest : public TestWithParam<PositionAdditionConsistencyTestConfig>
{
};

TEST_P(PositionAdditionConsistencyTest, position_composition_consistency)
{
    Transform tA(GetParam().a, DynamicOrientation(0, 0, 0, 0, 0, 0));
    Transform tB(GetParam().b, DynamicOrientation(0, 0, 0, 0, 0, 0));
    Transform tAc = tA.compose(tB);

    Transform tE(GetParam().a + GetParam().b, DynamicOrientation(0, 0, 0, 0, 0, 0));

    expectDynamicEq(tAc, tE);
}

std::vector<PositionAdditionConsistencyTestConfig> positionAdditionConsistencyTestCases = {
    {.a = DynamicPosition(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .b = DynamicPosition(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
    {.a = DynamicPosition(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
     .b = DynamicPosition(2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0)},
};

INSTANTIATE_TEST_SUITE_P(
    Transform,
    PositionAdditionConsistencyTest,
    ValuesIn(positionAdditionConsistencyTestCases));

struct OrientationCompositionConsistencyTestConfig
{
    DynamicOrientation a, b;
};

class OrientationCompositionConsistencyTest
    : public TestWithParam<OrientationCompositionConsistencyTestConfig>
{
};

TEST_P(OrientationCompositionConsistencyTest, position_composition_consistency)
{
    Transform tA(DynamicPosition(0, 0, 0, 0, 0, 0, 0, 0, 0), GetParam().a);
    Transform tB(DynamicPosition(0, 0, 0, 0, 0, 0, 0, 0, 0), GetParam().b);
    Transform tAc = tA.compose(tB);

    Transform tE(DynamicPosition(0, 0, 0, 0, 0, 0, 0, 0, 0), GetParam().a.compose(GetParam().b));

    expectDynamicEq(tAc, tE);
}

std::vector<OrientationCompositionConsistencyTestConfig>
    orientationCompositionConsistencyTestCases = {
        {.a = DynamicOrientation(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
         .b = DynamicOrientation(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
        {.a = DynamicOrientation(1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
         .b = DynamicOrientation(2.0, 2.0, 2.0, 2.0, 2.0, 2.0)},
};

INSTANTIATE_TEST_SUITE_P(
    Transform,
    OrientationCompositionConsistencyTest,
    ValuesIn(orientationCompositionConsistencyTestCases));

struct ProjectionTestConfig
{
    Transform t;
    float dt;
    Transform e;
};

class ProjectionTest : public TestWithParam<ProjectionTestConfig>
{
};

TEST_P(ProjectionTest, projection_test)
{
    expectDynamicEq(GetParam().t.projectForward(GetParam().dt), GetParam().e);
}

std::vector<ProjectionTestConfig> projectionTestCases = {
    // trivial case
    {.t = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .dt = 1.0f,
     .e = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},

    // static transform not affected by projection
    {.t = Transform(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0),
     .dt = 1.0f,
     .e = Transform(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0)},

    // pure translation projection
    {.t = Transform(3.0, 3.0, 3.0, 2.0, 2.0, 2.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .dt = 2.0f,
     .e = Transform(9.0, 9.0, 9.0, 4.0, 4.0, 4.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},

    // pure rotation projection
    {.t = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1, 0.0, 0.0),
     .dt = 2.0f,
     .e = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.1, 0.0, 0.0)},
    {.t = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1, 0.0),
     .dt = 2.0f,
     .e = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.1, 0.0)},
    {.t = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1),
     .dt = 2.0f,
     .e = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.1)},
};

INSTANTIATE_TEST_SUITE_P(Transform, ProjectionTest, ValuesIn(projectionTestCases));
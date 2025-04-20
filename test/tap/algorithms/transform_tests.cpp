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

inline void expectEq(const Position& actual, const Position& expected, const float epsilon = EPS)
{
    EXPECT_NEAR(actual.x(), expected.x(), epsilon);
    EXPECT_NEAR(actual.y(), expected.y(), epsilon);
    EXPECT_NEAR(actual.z(), expected.z(), epsilon);
}

inline void expectEq(const Vector& actual, const Vector& expected, const float epsilon = EPS)
{
    EXPECT_NEAR(actual.x(), expected.x(), epsilon);
    EXPECT_NEAR(actual.y(), expected.y(), epsilon);
    EXPECT_NEAR(actual.z(), expected.z(), epsilon);
}

inline void expectEq(
    const DynamicPosition& actual,
    const DynamicPosition& expected,
    const float epsilon = EPS)
{
    expectEq(actual.getPosition(), expected.getPosition(), epsilon);
    expectEq(actual.getVelocity(), expected.getVelocity(), epsilon);
    expectEq(actual.getAcceleration(), expected.getAcceleration(), epsilon);
}

inline void expectEq(
    const Orientation& actual,
    const Orientation& expected,
    const float epsilon = EPS)
{
    EXPECT_NEAR(actual.roll(), expected.roll(), epsilon);
    EXPECT_NEAR(actual.pitch(), expected.pitch(), epsilon);
    EXPECT_NEAR(actual.yaw(), expected.yaw(), epsilon);
}

inline void expectEq(
    const AngularVelocity& actual,
    const AngularVelocity& expected,
    const float epsilon = EPS)
{
    EXPECT_NEAR(actual.getRollVelocity(), expected.getRollVelocity(), epsilon);
    EXPECT_NEAR(actual.getPitchVelocity(), expected.getPitchVelocity(), epsilon);
    EXPECT_NEAR(actual.getYawVelocity(), expected.getYawVelocity(), epsilon);
}

inline void expectEq(
    const DynamicOrientation& actual,
    const DynamicOrientation& expected,
    const float epsilon = EPS)
{
    expectEq(actual.getOrientation(), expected.getOrientation(), epsilon);
    expectEq(actual.getAngularVelocity(), expected.getAngularVelocity(), epsilon);
}

inline void expectStaticEq(
    const Transform& actual,
    const Transform& expected,
    const float epsilon = EPS)
{
    expectEq(actual.getTranslation(), expected.getTranslation(), epsilon);
    expectEq(actual.getRotation(), expected.getRotation(), epsilon);
}

inline void expectEq(const Transform& actual, const Transform& expected, const float epsilon = EPS)
{
    expectEq(actual.getDynamicTranslation(), expected.getDynamicTranslation(), epsilon);
    expectEq(actual.getDynamicOrientation(), expected.getDynamicOrientation(), epsilon);
}

TEST(Transform, identity_transform_retains_position)
{
    // Given
    Position start(1.0, 2.0, 3.0);
    Transform identity(Transform::identity());

    // When
    Position finish = identity.apply(start);

    // Then
    expectEq(start, finish);
}

TEST(Transform, identity_transform_retains_vector)
{
    // Given
    Vector start(1.0, 2.0, 3.0);
    Transform identity(Transform::identity());

    // When
    Vector finish = identity.apply(start);

    // Then
    expectEq(start, finish);
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

    expectEq(expected, finish);
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

    expectEq(expected, finish);
}

TEST(Transform, pure_translation_transform_apply_to_vector)
{
    // Given
    Vector start(1.0, 2.0, 3.0);
    Transform translation(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);

    // When
    Vector finish = translation.apply(start);

    // Then
    expectEq(start, finish);
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
    expectEq(expected, finish);
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
    expectEq(expected, finish);
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
    expectEq(expected, finish);
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
    expectEq(expected, finish);
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
    expectEq(expected, finish);
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
    expectEq(expected, finish);
}

TEST(Transform, transform_compose_with_inverse_yields_identity)
{
    // Given
    Transform transform(0.0, 0.0, 0.0, M_SQRT2, -1.0, M_2_PI);

    // When
    Transform composed = transform.compose(transform.getInverse());

    // Then
    Transform identity(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    expectEq(composed, identity);
}

TEST(Transform, dynamic_transform_compose_with_inverse_yields_identity)
{
    // Given
    Transform transform(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

    // When
    Transform composed = transform.compose(transform.getInverse());

    // Then
    Transform identity = Transform::identity();
    expectEq(composed, identity);
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
    expectEq(GetParam().a.compose(GetParam().b), GetParam().e);
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

    expectEq(tAc, tE);
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

    expectEq(tAc, tE);
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

TEST_P(ProjectionTest, projection)
{
    expectEq(GetParam().t.projectForward(GetParam().dt), GetParam().e);
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
    {.t = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1, 0.0, 0.0),
     .dt = M_PI * 20.0f,
     .e = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1, 0.0, 0.0)},
};

INSTANTIATE_TEST_SUITE_P(Transform, ProjectionTest, ValuesIn(projectionTestCases));

struct ApplyDynamicPosTestConfig
{
    Transform t;
    DynamicPosition p, e;
};

class ApplyDynamicPosTest : public TestWithParam<ApplyDynamicPosTestConfig>
{
};

TEST_P(ApplyDynamicPosTest, apply_dynamic_position)
{
    expectEq(GetParam().t.apply(GetParam().p), GetParam().e);
}

std::vector<ApplyDynamicPosTestConfig> applyDynamicPosTestCases = {
    // trivial case
    {.t = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .p = DynamicPosition(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .e = DynamicPosition(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},

    {.t = Transform(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .p = DynamicPosition(1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0, 2.0, 3.0),
     .e = DynamicPosition(0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 1.0, 2.0, 3.0)},
    {.t = Transform(1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .p = DynamicPosition(1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0, 2.0, 3.0),
     .e = DynamicPosition(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},

    {.t = Transform(1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, M_PI_2, 0.0, 0.0, 1.0),
     .p = DynamicPosition(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .e = DynamicPosition(0.0, 1.0, -1.0, 1.0, 0.0, 0.0, 0.0, -1.0, 0.0)},
};

struct ApplyDynamicOriTestConfig
{
    Transform t;
    DynamicOrientation o, e;
};

class ApplyDynamicOriTest : public TestWithParam<ApplyDynamicOriTestConfig>
{
};

TEST_P(ApplyDynamicOriTest, apply_dynamic_orientation)
{
    expectEq(GetParam().t.apply(GetParam().o), GetParam().e);
}

std::vector<ApplyDynamicOriTestConfig> applyDynamicOriTestCases = {
    // trivial case
    {.t = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .o = DynamicOrientation(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .e = DynamicOrientation(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},

    // translation shouldn't affect this
    {.t = Transform(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .o = DynamicOrientation(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .e = DynamicOrientation(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},

    {.t = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
     .o = DynamicOrientation(0.1, 0.2, 0.3, 0.3, 0.2, 0.1),
     .e = DynamicOrientation(0.1, 0.2, 0.3, 0.3, 0.2, 0.1)},

    {.t = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, M_PI_2, 0.0, 0.0, 1.0),
     .o = DynamicOrientation(0.0, 0.0, M_PI_2, 0.0, 0.0, 0.0),
     .e = DynamicOrientation(0.0, 0.0, 0.0, 0.0, 0.0, -1.0)},
};

INSTANTIATE_TEST_SUITE_P(Transform, ApplyDynamicOriTest, ValuesIn(applyDynamicOriTestCases));

std::ostream& operator<<(std::ostream& stream, const Transform&) { return stream << "Transform"; }

std::ostream& operator<<(std::ostream& stream, const DynamicPosition&)
{
    return stream << "DynamicPosition";
}

std::ostream& operator<<(std::ostream& stream, const DynamicOrientation&)
{
    return stream << "DynamicOrientation";
}

std::ostream& operator<<(std::ostream& stream, const CompositionTestConfig&)
{
    return stream << "CompositionTestConfig";
}

std::ostream& operator<<(std::ostream& stream, const PositionAdditionConsistencyTestConfig&)
{
    return stream << "PositionAdditionConsistencyTestConfig";
}

std::ostream& operator<<(std::ostream& stream, const OrientationCompositionConsistencyTestConfig&)
{
    return stream << "OrientationCompositionConsistencyTestConfig";
}

std::ostream& operator<<(std::ostream& stream, const ProjectionTestConfig&)
{
    return stream << "ProjectionTestConfig";
}

std::ostream& operator<<(std::ostream& stream, const ApplyDynamicPosTestConfig&)
{
    return stream << "ApplyDynamicPosTestConfig";
}

std::ostream& operator<<(std::ostream& stream, const ApplyDynamicOriTestConfig&)
{
    return stream << "ApplyDynamicOriTestConfig";
}

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
    Transform<A, A> composed = transform.compose(transform.getInverse());

    // Then
    Transform<A, A> identity(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    EXPECT_NEAR(identity.getTranslation().x(), composed.getTranslation().x(), 1E-5);
    EXPECT_NEAR(identity.getTranslation().y(), composed.getTranslation().y(), 1E-5);
    EXPECT_NEAR(identity.getTranslation().z(), composed.getTranslation().z(), 1E-5);
    EXPECT_NEAR(identity.getRotation().roll(), composed.getRotation().roll(), 1E-5);
    EXPECT_NEAR(identity.getRotation().pitch(), composed.getRotation().pitch(), 1E-5);
    EXPECT_NEAR(identity.getRotation().yaw(), composed.getRotation().yaw(), 1E-5);
}

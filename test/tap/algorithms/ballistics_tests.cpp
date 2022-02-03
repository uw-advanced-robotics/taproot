/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/algorithms/ballistics.hpp"

using namespace tap::algorithms::ballistics;

TEST(Ballistics, quadraticKinematicProjection_dt_zero_pos_unmoving)
{
    EXPECT_EQ(1, quadraticKinematicProjection(0, 1, 2, 3));
}

TEST(
    Ballistics,
    quadraticKinematicProjection_position_constantly_increasing_when_vel_positive_acc_zero)
{
    EXPECT_NEAR(2, quadraticKinematicProjection(1, 1, 1, 0), 1E-5);
}

TEST(Ballistics, quadraticKinematicProjection_position_increases_quadratically_when_acc_positive)
{
    EXPECT_NEAR(1, quadraticKinematicProjection(1, 0, 0, 2), 1e-5);
    EXPECT_NEAR(4, quadraticKinematicProjection(2, 0, 0, 2), 1e-5);
    EXPECT_NEAR(9, quadraticKinematicProjection(3, 0, 0, 2), 1e-5);
}

TEST(Ballistics, projectForward_returns_constant_delta_position_when_vel_positive_no_acc)
{
    MeasuredKinematicState kinematicState{
        .position = {1, 1, 1},
        .velocity = {1, 1, 1},
        .acceleration = {0, 0, 0},
    };

    auto position1 = projectForward(kinematicState, 1);
    auto position2 = projectForward(kinematicState, 2);
    auto position3 = projectForward(kinematicState, 3);

    auto diff12 = position2 - position1;
    auto diff23 = position3 - position2;

    EXPECT_NEAR(diff12.x, diff23.x, 1E-5);
    EXPECT_NEAR(diff12.y, diff23.y, 1E-5);
    EXPECT_NEAR(diff12.z, diff23.z, 1E-5);
}

TEST(Ballistics, computeTravelTime_horizontal_distance_only_bulletVelocity_10)
{
    modm::Vector3f turretPosition(5, 0, 0);
    modm::Vector3f targetPosition(15, 0, 0);

    float travelTime = computeTravelTime(turretPosition, targetPosition, 20);

    // 20 m/s velocity, 10 m target distance, time should be slightly greater than .5
    EXPECT_LT(1, travelTime);
    EXPECT_NEAR(1, travelTime, 0.3f);
}

TEST(Ballistics, computeTravelTime_vertical_distance_only_bulletVelocity_10)
{
    modm::Vector3f turretPosition(0, 0, 0);
    modm::Vector3f targetPosition(0, 0, 10);

    float travelTime = computeTravelTime(turretPosition, targetPosition, 10);

    EXPECT_NEAR(10, travelTime, 1E-5);
}

TEST(Ballistics, computeTravelTime_bulletVelocity_0_travel_time_0)
{
    modm::Vector3f turretPosition(1, 2, 3);
    modm::Vector3f targetPosition(4, 5, 5);

    float travelTime = computeTravelTime(turretPosition, targetPosition, 0);

    EXPECT_EQ(0, travelTime);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_horizontal_distance_btwn_turret_and_target_target_stationary)
{
    modm::Vector3f turretPosition(0, 0, 0);
    MeasuredKinematicState targetState{
        .position = {10, 0, 0},
        .velocity = {0, 0, 0},
        .acceleration = {0, 0, 0},
    };

    auto targetIntersection = findTargetProjectileIntersection(turretPosition, targetState, 10, 3);

    EXPECT_NEAR(10, targetIntersection.x, 1E-5);
    EXPECT_NEAR(0, targetIntersection.y, 1E-5);
    EXPECT_NEAR(0, targetIntersection.z, 1E-5);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_horizontal_distance_btwn_turret_and_target_target_moving_away_from_turret)
{
    modm::Vector3f turretPosition(0, 0, 0);
    MeasuredKinematicState targetState{
        .position = {10, 0, 0},
        .velocity = {1, 0, 0},
        .acceleration = {0, 0, 0},
    };

    auto targetIntersection = findTargetProjectileIntersection(turretPosition, targetState, 30, 3);

    EXPECT_NEAR(10, targetIntersection.x, 1);
    EXPECT_LT(10, targetIntersection.x);
    EXPECT_NEAR(0, targetIntersection.y, 1E-5);
    EXPECT_NEAR(0, targetIntersection.z, 1E-5);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_horizontal_distance_btwn_turret_and_target_moving_perpendicular_to_turret)
{
    modm::Vector3f turretPosition(0, 0, 0);
    MeasuredKinematicState targetState{
        .position = {10, 0, 0},
        .velocity = {0, 1, 0},
        .acceleration = {0, 0, 0},
    };

    auto targetIntersection = findTargetProjectileIntersection(turretPosition, targetState, 30, 3);

    EXPECT_NEAR(10, targetIntersection.x, 1E-5);
    EXPECT_NEAR(0, targetIntersection.y, 1);
    EXPECT_LT(0, targetIntersection.y);
    EXPECT_NEAR(0, targetIntersection.z, 1E-5);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_horizontal_distance_btwn_turret_and_target_velocity_increases_lead_position_decreases)
{
    modm::Vector3f turretPosition(0, 0, 0);
    MeasuredKinematicState targetState{
        .position = {10, 0, 0},
        .velocity = {0, 1, 0},
        .acceleration = {0, 0, 0},
    };

    auto targetIntersection30ms =
        findTargetProjectileIntersection(turretPosition, targetState, 30, 3);
    auto targetIntersection40ms =
        findTargetProjectileIntersection(turretPosition, targetState, 40, 3);
    auto targetIntersection50ms =
        findTargetProjectileIntersection(turretPosition, targetState, 50, 3);

    EXPECT_GT(targetIntersection30ms.y, targetIntersection40ms.y);
    EXPECT_GT(targetIntersection40ms.y, targetIntersection50ms.y);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_horizontal_distance_btwn_turret_and_target_target_moving_torwards_turet)
{
    modm::Vector3f turretPosition(0, 0, 0);
    MeasuredKinematicState targetState{
        .position = {10, 0, 0},
        .velocity = {-1, 0, 0},
        .acceleration = {0, 0, 0},
    };

    auto targetIntersection = findTargetProjectileIntersection(turretPosition, targetState, 30, 3);

    EXPECT_GT(10, targetIntersection.x);
    EXPECT_NEAR(10, targetIntersection.x, 1);
    EXPECT_NEAR(0, targetIntersection.y, 1E-5);
    EXPECT_NEAR(0, targetIntersection.z, 1E-5);
}

TEST(Ballistics, findTargetProjectileIntersection_target_turret_position_identical)
{
    modm::Vector3f turretPosition(0, 0, 0);
    MeasuredKinematicState targetState{
        .position = {0, 0, 0},
        .velocity = {0, 0, 0},
        .acceleration = {0, 0, 0},
    };

    auto targetIntersection = findTargetProjectileIntersection(turretPosition, targetState, 30, 3);

    EXPECT_NEAR(0, targetIntersection.x, 1E-5);
    EXPECT_NEAR(0, targetIntersection.y, 1E-5);
    EXPECT_NEAR(0, targetIntersection.z, 1E-5);
}

TEST(Ballistics, findTargetProjectileIntersection_target_out_of_range)
{
    modm::Vector3f turretPosition(20, 0, 0);
    MeasuredKinematicState targetState{
        .position = {0, 0, 0},
        .velocity = {0, 0, 0},
        .acceleration = {0, 0, 0},
    };

    auto targetIntersection = findTargetProjectileIntersection(turretPosition, targetState, 1, 3);

    EXPECT_NEAR(0, targetIntersection.x, 1E-5);
    EXPECT_NEAR(0, targetIntersection.y, 1E-5);
    EXPECT_NEAR(0, targetIntersection.z, 1E-5);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_vertical_dist_btwn_turret_and_target_target_moving_away_from_turret)
{
    modm::Vector3f turretPosition(0, 0, 0);
    MeasuredKinematicState targetState{
        .position = {0, 0, 10},
        .velocity = {0, 0, 1},
        .acceleration = {0, 0, 0},
    };

    auto targetIntersection = findTargetProjectileIntersection(turretPosition, targetState, 30, 3);

    EXPECT_NEAR(0, targetIntersection.x, 1E-5);
    EXPECT_NEAR(0, targetIntersection.y, 1E-5);
    EXPECT_GT(10, targetIntersection.z);
    EXPECT_NEAR(10, targetIntersection.z, 1);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_vertical_and_hozirontal_dist_nonzero_turret_position_target_not_moving)
{
    modm::Vector3f turretPosition(0, 1, 2);
    MeasuredKinematicState targetState{
        .position = {turretPosition.x + 3, turretPosition.y - 4, turretPosition.z + 5},
        .velocity = {0, 0, 0},
        .acceleration = {0, 0, 0},
    };

    auto targetIntersection = findTargetProjectileIntersection(turretPosition, targetState, 30, 3);

    EXPECT_NEAR(targetState.position.x, targetIntersection.x, 1E-5);
    EXPECT_NEAR(targetState.position.y, targetIntersection.y, 1E-5);
    EXPECT_NEAR(targetState.position.z, targetIntersection.z, 1E-5);
}

TEST(Ballistics, computePitch_same_turret_target_position)
{
    modm::Vector3f position(1, 2, 3);

    EXPECT_EQ(0, computePitch(position, position));
}

TEST(Ballistics, computePitch_positions_translated_no_angle_diff)
{
    modm::Vector3f turretPosition(1, 2, 3);
    modm::Vector3f targetPosition(43, 42, 41);
    modm::Vector3f translation(-100, 23, 90);

    EXPECT_NEAR(
        computePitch(turretPosition, targetPosition),
        computePitch(turretPosition + translation, targetPosition + translation),
        1E-5);
}

TEST(Ballistics, computePitch_isosceles_triangle)
{
    modm::Vector3f turretPosition(0, 0, 0);
    modm::Vector3f targetPosition(1, 0, 1);

    EXPECT_NEAR(M_PI_4, computePitch(turretPosition, targetPosition), 1E-5);
}

TEST(Ballistics, computePitch_negative_isosceles_triangle)
{
    modm::Vector3f turretPosition(0, 0, 0);
    modm::Vector3f targetPosition(-1, 0, -1);

    EXPECT_NEAR(M_PI_4 - M_PI, computePitch(turretPosition, targetPosition), 1E-5);
}

TEST(Ballistics, computePitch_uses_dist_in_xy_dir)
{
    modm::Vector3f turretPosition(0, 0, 0);
    modm::Vector3f targetPosition(3, 4, 5);

    EXPECT_NEAR(M_PI_4, computePitch(turretPosition, targetPosition), 1E-5);
}

TEST(Ballistics, pitch_special_right_triangle)
{
    modm::Vector3f turretPosition(0, 0, 0);
    modm::Vector3f targetPosition(1, 0, sqrt(3));

    EXPECT_NEAR(modm::toRadian(60), computePitch(turretPosition, targetPosition), 1E-5);
}

TEST(Ballistics, computePitch_multiples_of_pi_div_2)
{
    modm::Vector3f turretPosition(0, 0, 0);

    EXPECT_NEAR(0, computePitch(turretPosition, modm::Vector3f(1, 0, 0)), 1E-5);
    EXPECT_NEAR(M_PI_2, computePitch(turretPosition, modm::Vector3f(0, 0, 1)), 1E-5);
    EXPECT_NEAR(M_PI, computePitch(turretPosition, modm::Vector3f(-1, 0, 0)), 1E-5);
    EXPECT_NEAR(-M_PI_2, computePitch(turretPosition, modm::Vector3f(0, 0, -1)), 1E-5);
}

TEST(Ballistics, computeYaw_same_turret_target_position)
{
    modm::Vector3f position(1, 2, 3);

    EXPECT_EQ(0, computeYaw(position, position));
}

TEST(Ballistics, computeYaw_positions_translated_no_angle_diff)
{
    modm::Vector3f turretPosition(1, 2, 3);
    modm::Vector3f targetPosition(43, 42, 41);
    modm::Vector3f translation(-100, 23, 90);

    EXPECT_NEAR(
        computeYaw(turretPosition, targetPosition),
        computeYaw(turretPosition + translation, targetPosition + translation),
        1E-5);
}

TEST(Ballistics, computeYaw_isosceles_triangle)
{
    modm::Vector3f turretPosition(0, 0, 0);
    modm::Vector3f targetPosition(1, 1, 0);

    EXPECT_NEAR(M_PI_4, computeYaw(turretPosition, targetPosition), 1E-5);
}

TEST(Ballistics, computeYaw_negative_isosceles_triangle)
{
    modm::Vector3f turretPosition(0, 0, 0);
    modm::Vector3f targetPosition(-1, -1, 0);

    EXPECT_NEAR(M_PI_4 - M_PI, computeYaw(turretPosition, targetPosition), 1E-5);
}

TEST(Ballistics, computeYaw_special_right_triangle)
{
    modm::Vector3f turretPosition(0, 0, 0);
    modm::Vector3f targetPosition(1, sqrt(3), 0);

    EXPECT_NEAR(modm::toRadian(60), computeYaw(turretPosition, targetPosition), 1E-5);
}

TEST(Ballistics, computeYaw_multiples_of_pi_div_2)
{
    modm::Vector3f turretPosition(0, 0, 0);

    EXPECT_NEAR(0, computeYaw(turretPosition, modm::Vector3f(1, 0, 0)), 1E-5);
    EXPECT_NEAR(M_PI_2, computeYaw(turretPosition, modm::Vector3f(0, 1, 0)), 1E-5);
    EXPECT_NEAR(M_PI, computeYaw(turretPosition, modm::Vector3f(-1, 0, 0)), 1E-5);
    EXPECT_NEAR(-M_PI_2, computeYaw(turretPosition, modm::Vector3f(0, -1, 0)), 1E-5);
}

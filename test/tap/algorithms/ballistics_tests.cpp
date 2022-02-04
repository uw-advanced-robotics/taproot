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
    modm::Vector3f targetPosition(10, 0, 0);

    float travelTime = computeTravelTime(targetPosition, 20);

    // 20 m/s velocity, 10 m target distance, time should be slightly greater than .5
    EXPECT_LT(1, travelTime);
    EXPECT_NEAR(1, travelTime, 0.3f);
}

TEST(Ballistics, computeTravelTime_vertical_distance_only_bulletVelocity_10)
{
    modm::Vector3f targetPosition(0, 0, 10);

    float travelTime = computeTravelTime(targetPosition, 10);

    EXPECT_NEAR(10, travelTime, 1E-5);
}

TEST(Ballistics, computeTravelTime_bulletVelocity_0_travel_time_0)
{
    modm::Vector3f targetPosition(3, 3, 2);

    float travelTime = computeTravelTime(targetPosition, 0);

    EXPECT_EQ(0, travelTime);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_horizontal_distance_btwn_turret_and_target_target_stationary)
{
    MeasuredKinematicState targetState{
        .position = {10, 0, 0},
        .velocity = {0, 0, 0},
        .acceleration = {0, 0, 0},
    };

    auto targetIntersection = findTargetProjectileIntersection(targetState, 10, 3);

    EXPECT_NEAR(10, targetIntersection.x, 1E-5);
    EXPECT_NEAR(0, targetIntersection.y, 1E-5);
    EXPECT_NEAR(0, targetIntersection.z, 1E-5);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_horizontal_distance_btwn_turret_and_target_target_moving_away_from_turret)
{
    MeasuredKinematicState targetState{
        .position = {10, 0, 0},
        .velocity = {1, 0, 0},
        .acceleration = {0, 0, 0},
    };

    auto targetIntersection = findTargetProjectileIntersection(targetState, 30, 3);

    EXPECT_NEAR(10, targetIntersection.x, 1);
    EXPECT_LT(10, targetIntersection.x);
    EXPECT_NEAR(0, targetIntersection.y, 1E-5);
    EXPECT_NEAR(0, targetIntersection.z, 1E-5);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_horizontal_distance_btwn_turret_and_target_moving_perpendicular_to_turret)
{
    MeasuredKinematicState targetState{
        .position = {10, 0, 0},
        .velocity = {0, 1, 0},
        .acceleration = {0, 0, 0},
    };

    auto targetIntersection = findTargetProjectileIntersection(targetState, 30, 3);

    EXPECT_NEAR(10, targetIntersection.x, 1E-5);
    EXPECT_NEAR(0, targetIntersection.y, 1);
    EXPECT_LT(0, targetIntersection.y);
    EXPECT_NEAR(0, targetIntersection.z, 1E-5);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_horizontal_distance_btwn_turret_and_target_velocity_increases_lead_position_decreases)
{
    MeasuredKinematicState targetState{
        .position = {10, 0, 0},
        .velocity = {0, 1, 0},
        .acceleration = {0, 0, 0},
    };

    auto targetIntersection30ms =
        findTargetProjectileIntersection(targetState, 30, 3);
    auto targetIntersection40ms =
        findTargetProjectileIntersection(targetState, 40, 3);
    auto targetIntersection50ms =
        findTargetProjectileIntersection(targetState, 50, 3);

    EXPECT_GT(targetIntersection30ms.y, targetIntersection40ms.y);
    EXPECT_GT(targetIntersection40ms.y, targetIntersection50ms.y);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_horizontal_distance_btwn_turret_and_target_target_moving_torwards_turet)
{
    MeasuredKinematicState targetState{
        .position = {10, 0, 0},
        .velocity = {-1, 0, 0},
        .acceleration = {0, 0, 0},
    };

    auto targetIntersection = findTargetProjectileIntersection(targetState, 30, 3);

    EXPECT_GT(10, targetIntersection.x);
    EXPECT_NEAR(10, targetIntersection.x, 1);
    EXPECT_NEAR(0, targetIntersection.y, 1E-5);
    EXPECT_NEAR(0, targetIntersection.z, 1E-5);
}

TEST(Ballistics, findTargetProjectileIntersection_target_turret_position_identical)
{
    MeasuredKinematicState targetState{
        .position = {0, 0, 0},
        .velocity = {0, 0, 0},
        .acceleration = {0, 0, 0},
    };

    auto targetIntersection = findTargetProjectileIntersection(targetState, 30, 3);

    EXPECT_NEAR(0, targetIntersection.x, 1E-5);
    EXPECT_NEAR(0, targetIntersection.y, 1E-5);
    EXPECT_NEAR(0, targetIntersection.z, 1E-5);
}

TEST(Ballistics, findTargetProjectileIntersection_target_out_of_range)
{
    MeasuredKinematicState targetState{
        .position = {-20, 0, 0},
        .velocity = {0, 0, 0},
        .acceleration = {0, 0, 0},
    };

    auto targetIntersection = findTargetProjectileIntersection(targetState, 1, 3);

    EXPECT_NEAR(0, targetIntersection.x, 1E-5);
    EXPECT_NEAR(0, targetIntersection.y, 1E-5);
    EXPECT_NEAR(0, targetIntersection.z, 1E-5);
}

TEST(
    Ballistics,
    findTargetProjectileIntersection_only_vertical_dist_btwn_turret_and_target_target_moving_away_from_turret)
{
    MeasuredKinematicState targetState{
        .position = {0, 0, 10},
        .velocity = {0, 0, 1},
        .acceleration = {0, 0, 0},
    };

    auto targetIntersection = findTargetProjectileIntersection(targetState, 30, 3);

    EXPECT_NEAR(0, targetIntersection.x, 1E-5);
    EXPECT_NEAR(0, targetIntersection.y, 1E-5);
    EXPECT_GT(10, targetIntersection.z);
    EXPECT_NEAR(10, targetIntersection.z, 1);
}

TEST(Ballistics, computePitch_same_turret_target_position)
{
    modm::Vector3f position(0, 0, 0);

    EXPECT_EQ(0, computePitch(position));
}

TEST(Ballistics, computePitch_isosceles_triangle)
{
    modm::Vector3f targetPosition(1, 0, 1);

    EXPECT_NEAR(M_PI_4, computePitch(targetPosition), 1E-5);
}

TEST(Ballistics, computePitch_negative_isosceles_triangle)
{
    modm::Vector3f targetPosition(-1, 0, -1);

    EXPECT_NEAR(M_PI_4 - M_PI, computePitch(targetPosition), 1E-5);
}

TEST(Ballistics, computePitch_uses_dist_in_xy_dir)
{
    modm::Vector3f targetPosition(3, 4, 5);

    EXPECT_NEAR(M_PI_4, computePitch(targetPosition), 1E-5);
}

TEST(Ballistics, pitch_special_right_triangle)
{
    modm::Vector3f targetPosition(1, 0, sqrt(3));

    EXPECT_NEAR(modm::toRadian(60), computePitch(targetPosition), 1E-5);
}

TEST(Ballistics, computePitch_multiples_of_pi_div_2)
{
    EXPECT_NEAR(0, computePitch(modm::Vector3f(1, 0, 0)), 1E-5);
    EXPECT_NEAR(M_PI_2, computePitch(modm::Vector3f(0, 0, 1)), 1E-5);
    EXPECT_NEAR(M_PI, computePitch(modm::Vector3f(-1, 0, 0)), 1E-5);
    EXPECT_NEAR(-M_PI_2, computePitch(modm::Vector3f(0, 0, -1)), 1E-5);
}

TEST(Ballistics, computeYaw_same_turret_target_position)
{
    modm::Vector3f position(1, 2, 3);

    EXPECT_EQ(0, computeYaw(position));
}

TEST(Ballistics, computeYaw_isosceles_triangle)
{
    modm::Vector3f targetPosition(1, 1, 0);

    EXPECT_NEAR(M_PI_4, computeYaw(targetPosition), 1E-5);
}

TEST(Ballistics, computeYaw_negative_isosceles_triangle)
{
    modm::Vector3f targetPosition(-1, -1, 0);

    EXPECT_NEAR(M_PI_4 - M_PI, computeYaw(targetPosition), 1E-5);
}

TEST(Ballistics, computeYaw_special_right_triangle)
{
    modm::Vector3f targetPosition(1, sqrt(3), 0);

    EXPECT_NEAR(modm::toRadian(60), computeYaw(targetPosition), 1E-5);
}

TEST(Ballistics, computeYaw_multiples_of_pi_div_2)
{
    EXPECT_NEAR(0, computeYaw(modm::Vector3f(1, 0, 0)), 1E-5);
    EXPECT_NEAR(M_PI_2, computeYaw(modm::Vector3f(0, 1, 0)), 1E-5);
    EXPECT_NEAR(M_PI, computeYaw(modm::Vector3f(-1, 0, 0)), 1E-5);
    EXPECT_NEAR(-M_PI_2, computeYaw(modm::Vector3f(0, -1, 0)), 1E-5);
}

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

#ifndef BALLISTICS_HPP_
#define BALLISTICS_HPP_

#include <cmath>
#include "modm/math.hpp"

namespace tap
{
namespace algorithms
{

// struct Vector3D {
//     float x;
//     float y;
//     float z;
// };

struct MeasuredKinematicState {
    modm::Vector<float, 3> position;
    modm::Vector<float, 3> velocity;
    modm::Vector<float, 3> acceleration;
};

// /**
//  * Doc cmnt
//  */
// class MeasuredKinematicState
// {
//     public:
//     MeasuredKinematicState(float x, float y, float z, float v_x, float v_y, float v_z, float a_x, float a_y, float a_z)
//     : position({x, y, z}),
//       velocity({v_x, v_y, v_z}),
//       acceleration({a_x, a_y, a_z}) {};

//     static float quadraticKinematicProjection(float dt, float x_i, float v_i, float a_i) {
//         return x_i + v_i*dt + 0.5*pow(a_i, 2);
//     }

//     std::array<float> projectKinematicState(float dt) {
//         return {quadraticKinematicProjection(dt, position[0], velocity[0], acceleration[0]),
//                 quadraticKinematicProjection(dt, position[1], velocity[1], acceleration[1]),
//                 quadraticKinematicProjection(dt, position[2], velocity[2], acceleration[2])};
//     }

//     private:
//     float position[3]{};
//     float velocity[3]{};
//     float acceleration[3]{};
// };

inline float quadraticKinematicProjection(float dt, float s, float v, float a) {
    return s + v*dt + 0.5f*a*powf(dt, 2.0f);
}

modm::Vector<float, 3> projectForward(MeasuredKinematicState state, float dt) {
    return modm::Vector<float, 3>({quadraticKinematicProjection(dt, state.position[0], state.velocity[0], state.acceleration[0]),
                                   quadraticKinematicProjection(dt, state.position[1], state.velocity[1], state.acceleration[1]),
                                   quadraticKinematicProjection(dt, state.position[2], state.velocity[2], state.acceleration[2])});
}

}  // namespace algorithms

}  // namespace tap

#endif  // BALLISTICS_HPP_

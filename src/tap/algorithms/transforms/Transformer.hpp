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
#ifndef TAPROOT_TRANSFORMER_HPP_
#define TAPROOT_TRANSFORMER_HPP_

#include <functional>

#include "transform.hpp"

namespace tap::algorithms
{

/**
 * An Interface for a Transformer, which maintains several 
 * transforms.
 * 
 *  A Transformer stores, maintains, and distributes various 
 *  transforms (tap::algorithms::Transform) throughout the lifecycle 
 *  of a robot. 
*/

class Transformer
{
    /**
     * Instantiate a new Transformer
    */
    Transformer();

    // Disable copy constructor and assignment
    Transformer(const TransformsProvider& other) = delete;
    Transformer &operator=(const TransformsProvider& other) = delete;
b
    /**
     * Updates all stored transforms
    */
    virtual void update();

}; // class Transformer
} // namespace tap::algorithms

#endif  // TAPROOT_TRANSFORMER_HPP_
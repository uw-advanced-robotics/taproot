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
#ifndef TAPROOT_TRANSFORMSPROVIDER_HPP_
#define TAPROOT_TRANSFORMSPROVIDER_HPP_

#include <functional>

#include "transform.hpp"
#include "frames.hpp"
#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/algorithms/math_user_utils.hpp"


namespace tap::algorithms
{
/**
 * An Interface for a TransformProvider, providing transforms upon request
 * 
 * 
 *  A TransformProvider stores, maintains, and distributes various 
 *  transforms (tap::algorithms::Transform) throughout the lifecycle 
 *  of a robot. 
*/

class TransformsProvider
{
    /**
     * Instantiate a new TransformsProvider
     * 
    */
    TransformsProvider();

    /**
     * Updates all stored transforms according to their 
     * update functions provided when they called register
    */
    virtual void update();

    // TODO: figure out which types each of these should use
    // TODO: document
    virtual void registerTransform(Transform<Frame,Frame> &initialTransform, 
        std::function<CMSISMat<3,3>()> getNewRotation, 
        std::function<CMSISMat<3,1>()> getNewPosition) = 0;


    virtual Transform<Frame, Frame>&  
}; // class TransformProvider
} // namespace tap::algorithms


#endif  // TAPROOT_TRANSFORMSPROVIDER_HPP_
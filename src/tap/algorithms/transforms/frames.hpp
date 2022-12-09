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

#ifndef TAPROOT_FRAMES_HPP_
#define TAPROOT_FRAMES_HPP_

namespace tap::algorithms
{
/**
 * Frame is an empty class to provide type-checking for
 * generic Transforms. This class is intended to be inherited
 * by more specific frame subclasses, which should also be empty.
 */

class Frame
{
};
}  // namespace tap::algorithms

#endif  // TAPROOT_FRAMES_HPP_

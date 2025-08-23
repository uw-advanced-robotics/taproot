/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef TAPROOT_FRAME_HPP_
#define TAPROOT_FRAME_HPP_

#include <ratio>

namespace tap::units
{
using std::ratio;

template <class a, class b>
struct FrameConversion
{
    using factor = ratio<0, 1>;
};

#define NEW_FRAME(name) \
struct name {}; \
template<> struct FrameConversion<name, name> { \
    using factor = ratio<1,1>; \
};

NEW_FRAME(DefaultFrame)

template <class a, class b>
using factor = FrameConversion<a, b>.factor;

};      // namespace tap::units
#endif  // TAPROOT_FRAME_HPP_
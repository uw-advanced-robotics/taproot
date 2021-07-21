/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruwlib.
 *
 * aruwlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruwlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruwlib.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef DUMMY_ALLOCATOR_HPP_
#define DUMMY_ALLOCATOR_HPP_

#include "modm/utils/allocator/allocator_base.hpp"

namespace aruwlib
{
namespace display
{
template <typename T>
class DummyAllocator : public modm::allocator::AllocatorBase<T>
{
public:
    DummyAllocator() : modm::allocator::AllocatorBase<T>() {}

    DummyAllocator(const DummyAllocator& other) = default;

    T* allocate(size_t) { return nullptr; }

    void deallocate(T*) {}
};  // class DummyAllocator
}  // namespace display
}  // namespace aruwlib

#endif  // DUMMY_ALLOCATOR_HPP_

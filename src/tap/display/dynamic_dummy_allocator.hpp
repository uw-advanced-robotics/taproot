/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_DYNAMIC_DUMMY_ALLOCATOR_HPP_
#define TAPROOT_DYNAMIC_DUMMY_ALLOCATOR_HPP_

#include "modm/utils/allocator/allocator_base.hpp"

namespace tap
{
namespace display
{
template <typename T>
class DynamicDummy : public AllocatorBase<T>
{
public:
    template <typename U>
    struct rebind
    {
        typedef DynamicDummy<U> other;
    };

public:
    DynamicDummy() : AllocatorBase<T>() {}

    DynamicDummy(const Dynamic& other) : AllocatorBase<T>(other) {}

    template <typename U>
    DynamicDummy(const Dynamic<U>&) : AllocatorBase<T>()
    {
    }

    T* allocate(size_t n, bool managed = true)
    {
        isManaged = managed;
        if (!isManaged) return nullptr;

        // allocate the memory without calling the constructor
        // of the associated data-type.
        return static_cast<T*>(::operator new(n * sizeof(T)));
    }

    void deallocate(T* p)
    {
        if (!isManaged) return;

        // it is important to use this form here, otherwise the
        // destructor of p will be called which is unwanted here.
        // The destructor can be called with the destroy()-method.
        ::operator delete(p);
    }

private:
    bool isManaged = true;
};
}  // namespace display
}  // namespace tap

#endif  // TAPROOT_DUMMY_ALLOCATOR_HPP_

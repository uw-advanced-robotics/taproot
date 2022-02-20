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

#ifndef PROFILER_HPP_
#define PROFILER_HPP_

#include <unordered_map>

#include "modm/container.hpp"

#ifdef RUN_WITH_PROFILING
#define PROFILE(profiler, func, params) \
    do                                  \
    {                                   \
        int key = profiler.push(#func); \
        func params;                    \
        profiler.pop(key);              \
    } while (0);
#else
#define PROFILE(profiler, func, params) func params
#endif

namespace tap
{
class Drivers;
}

namespace tap::arch
{
class Profiler
{
public:
    static constexpr std::size_t MAX_PROFILED_ELEMENTS = 128;
    static constexpr float AVG_LOW_PASS_ALPHA = 0.01f;

    struct ProfilerData
    {
        const char* name;
        uint32_t min = UINT32_MAX;
        uint32_t max = 0;
        uint32_t avg = 0;
        uint32_t prevPushedTime = 0;

        void reset()
        {
            min = UINT32_MAX;
            max = 0;
            avg = 0;
        }
    };

    Profiler(tap::Drivers* drivers);

    std::size_t push(const char* profile);

    void pop(std::size_t key);

    inline ProfilerData getData(std::size_t key)
    {
        if (key >= profiledElements.getSize())
        {
            return ProfilerData();
        }
        else
        {
            return profiledElements.get(key);
        }
    }

    inline void reset(std::size_t key)
    {
        if (key < profiledElements.getSize())
        {
            profiledElements[key].reset();
        }
    }

private:
    tap::Drivers* drivers;

    /**
     * Map element names (function names) to index in profiledElements. Don't directly store
     * ProfilerData's in this map to allow for easier accessability of the elements during
     * debugging. std::map on the embedded system is scuffed and doesn't allow for easy access to
     * internal elements.
     */
    std::unordered_map<const char*, std::size_t> elementNameToIndexMap;

    /**
     * Array of profiling data information
     */
    modm::BoundedDeque<ProfilerData, MAX_PROFILED_ELEMENTS> profiledElements;
};

}  // namespace tap::arch

#endif  // PROFILER_HPP_

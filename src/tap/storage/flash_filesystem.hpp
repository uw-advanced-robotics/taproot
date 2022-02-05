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

#ifndef FLASH_FILESYSTEM_HPP_
#define FLASH_FILESYSTEM_HPP_

/**
 * This is a wrapper (driver) for littleFS on internal flash
 * 
 * Plan: 
 * 1. Encapsulate littleFS initialization, mount, buffer
 * 2. Provide Hardcoded read and write for each individual file (Template magic to select files?)
 * 3. It should ONLY be used by file subsystem, NEVER use it directly, especially not when motor is running
 */

namespace tap::storage
{
    class FlashFilesystem
    {
        
    public:
        FlashFilesystem(/* args */);
        int initialize();

        int read();

       private:
    };
    
}

#endif
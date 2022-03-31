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

#include "littlefs_internal.hpp"

#include "modm/platform/flash/flash.hpp"

using namespace tap::storage;
using namespace modm::platform;

LittleFSInternal::LittleFSInternal() {
}

void LittleFSInternal::initialize() {
    Flash::enable();
    Flash::unlock();
}

int LittleFSInternal::lfs_read(const struct lfs_config *c, lfs_block_t block,
                            lfs_off_t off, void *buffer, lfs_size_t size) {
    // Offset and size must be aligned
    if (block >= c->block_count || (size % c->read_size) != 0 || (off % c->read_size) != 0) {
        return LFS_ERR_IO;
    }

    uintptr_t startAddr = OriginAddr + block * c->block_size + off;

    for (size_t i = 0; i < size / sizeof(uint32_t); i++) {
        ((uint32_t *)buffer)[i] = *(reinterpret_cast<uint32_t *>(startAddr) + i);
    }

    return LFS_ERR_OK;
}

int LittleFSInternal::lfs_program(const struct lfs_config *c, lfs_block_t block,
                               lfs_off_t off, const void *buffer, lfs_size_t size) {
    // Offset and size must be aligned
    if (block >= c->block_count || (size % c->prog_size) != 0 || (off % c->prog_size) != 0) {
        return LFS_ERR_IO;
    }

    uintptr_t startAddr = OriginAddr + block * c->block_size + off;

    for (size_t i = 0; i < size / sizeof(uint32_t); i++) {
        if (Flash::program(startAddr + i * sizeof(uint32_t), ((uint32_t *)buffer)[i]) != 0) {
            return LFS_ERR_IO;
        }
    }

    return LFS_ERR_OK;
}

int LittleFSInternal::lfs_erase(const struct lfs_config *c, lfs_block_t block) {
    static constexpr int BANK2_INDEX_OFFSET = 4;
    if (block >= c->block_count) {
        return LFS_ERR_IO;
    }
    return (Flash::erase(SECTOR_ZERO + block + BANK2_INDEX_OFFSET) == 0 ? LFS_ERR_OK : LFS_ERR_IO);
}

int LittleFSInternal::lfs_sync(__unused const struct lfs_config *c) {
    return LFS_ERR_OK;
}

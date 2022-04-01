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

#ifndef LITTLEFS_FLASH_HPP_
#define LITTLEFS_FLASH_HPP_

#include "littlefs/lfs.h"

namespace tap::storage {
class LittleFSInternal {
   public:
    LittleFSInternal();
    void initialize();
    lfs_t *getFS() { return &fs; }
    lfs_config *getFSConfig() { return &fsconfig; }


   private:
    // See RM0090 Page 77
    static constexpr size_t SECTOR_SIZE = 1ul << 17;  // Use 128kB Sectors
    static constexpr uint8_t SECTOR_ZERO = 17;  // 128kB Sector 17 to 23
    static constexpr uint8_t BLOCK_COUNT = 7;  // 17 to 23, 7 blocks in total

    static constexpr int LFS_CACHE_SIZE = 256;
    static constexpr int LFS_LOOKAHEAD_BUFFER_SIZE = 256;
    static constexpr int LFS_MAX_FILE_SIZE = 100 * 1024 * 1024; // 100kB
    static constexpr int LFS_MAX_FILENAME_LENGTH = 64;

    int lfs_read_buffer[LFS_CACHE_SIZE] = {0};
    int lfs_prog_buffer[LFS_CACHE_SIZE] = {0};
    int lfs_lookahead_buffer[LFS_LOOKAHEAD_BUFFER_SIZE] = {0};

    static constexpr uintptr_t OriginAddr{0x08120000};  // 128KiB Sector 17
    static inline uint8_t *const Origin{(uint8_t *)OriginAddr};

    lfs_t fs;

    lfs_config fsconfig = {
        .context = 0,
        .read = lfs_read,
        .prog = lfs_program,
        .erase = lfs_erase,
        .sync = lfs_sync,

        // block device configuration
        .read_size = 4,
        .prog_size = 4,
        .block_size = SECTOR_SIZE,  // 128KiB
        .block_count = BLOCK_COUNT,
        .block_cycles = 100,
        .cache_size = LFS_CACHE_SIZE,
        .lookahead_size = LFS_LOOKAHEAD_BUFFER_SIZE,
        .read_buffer = static_cast<void *>(lfs_read_buffer),
        .prog_buffer = static_cast<void *>(lfs_prog_buffer),
        .lookahead_buffer = static_cast<void *>(lfs_lookahead_buffer),
        .name_max = LFS_MAX_FILENAME_LENGTH,
        .file_max = LFS_MAX_FILE_SIZE,
        .attr_max = 0,
        .metadata_max = 2048,
    };

    static int lfs_read(const struct lfs_config *c, lfs_block_t block,
                        lfs_off_t off, void *buffer, lfs_size_t size);

    static int lfs_program(const struct lfs_config *c, lfs_block_t block,
                           lfs_off_t off, const void *buffer, lfs_size_t size);

    static int lfs_erase(const struct lfs_config *c, lfs_block_t block);

    static int lfs_sync(const struct lfs_config *c);

};

}  // namespace tap::storage

#endif

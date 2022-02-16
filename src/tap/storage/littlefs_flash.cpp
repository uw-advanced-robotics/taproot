#include "littlefs_flash.hpp"

#include "modm/architecture/interface/register.hpp"
#include "modm/platform/flash/flash.hpp"

using namespace tap::storage;
using namespace modm::platform;

LittleFSFlash::LittleFSFlash() {
}

int LittleFSFlash::initialize() {
    Flash::enable();
    for (size_t i = 0; i < 10 && Flash::isLocked(); i++) {
        Flash::unlock();
    }
    if (Flash::isLocked()) {
        return -1;
    }

    return 0;
}

int LittleFSFlash::lfs_read(const struct lfs_config *c, lfs_block_t block,
                            lfs_off_t off, void *buffer, lfs_size_t size) {
    if (block >= c->block_count) {
        return LFS_ERR_IO;
    }

    uintptr_t startAddr = OriginAddr + block * c->block_size + off;
    uintptr_t endAddr = OriginAddr + block * c->block_size + off + size;
    uintptr_t alignedEndAddr = (endAddr >> 2) << 2;
    uintptr_t alignedSize = (size >> 2) << 2;

    for (size_t i = 0; i < size / sizeof(uint32_t); i++) {
        ((uint32_t *)buffer)[i] = *(reinterpret_cast<uint32_t *>(startAddr) + i);
    }
    for (size_t i = 0; i < size % sizeof(uint32_t); i++) {
        ((uint8_t *)buffer)[alignedSize + i] = *(reinterpret_cast<uint8_t *>(alignedEndAddr) + i);
    }

    return LFS_ERR_OK;
}

int LittleFSFlash::lfs_program(const struct lfs_config *c, lfs_block_t block,
                               lfs_off_t off, const void *buffer, lfs_size_t size) {
    if (block >= c->block_count) {
        return LFS_ERR_IO;
    }

    uintptr_t startAddr = OriginAddr + block * c->block_size + off;
    uintptr_t endAddr = OriginAddr + block * c->block_size + off + size;
    uintptr_t alignedEndAddr = (endAddr / sizeof(uint32_t)) * sizeof(uint32_t);
    uintptr_t alignedSize = (size / sizeof(uint32_t)) * sizeof(uint32_t);

    for (size_t i = 0; i < size / sizeof(uint32_t); i++) {
        if (Flash::program(startAddr + i * sizeof(uint32_t), ((uint32_t *)buffer)[i]) != 0) {
            return LFS_ERR_IO;
        }
    }

    for (size_t i = 0; i < size % sizeof(uint32_t); i++) {
        if (Flash::program(alignedEndAddr + i * sizeof(uint8_t), ((uint8_t *)buffer)[alignedSize + i], Flash::WordSize::B8) != 0) {
            return LFS_ERR_IO;
        }
    }

    return LFS_ERR_OK;
}

int LittleFSFlash::lfs_erase(const struct lfs_config *c, lfs_block_t block) {
    static constexpr int BANK2_INDEX_OFFSET = 4;
    if (block >= c->block_count) {
        return LFS_ERR_IO;
    }
    return (Flash::erase(SECTOR_ZERO + block + BANK2_INDEX_OFFSET) == 0 ? LFS_ERR_OK : LFS_ERR_IO);
}

int LittleFSFlash::lfs_sync(__unused const struct lfs_config *c) {
    return LFS_ERR_OK;
}

int LittleFSFlash::mount() {
    return lfs_mount(&fs, &fsconfig);
}

int LittleFSFlash::format() {
    return lfs_format(&fs, &fsconfig);
}

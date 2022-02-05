#include "littlefs_flash.hpp"

#include "modm/platform/flash/flash.hpp"

using namespace tap::storage;
using namespace modm::platform;

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
    
    uint32_t *addr = reinterpret_cast<uint32_t *>(Origin + block * c->block_size + off);

    for (size_t i = 0; i < size / sizeof(uint32_t); i++) {
        ((uint32_t *)buffer)[i] = *(addr + i);
    }
    return LFS_ERR_OK;
}

int LittleFSFlash::lfs_program(const struct lfs_config *c, lfs_block_t block,
                               lfs_off_t off, const void *buffer, lfs_size_t size) {
    if (block >= c->block_count) {
        return LFS_ERR_IO;
    }

    uintptr_t addr = OriginAddr + block * c->block_size + off;

    for (size_t i = 0; i < size / sizeof(uint32_t); i++) {
        if (Flash::program(addr, ((uint32_t *)buffer)[i]) != 0) {
            return LFS_ERR_IO;
        }
    }
    return LFS_ERR_OK;
}

int LittleFSFlash::lfs_erase(const struct lfs_config *c, lfs_block_t block) {
    if (block >= c->block_count) {
        return LFS_ERR_IO;
    }
    return (Flash::erase(SECTOR_ZERO + block) == 0 ? 0 : LFS_ERR_IO);
}

int LittleFSFlash::lfs_sync(__unused const struct lfs_config *c) {
    return LFS_ERR_OK;
}

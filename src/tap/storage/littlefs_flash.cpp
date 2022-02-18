#include "littlefs_flash.hpp"

#include "modm/architecture/interface/register.hpp"
#include "modm/platform/flash/flash.hpp"

using namespace tap::storage;
using namespace modm::platform;

LittleFSFlash::LittleFSFlash() {
}

void LittleFSFlash::initialize() {
    Flash::enable();
    Flash::unlock();
}

int LittleFSFlash::lfs_read(const struct lfs_config *c, lfs_block_t block,
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

int LittleFSFlash::lfs_program(const struct lfs_config *c, lfs_block_t block,
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

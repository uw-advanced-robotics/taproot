#ifndef IST8310_HPP_
#define IST8310_HPP_

#include "tap/rm-dev-board-a/board.hpp"

namespace tap
{
namespace sensors
{
class Mpu6500;

class Ist8310
{
public:
    Ist8310(Mpu6500 *mpu) : mpu(mpu) {}

    struct Ist
    {
        int16_t mx;
        int16_t my;
        int16_t mz;

        int16_t mxOffset;
        int16_t myOffset;
        int16_t mzOffset;
    };

    void init(void);

    void istRegWriteByMpu(uint8_t addr, uint8_t data);

    uint8_t istRegReadByMpu(uint8_t addr);

    void mpuMstI2cAutoReadConfig(
        uint8_t device_address,
        uint8_t reg_base_addr,
        uint8_t data_num);

    uint8_t ist8310Init(void);

    void ist8310GetData(void);

    // this function takes 24.6us.(42M spi)
    void mpu_get_data(struct ahrs_sensor *sensor);

    void getIstMagOffset(void);

    bool caliIst;

    Ist ist;

    Mpu6500 *mpu;

};

}  // namespace sensors

}  // namespace tap

#endif  // IST8310_HPP_

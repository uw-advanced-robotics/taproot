/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "ist8310.hpp"
#include "ist8310_reg.hpp"
#include "mpu6500.hpp"
#include "mpu6500_reg.hpp"

#define MPU_IO_PROBE() //HAL_GPIO_TogglePin(IO_PROBE_GPIO_Port, IO_PROBE_Pin);

namespace tap
{

namespace sensors
{
    void Ist8310::init()
    {
        Board::IstEnablePin::setOutput();
        Board::IstEnablePin::setOutput(modm::GpioOutput::High);
        ist8310Init();
    }

    void Ist8310::istRegWriteByMpu(uint8_t addr, uint8_t data)
    {
        //turn off slave 1 at first
        mpu->spiWriteRegister(MPU6500_I2C_SLV1_CTRL, 0x00);
        modm::delay_ms(2);
        mpu->spiWriteRegister(MPU6500_I2C_SLV1_REG, addr);
        modm::delay_ms(2);
        mpu->spiWriteRegister(MPU6500_I2C_SLV1_DO, data);
        modm::delay_ms(2);
        //turn on slave 1 with one byte transmitting
        mpu->spiWriteRegister(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
        modm::delay_ms(10);
    }

    uint8_t Ist8310::istRegReadByMpu(uint8_t addr)
    {
        uint8_t retval;
        mpu->spiWriteRegister(MPU6500_I2C_SLV4_REG, addr);
        modm::delay_ms(10);
        mpu->spiWriteRegister(MPU6500_I2C_SLV4_CTRL, 0x80);
        modm::delay_ms(10);
        retval = mpu->spiReadRegister(MPU6500_I2C_SLV4_DI);
        //turn off slave4 after read
        mpu->spiWriteRegister(MPU6500_I2C_SLV4_CTRL, 0x00);
        modm::delay_ms(10);
        return retval;
    }

    void Ist8310::mpuMstI2cAutoReadConfig(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
    {
        //configure the device address of the IST8310
        //use slave1,auto transmit single measure mode.
        mpu->spiWriteRegister(MPU6500_I2C_SLV1_ADDR, device_address);
        modm::delay_ms(2);
        mpu->spiWriteRegister(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
        modm::delay_ms(2);
        mpu->spiWriteRegister(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
        modm::delay_ms(2);

        //use slave0,auto read data
        mpu->spiWriteRegister(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
        modm::delay_ms(2);
        mpu->spiWriteRegister(MPU6500_I2C_SLV0_REG, reg_base_addr);
        modm::delay_ms(2);

        //every eight mpu6500 internal samples one i2c master read
        mpu->spiWriteRegister(MPU6500_I2C_SLV4_CTRL, 0x03);
        modm::delay_ms(2);
        //enable slave 0 and 1 access delay
        mpu->spiWriteRegister(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
        modm::delay_ms(2);
        //enable slave 1 auto transmit
        mpu->spiWriteRegister(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
        modm::delay_ms(6); //Wait 6ms (minimum waiting time for 16 times internal average setup)
        //enable slave 0 with data_num bytes reading
        mpu->spiWriteRegister(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
        modm::delay_ms(2);
    }

    uint8_t Ist8310::ist8310Init(void)
    {
        //Enable I2C master mode, Reset I2C Slave module
        mpu->spiWriteRegister(MPU6500_USER_CTRL, 0x30);
        modm::delay_ms(10);
        //I2C master clock 400kHz
        mpu->spiWriteRegister(MPU6500_I2C_MST_CTRL, 0x0d);
        modm::delay_ms(10);

        //turn on slave 1 for ist write and slave 4 for ist read
        mpu->spiWriteRegister(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS); //write ist
        modm::delay_ms(10);
        mpu->spiWriteRegister(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS); //read ist
        modm::delay_ms(10);

        //reset ist8310
        istRegWriteByMpu(IST8310_R_CONFB, 0x01);
        modm::delay_ms(10);

        if (IST8310_DEVICE_ID_A != istRegReadByMpu(IST8310_WHO_AM_I))
            return 1;

        istRegWriteByMpu(IST8310_R_CONFB, 0x01);
        modm::delay_ms(10);

        //config as ready mode to access reg
        istRegWriteByMpu(IST8310_R_CONFA, 0x00);
        if (istRegReadByMpu(IST8310_R_CONFA) != 0x00)
            return 2;
        modm::delay_ms(10);

        //normal state, no int
        istRegWriteByMpu(IST8310_R_CONFB, 0x00);
        if (istRegReadByMpu(IST8310_R_CONFB) != 0x00)
            return 3;
        modm::delay_ms(10);

        //config low noise mode, x,y,z axis 16 time 1 avg,
            istRegWriteByMpu(IST8310_AVGCNTL, 0x24); //100100
            if (istRegReadByMpu(IST8310_AVGCNTL) != 0x24)
                    return 4;
        modm::delay_ms(10);

        //Set/Reset pulse duration setup, normal mode
        istRegWriteByMpu(IST8310_PDCNTL, 0xc0);
        if (istRegReadByMpu(IST8310_PDCNTL) != 0xc0)
            return 5;
        modm::delay_ms(10);

        //turn off slave1 & slave 4
        mpu->spiWriteRegister(MPU6500_I2C_SLV1_CTRL, 0x00);
        modm::delay_ms(10);
        mpu->spiWriteRegister(MPU6500_I2C_SLV4_CTRL, 0x00);
        modm::delay_ms(10);

        //configure and turn on slave 0
        mpuMstI2cAutoReadConfig(IST8310_ADDRESS, IST8310_R_XL, 0x06);
        modm::delay_ms(100);
        return 0;
    }

    void Ist8310::ist8310GetData()
    {
        uint8_t magData[6];
        mpu->spiReadRegisters(MPU6500_EXT_SENS_DATA_00, magData, 6);
        ist.mx = (magData[1] << 8 | magData[0]) - ist.mxOffset;
        ist.my = -((magData[3] << 8 | magData[2]) - ist.myOffset);
        ist.mz = -((magData[5] << 8 | magData[4]) - ist.mzOffset);
    }

    void Ist8310::getIstMagOffset(void)
    {
        // int16_t mag_max[3], mag_min[3];
        for (int i = 0; i < 5000; i++)
        { 
            // ist8310GetData((uint8_t *)&mpu_data.mx);
            // if ((abs(mpu_data.mx) < 400) && (abs(mpu_data.my) < 400) && (abs(mpu_data.mz) < 400))
            // {
            //     mag_max[0] = VAL_MAX(mag_max[0], mpu_data.mx);
            //     mag_min[0] = VAL_MIN(mag_min[0], mpu_data.mx);

            //     mag_max[1] = VAL_MAX(mag_max[1], mpu_data.my);
            //     mag_min[1] = VAL_MIN(mag_min[1], mpu_data.my);

            //     mag_max[2] = VAL_MAX(mag_max[2], mpu_data.mz);
            //     mag_min[2] = VAL_MIN(mag_min[2], mpu_data.mz);
            // }
            modm::delay_ms(2);
        }
        // mpu_data.mx_offset = (int16_t)((mag_max[0] + mag_min[0]) * 0.5f);
        // mpu_data.my_offset = (int16_t)((mag_max[1] + mag_min[1]) * 0.5f);
        // mpu_data.mz_offset = (int16_t)((mag_max[2] + mag_min[2]) * 0.5f);

        caliIst = 0;
    }
}  // namespace sensors

}  // namspace tap

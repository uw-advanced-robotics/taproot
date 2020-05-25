#ifndef CRC_HPP_
#define CRC_HPP_

#include <stdint.h>

namespace aruwlib
{

namespace algorithms
{

#define CRC8_INIT 0xff
#define CRC16_INIT 0xffff

/**
 * Fast crc8 calculation using a lookup table. The crc looks at messageLength
 * bytes in the message to calculate the crc.
 *
 * @param[in] message the message to be used for calculation.
 * @param[in] messageLength the number of bytes to look at when calculating the crc.
 * @param[in] initCRC8 normally leave as CRC8_INIT.
 * @return the calculated crc.
 */
uint8_t calculateCRC8(const uint8_t *message,
                      uint32_t messageLength,
                      uint8_t initCRC8 = CRC8_INIT);

/**
 * Fast crc16 calculation using a lookup table.
 *
 * @see calculateCRC8
 *
 * @param[in] message the message to be used for calculation.
 * @param[in] messageLength the number of bytes to look at when calculating the crc.
 * @param[in] initCRC8 normally leave as CRC8_INIT.
 * @return the calculated crc.
 */
uint16_t calculateCRC16(const uint8_t *message,
                        uint32_t messageLength,
                        uint16_t initCRC16 = CRC16_INIT);

}  // namespace algorithms

}  // namespace aruwlib

#endif  // CRC_HPP_

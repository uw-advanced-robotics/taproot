#ifndef __crc_h_
#define __crc_h_

#include <stdint.h>

namespace aruwlib
{

namespace algorithms
{

#define CRC8_INIT 0xff
#define CRC16_INIT 0xffff

uint8_t calculateCRC8(const uint8_t *message, uint32_t messageLength, uint8_t initCRC8);
uint16_t calculateCRC16(const uint8_t *message, uint32_t messageLength, uint16_t initCRC16);

}  // namespace algorithms

}  // namespace aruwlib

#endif

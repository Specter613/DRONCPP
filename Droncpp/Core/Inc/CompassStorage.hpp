/*
 * CompassStorage.hpp
 *
 *  Created on: 14 jul 2026
 *      Author: specter0163
 */

#ifndef INC_COMPASSSTORAGE_HPP_
#define INC_COMPASSSTORAGE_HPP_

#include <cstdint>
#include "stm32f4xx_hal.h"

struct MagCalibData
{
    uint32_t magic;
    float offsetX, offsetY, offsetZ;
};

class MagCalibrationStorage
{
public:
    static constexpr uint32_t MAGIC = 0xCAFEBABEUL;
    static constexpr uint32_t FLASH_ADDR = 0x080E0000;
    static constexpr uint32_t FLASH_SECTOR = FLASH_SECTOR_11;

    static void Save(float offsetX, float offsetY, float offsetZ);
    static bool Load(float &offsetX, float &offsetY, float &offsetZ);
};

#endif /* INC_COMPASSSTORAGE_HPP_ */

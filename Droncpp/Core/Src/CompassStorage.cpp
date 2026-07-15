/*
 * CompassStorage.cpp
 *
 *  Created on: 14 jul 2026
 *      Author: specter0163
 */
// mag_calibration_storage.cpp
#include "CompassStorage.hpp"

void MagCalibrationStorage::Save(float offsetX, float offsetY, float offsetZ)
{
    MagCalibData data = { MAGIC, offsetX, offsetY, offsetZ };

    __disable_irq();
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {};
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = FLASH_SECTOR;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    uint32_t error;
    HAL_FLASHEx_Erase(&erase, &error);

    const uint32_t *src = reinterpret_cast<const uint32_t*>(&data);
    for(uint32_t i = 0; i < sizeof(data) / 4; i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ADDR + i * 4, src[i]);
    }

    HAL_FLASH_Lock();
    __enable_irq();
}

bool MagCalibrationStorage::Load(float &offsetX, float &offsetY, float &offsetZ)
{
    const auto *data = reinterpret_cast<const MagCalibData*>(FLASH_ADDR);
    if(data->magic != MAGIC) return false;

    offsetX = data->offsetX;
    offsetY = data->offsetY;
    offsetZ = data->offsetZ;
    return true;
}




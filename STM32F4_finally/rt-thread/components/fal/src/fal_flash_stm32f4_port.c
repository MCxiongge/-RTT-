/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-01-26     armink       the first version
 */

#include <fal.h>

#include <stm32f4xx.h>

#define DBG_TAG "fal_flash_port"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* base address of the flash sectors */
#define ADDR_FLASH_SECTOR_0      ((uint32_t)0x08000000) /* Base address of Sector 0, 16 K bytes   */
#define ADDR_FLASH_SECTOR_1      ((uint32_t)0x08004000) /* Base address of Sector 1, 16 K bytes   */
#define ADDR_FLASH_SECTOR_2      ((uint32_t)0x08008000) /* Base address of Sector 2, 16 K bytes   */
#define ADDR_FLASH_SECTOR_3      ((uint32_t)0x0800C000) /* Base address of Sector 3, 16 K bytes   */
#define ADDR_FLASH_SECTOR_4      ((uint32_t)0x08010000) /* Base address of Sector 4, 64 K bytes   */
#define ADDR_FLASH_SECTOR_5      ((uint32_t)0x08020000) /* Base address of Sector 5, 128 K bytes  */
#define ADDR_FLASH_SECTOR_6      ((uint32_t)0x08040000) /* Base address of Sector 6, 128 K bytes  */
#define ADDR_FLASH_SECTOR_7      ((uint32_t)0x08060000) /* Base address of Sector 7, 128 K bytes  */
#define ADDR_FLASH_SECTOR_8      ((uint32_t)0x08080000) /* Base address of Sector 8, 128 K bytes  */
#define ADDR_FLASH_SECTOR_9      ((uint32_t)0x080A0000) /* Base address of Sector 9, 128 K bytes  */
#define ADDR_FLASH_SECTOR_10     ((uint32_t)0x080C0000) /* Base address of Sector 10, 128 K bytes */
#define ADDR_FLASH_SECTOR_11     ((uint32_t)0x080E0000) /* Base address of Sector 11, 128 K bytes */

//#define ADDR_FLASH_SECTOR_12      ((uint32_t)0x08100000) /* Base address of Sector 0, 16 K bytes   */
//#define ADDR_FLASH_SECTOR_13      ((uint32_t)0x08104000) /* Base address of Sector 1, 16 K bytes   */
//#define ADDR_FLASH_SECTOR_14      ((uint32_t)0x08108000) /* Base address of Sector 2, 16 K bytes   */
//#define ADDR_FLASH_SECTOR_15      ((uint32_t)0x0810C000) /* Base address of Sector 3, 16 K bytes   */
//#define ADDR_FLASH_SECTOR_16      ((uint32_t)0x08110000) /* Base address of Sector 4, 64 K bytes   */
//#define ADDR_FLASH_SECTOR_17      ((uint32_t)0x08120000) /* Base address of Sector 5, 128 K bytes  */
//#define ADDR_FLASH_SECTOR_18      ((uint32_t)0x08140000) /* Base address of Sector 6, 128 K bytes  */
//#define ADDR_FLASH_SECTOR_19      ((uint32_t)0x08160000) /* Base address of Sector 7, 128 K bytes  */
//#define ADDR_FLASH_SECTOR_20      ((uint32_t)0x08180000) /* Base address of Sector 8, 128 K bytes  */
//#define ADDR_FLASH_SECTOR_21      ((uint32_t)0x081A0000) /* Base address of Sector 9, 128 K bytes  */
//#define ADDR_FLASH_SECTOR_22      ((uint32_t)0x081C0000) /* Base address of Sector 10, 128 K bytes */
//#define ADDR_FLASH_SECTOR_23      ((uint32_t)0x081E0000) /* Base address of Sector 11, 128 K bytes */

/**
 * Get the sector of a given address
 *
 * @param address flash address
 *
 * @return The sector of a given address
 */
static uint32_t stm32_get_sector(uint32_t address)
{
    uint32_t sector = 0;

    if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
    {
        sector = FLASH_SECTOR_0;
    }
    else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
    {
        sector = FLASH_SECTOR_1;
    }
    else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
    {
        sector = FLASH_SECTOR_2;
    }
    else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
    {
        sector = FLASH_SECTOR_3;
    }
    else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
    {
        sector = FLASH_SECTOR_4;
    }
    else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
    {
        sector = FLASH_SECTOR_5;
    }
    else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6))
    {
        sector = FLASH_SECTOR_6;
    }
    else if ((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7))
    {
        sector = FLASH_SECTOR_7;
    }
    else if ((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8))
    {
        sector = FLASH_SECTOR_8;
    }
    else if ((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9))
    {
        sector = FLASH_SECTOR_9;
    }
    else if ((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10))
    {
        sector = FLASH_SECTOR_10;
    }
    else{
        sector = FLASH_SECTOR_11;
        }
//    else if ((address < ADDR_FLASH_SECTOR_12) && (address >= ADDR_FLASH_SECTOR_11)){
//        sector = FLASH_SECTOR_11;
//    }
//    else if ((address < ADDR_FLASH_SECTOR_13) && (address >= ADDR_FLASH_SECTOR_12))
//    {
//        sector = FLASH_SECTOR_12;
//    }
//    else if ((address < ADDR_FLASH_SECTOR_14) && (address >= ADDR_FLASH_SECTOR_13))
//    {
//        sector = FLASH_SECTOR_13;
//    }
//    else if ((address < ADDR_FLASH_SECTOR_15) && (address >= ADDR_FLASH_SECTOR_14))
//    {
//        sector = FLASH_SECTOR_14;
//    }
//    else if ((address < ADDR_FLASH_SECTOR_16) && (address >= ADDR_FLASH_SECTOR_15))
//    {
//        sector = FLASH_SECTOR_15;
//    }
//    else if ((address < ADDR_FLASH_SECTOR_17) && (address >= ADDR_FLASH_SECTOR_16))
//    {
//        sector = FLASH_SECTOR_16;
//    }
//    else if ((address < ADDR_FLASH_SECTOR_18) && (address >= ADDR_FLASH_SECTOR_17))
//    {
//        sector = FLASH_SECTOR_17;
//    }
//    else if ((address < ADDR_FLASH_SECTOR_19) && (address >= ADDR_FLASH_SECTOR_18))
//    {
//        sector = FLASH_SECTOR_18;
//    }
//    else if ((address < ADDR_FLASH_SECTOR_20) && (address >= ADDR_FLASH_SECTOR_19))
//    {
//        sector = FLASH_SECTOR_19;
//    }
//    else if ((address < ADDR_FLASH_SECTOR_21) && (address >= ADDR_FLASH_SECTOR_20))
//    {
//        sector = FLASH_SECTOR_20;
//    }else if ((address < ADDR_FLASH_SECTOR_22) && (address >= ADDR_FLASH_SECTOR_21))
//    {
//        sector = FLASH_SECTOR_21;
//    }
//    else if ((address < ADDR_FLASH_SECTOR_23) && (address >= ADDR_FLASH_SECTOR_22))
//    {
//        sector = FLASH_SECTOR_22;
//    }else {
//        sector = FLASH_SECTOR_23;
//    }

    return sector;
}

/**
 * Get the sector size
 *
 * @param sector sector
 *
 * @return sector size
 */
static uint32_t stm32_get_sector_size(uint32_t sector) {
    assert(IS_FLASH_SECTOR(sector));

    switch (sector) {
    case FLASH_SECTOR_0: return 16 * 1024;
    case FLASH_SECTOR_1: return 16 * 1024;
    case FLASH_SECTOR_2: return 16 * 1024;
    case FLASH_SECTOR_3: return 16 * 1024;
    case FLASH_SECTOR_4: return 64 * 1024;
    case FLASH_SECTOR_5: return 128 * 1024;
    case FLASH_SECTOR_6: return 128 * 1024;
    case FLASH_SECTOR_7: return 128 * 1024;
    case FLASH_SECTOR_8: return 128 * 1024;
    case FLASH_SECTOR_9: return 128 * 1024;
    case FLASH_SECTOR_10: return 128 * 1024;
    case FLASH_SECTOR_11: return 128 * 1024;

//    case FLASH_SECTOR_12: return 16 * 1024;
//    case FLASH_SECTOR_13: return 16 * 1024;
//    case FLASH_SECTOR_14: return 16 * 1024;
//    case FLASH_SECTOR_15: return 16 * 1024;
//    case FLASH_SECTOR_16: return 64 * 1024;
//    case FLASH_SECTOR_17: return 128 * 1024;
//    case FLASH_SECTOR_18: return 128 * 1024;
//    case FLASH_SECTOR_19: return 128 * 1024;
//    case FLASH_SECTOR_20: return 128 * 1024;
//    case FLASH_SECTOR_21: return 128 * 1024;
//    case FLASH_SECTOR_22: return 128 * 1024;
//    case FLASH_SECTOR_23: return 128 * 1024;

    default : return 128 * 1024;
    }
}
static int init(void)
{
    /* do nothing now */
}

static int read(long offset, uint8_t *buf, size_t size)
{
    size_t i;
    uint32_t addr = stm32f4_onchip_flash.addr + offset;

    if ((addr + size) > STM32_FLASH_END_ADDRESS)
    {
        LOG_E("read outrange flash size! addr is (0x%p)", (void*)(addr + size));
        return -1;
    }

    for (i = 0; i < size; i++, addr++, buf++)
    {
        *buf = *(uint8_t *) addr;
    }

    return size;
}

static int write(long offset, const uint8_t *buf, size_t size)
{
    size_t i;
    rt_err_t result      = RT_EOK;
    uint32_t read_data;
    uint32_t addr = stm32f4_onchip_flash.addr + offset;
    uint32_t end_addr = addr + size;

    if ((end_addr) > STM32_FLASH_END_ADDRESS)
    {
        LOG_E("write outrange flash size! addr is (0x%p)", (void*)(addr + size));
        return -RT_EINVAL;
    }

    if (size < 1)
    {
        return -RT_EINVAL;
    }

    HAL_FLASH_Unlock();

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    for (size_t i = 0; i < size; i++, addr++, buf++)
    {
        /* write data to flash */
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr, (rt_uint64_t)(*buf)) == HAL_OK)
        {
            if (*(rt_uint8_t *)addr != *buf)
            {
                result = -RT_ERROR;
                break;
            }
        }
        else
        {
            result = -RT_ERROR;
            break;
        }
    }

    HAL_FLASH_Lock();

    if (result != RT_EOK)
    {
        return result;
    }

    return size;
}

static int erase(long offset, size_t size)
{
    uint32_t addr = stm32f4_onchip_flash.addr + offset;

    rt_err_t result = RT_EOK;
    rt_uint32_t FirstSector = 0, NbOfSectors = 0;
    rt_uint32_t SECTORError = 0;

    if ((addr + size) > STM32_FLASH_END_ADDRESS)
    {
        LOG_E("ERROR: erase outrange flash size! addr is (0x%p)\n", (void*)(addr + size));
        return -RT_EINVAL;
    }

    /*Variable used for Erase procedure*/
    FLASH_EraseInitTypeDef EraseInitStruct;

    /* Unlock the Flash to enable the flash control register access */
    HAL_FLASH_Unlock();

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    /* Get the 1st sector to erase */
    FirstSector = stm32_get_sector(addr);
    /* Get the number of sector to erase from 1st sector*/
    NbOfSectors = stm32_get_sector(addr + size - 1) - FirstSector + 1;
    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector        = FirstSector;
    EraseInitStruct.NbSectors     = NbOfSectors;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, (uint32_t *)&SECTORError) != HAL_OK)
    {
        result = -RT_ERROR;
        goto __exit;
    }

__exit:
    HAL_FLASH_Lock();

    if (result != RT_EOK)
    {
        return result;
    }

    LOG_D("erase done: addr (0x%p), size %d", (void*)addr, size);
    return size;

}

static int stm32_flash_init(void)
{
    /* doing nothing now */
}

const struct fal_flash_dev stm32f4_onchip_flash =
{
    .name       = "stm32f4_onchip",
    .addr       = 0x08000000,  // Flash start address
    .len        = 512*1024, //stm32f429bit6 2M Flash
    .blk_size   = 128*1024,    //largest sector size
    //.ops        = {stm32_flash_init, stm32_flash_read, stm32_flash_write, stm32_flash_erase},
    .ops        = {stm32_flash_init, read, write, erase},
    .write_gran = 8
};


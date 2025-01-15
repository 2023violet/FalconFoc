#ifndef __FLASH_DRIVER_H__
#define __FLASH_DRIVER_H__

#include "general_def.h"
#include "bsp_flash.h"

uint8_t flash_erase_page(uint32_t page);
uint8_t flash_erase_pages(uint32_t start_add, uint32_t end_add);
void flash_write_data(uint32_t addr, void *data, uint32_t size);
void flash_read_data(uint32_t addr, uint32_t *data, uint32_t size);

#endif /* __FLASH_DRIVER_H__ */

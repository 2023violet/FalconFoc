#include "flash_driver.h"
/**
***********************************************************************
* @brief:      flash_erase_page(uint8_t page)
* @param:		   page：flash页面
* @retval:     i 0：失败 1：成功
* @details:    按页面擦除，擦单页
***********************************************************************
**/
uint8_t flash_erase_page(uint32_t page)
{
	uint8_t i;
	i = flash_bsp_erase_page(page);
	return i;
}
/**
***********************************************************************
* @brief:      flash_erase_pages(uint32_t start_addr, uint32_t end_addr)
* @param:		   start_addr：flash首地址
* @param:		   end_addr：flash尾地址
* @retval:     i 0：失败 1：成功
* @details:    按页面地址擦除，擦多页
***********************************************************************
**/
uint8_t flash_erase_pages(uint32_t start_add, uint32_t end_add)
{
	uint8_t i;
	i = flash_bsp_erase_pages(start_add, end_add);
	return i;
}
/**
***********************************************************************
* @brief:      flash_write_data(uint32_t addr, void *data, uint32_t size)
* @param:		   addr：flash 地址
* @param:		   data：写入的数据
* @param:		   size：写入的数据的字节数
* @retval:     void
* @details:    向flash中存入数据
***********************************************************************
**/
void flash_write_data(uint32_t addr, void *data, uint32_t size)
{
	flash_bsp_write_data(addr, data, size);
}
/**
***********************************************************************
* @brief:      flash_read_data(uint32_t addr, void *data, uint32_t size)
* @param:		   addr：flash 地址
* @param:		   data：读取的数据
* @param:		   size：读取的数据字节数
* @retval:     void
* @details:    从flash中读取数据
***********************************************************************
**/
void flash_read_data(uint32_t addr, uint32_t *data, uint32_t size)
{
	memcpy(data, (uint32_t *)addr, size);
}

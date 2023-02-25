#include "data_logger.h"
#include "ringfs.h"
#include "flash_interface.h"
#include <xdc/runtime/Log.h>

#define DL_VERSION 			(0x01) /* version 1 */
#define DL_OBJECT_SIZE 		(0x0A) /* 32-bit timestamp, 32-bit value, 12-bit battery level  */
#define DL_SECTOR_SIZE		(0x1000)
#define DL_SECTOR_OFFSET	(3) //was 3
#define DL_SECTOR_COUNT		(45)//changed from 12 //from 180 8/9/2020

static int op_sector_erase(struct ringfs_flash_partition *flash, int address)
{
	int page = address / flash->sector_size;
	eraseFlash(page);
    return 0;
}

static size_t op_program(struct ringfs_flash_partition *flash, int address, const void *data, size_t size)
{
	int page = address / flash->sector_size;
	int offset = address % flash->sector_size;
	return writeFlashPg(page, offset, (uint8_t *)data, size);	
}

static size_t op_read(struct ringfs_flash_partition *flash, int address, void *data, size_t size)
{
	int page = address / flash->sector_size;
	int offset = address % flash->sector_size;
	return readFlashPg(page, offset, (uint8_t *)data, size);		
}

static struct ringfs fs;

static struct ringfs_flash_partition flash = {
	.sector_size = DL_SECTOR_SIZE,
	.sector_offset = DL_SECTOR_OFFSET,
	.sector_count = DL_SECTOR_COUNT,
    .sector_erase = op_sector_erase,
    .program = op_program,
    .read = op_read
};

static uint8_t object_buffer[DL_OBJECT_SIZE];

int dl_init(void)
{
	int res;

	res = ringfs_init(&fs, &flash, DL_VERSION, DL_OBJECT_SIZE);

	if (res != 0)
	{
		Log_info0("ringfs initialisation failed");
		return res;
	}
	// ringfs_format(&fs);
	res = ringfs_scan(&fs);

	if (res < 0)
	{
		Log_info0("Failed to find a valid filesystem, formatting");
		res = ringfs_format(&fs);
	}

	if (res != 0)
	{
		Log_info0("Unable to init data logger");
		return res;
	}
	//res = ringfs_format(&fs); // just for testing
	return res;
}

int dl_append(uint32_t timestamp, uint32_t tank_level, uint32_t battery_level)
{	
	object_buffer[0] = (timestamp >> 24) & 0xFF;
	object_buffer[1] = (timestamp >> 16) & 0xFF;
	object_buffer[2] = (timestamp >> 8) & 0xFF;
	object_buffer[3] = timestamp & 0xFF;
	object_buffer[4] = (tank_level >> 24) & 0xFF;
	object_buffer[5] = (tank_level >> 16) & 0xFF;
	object_buffer[6] = (tank_level >> 8) & 0xFF;
	object_buffer[7] = tank_level & 0xFF;	
	object_buffer[8] = (battery_level >> 8) & 0xFF;
	object_buffer[9] = battery_level  & 0xFF;
	return ringfs_append(&fs, object_buffer);
}

int dl_fetch_raw(uint8_t *object_value, size_t size)
{
	if (size != DL_OBJECT_SIZE)
	{
		Log_info0("fetch raw invalid length");
		return -1;		
	}

	int res = ringfs_fetch(&fs, object_value);

	if (res != 0)
	{
		Log_info0("Failed to fetch object");
		return res;
	}				
		
	return DL_SUCCESS;
}

int dl_fetch_raw_LIFO(uint8_t *object_value, size_t size)
{
    if (size != DL_OBJECT_SIZE)
    {
        Log_info0("fetch raw invalid length");
        return -1;
    }

    int res = ringfs_fetch_LIFO(&fs, object_value);

    if (res != 0)
    {
        Log_info0("Failed to fetch object");
        return res;
    }

    return DL_SUCCESS;
}




int dl_fetch(uint32_t *timestamp, uint32_t *tank_level, uint32_t *battery_level)
{
	int res = dl_fetch_raw(object_buffer, DL_OBJECT_SIZE);

	if (res == DL_SUCCESS)
	{
		*timestamp = (object_buffer[0] << 24) | 
			(object_buffer[1] << 16) |
			(object_buffer[2] << 8) |
			object_buffer[3];

		*tank_level = (object_buffer[4] << 24) | 
			(object_buffer[5] << 16) |
			(object_buffer[6] << 8) |
			object_buffer[7];

		*battery_level = (object_buffer[8] << 8) |
			object_buffer[9];			
	}

	return res;
}

int dl_rewind()
{
	return ringfs_rewind(&fs);
}

int dl_fastforward()
{
    return ringfs_fastforward(&fs);
}

/**
 * @brief       删除timestamp之前时间的log
 * @param[in]   timestamp: 需要删除的多久之前的日志
 * @return      删除条目数量
 */
int dl_delete(uint32_t timestamp)
{
	int res;
	uint32_t log_time;
	int detele_items = 0;

	ringfs_rewind(&fs);
	do{
		res = ringfs_fetch(&fs, object_buffer);

		if(res == 0)
		{
			log_time = (object_buffer[0] << 24) | 
					(object_buffer[1] << 16) |
					(object_buffer[2] << 8) |
					object_buffer[3];
			if(log_time < timestamp)
			{
				ringfs_discard(&fs);
				detele_items++;
			}
			else
			{
				break;
			}
			
		}
	}while (res == 0);
	
	return detele_items;
}

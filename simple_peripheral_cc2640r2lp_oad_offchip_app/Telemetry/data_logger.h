#ifndef SBP_TELEMETRY_DATA_LOGGER_H
#define SBP_TELEMETRY_DATA_LOGGER_H

#include <string.h>
#include "hal_board.h"
#include <stdlib.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define DL_SUCCESS      (0x00)

// initialises the data logger
int dl_init(void);

// appends a sensor reading to flash
int dl_append(uint32_t timestamp, uint32_t tank_level, uint32_t battery_level);

// fetches the oldest value from flash, uses a read
// cursor which is increment each fetch
// once the read cursor reaches head the fetch will fail
// use dl_rewind to reset the read cursor back to tail
int dl_fetch_raw(uint8_t *object_value, size_t size);

// fetches the newest value from flash, uses a read
// cursor which is idecremented each fetch
// once the read cursor reaches tail the fetch will fail
// use dl_rewind to reset the read cursor back to head
int dl_fetch_raw_LIFO(uint8_t *object_value, size_t size);

// fetches a sensor reading from flash using dl_fetch_raw
int dl_fetch(uint32_t *timestamp, uint32_t *tank_level, uint32_t *battery_level);

// rewinds the read cursor back to the oldest stored value
int dl_rewind();

// fastforward the read cursor to the newest stored value
int dl_fastforward();

/**
 * @brief       删除timestamp之前时间的log
 * @param[in]   timestamp: 需要删除的多久之前的日志
 * @return      删除条目数量
 */
int dl_delete(uint32_t timestamp);


#ifdef __cplusplus
}
#endif

#endif /* SBP_TELEMETRY_DATA_LOGGER_H */

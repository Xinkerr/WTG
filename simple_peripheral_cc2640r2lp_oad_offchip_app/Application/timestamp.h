/******************************************************************************

 @file       timestamp.h

 ******************************************************************************/
 
#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// set the current rtc timestamp, seconds since epoch
void Timestamp_Set(uint32_t timestamp);

// gets the current rtc timestamp
uint32_t Timestamp_Get(void);

// get the current rtc into a 4 byte buffer
// if the len is not 4 bytes nothing will happen
void Timestamp_Fetch(uint8_t *timestamp, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* TIMESTAMP_H */

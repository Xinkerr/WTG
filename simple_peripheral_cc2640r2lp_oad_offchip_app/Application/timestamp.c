#include "timestamp.h"

#include <ti/sysbios/hal/Seconds.h>
#include "tank_meter_service.h"


void Timestamp_Set(uint32_t timestamp)
{
  Seconds_set(timestamp);
}

uint32_t Timestamp_Get(void)
{
  return Seconds_get();
}

void Timestamp_Fetch(uint8_t *timestamp, uint16_t len)
{  
  if (len == TMS_TIME_LEN)
  {
    uint32_t now = Seconds_get();
    timestamp[0] = (now >> 24) & 0xFF;
    timestamp[1] = (now >> 16) & 0xFF;
    timestamp[2] = (now >> 8) & 0xFF;
    timestamp[3] = now & 0xFF;    
  }
}

/******************************************************************************
 * Filename:       Tank_Meter_Service.c
 *
 * Description:    This file contains the implementation of the service.
 *
 *                 Generated by:
 *                 BDS version: 1.1.3139.0
 *                 Plugin:      Texas Instruments BLE SDK GATT Server plugin 1.0.9
 *                 Time:        Tue Jun 26 2018 14:32:43 GMT+10:00
 *

 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

//#define xdc_runtime_Log_DISABLE_ALL 1  // Add to disable logs from this file
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>
#ifdef UARTLOG_ENABLE
#  include "UartLog.h"
#endif

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "tank_meter_service.h"
#include "timestamp.h"
#include "simple_peripheral_oad_offchip.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/

// Tank_Meter_Service Service UUID
CONST uint8_t TankMeterServiceUUID[ATT_UUID_SIZE] =
{
  TANK_METER_SERVICE_SERV_UUID_BASE128(TANK_METER_SERVICE_SERV_UUID)
};

// Tank_Level UUID
CONST uint8_t tms_Tank_LevelUUID[ATT_UUID_SIZE] =
{
  TMS_TANK_LEVEL_UUID_BASE128(TMS_TANK_LEVEL_UUID)
};

// Manufacture_Date UUID
CONST uint8_t tms_Manufacture_DateUUID[ATT_UUID_SIZE] =
{
  TMS_MANUFACTURE_DATE_UUID_BASE128(TMS_MANUFACTURE_DATE_UUID)
};

// Comms_Board UUID
CONST uint8_t tms_Comms_BoardUUID[ATT_UUID_SIZE] =
{
  TMS_COMMS_BOARD_UUID_BASE128(TMS_COMMS_BOARD_UUID)
};

// Time UUID
CONST uint8_t tms_TimeUUID[ATT_UUID_SIZE] =
{
  TMS_TIME_UUID_BASE128(TMS_TIME_UUID)
};

// LED UUID
CONST uint8_t tms_LEDUUID[ATT_UUID_SIZE] =
{
  TMS_LED_UUID_BASE128(TMS_LED_UUID)
};

// Historic_Level UUID
CONST uint8_t tms_Historic_LevelUUID[ATT_UUID_SIZE] =
{
  TMS_HISTORIC_LEVEL_UUID_BASE128(TMS_HISTORIC_LEVEL_UUID)
};

// Battery_Level UUID
CONST uint8_t tms_Battery_LevelUUID[ATT_UUID_SIZE] =
{
  TMS_BATTERY_LEVEL_UUID_BASE128(TMS_BATTERY_LEVEL_UUID)
};


/*********************************************************************
 * LOCAL VARIABLES
 */

static TankMeterServiceCBs_t *pAppCBs = NULL;
static uint8_t tms_icall_rsp_task_id = INVALID_TASK_ID;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t TankMeterServiceDecl = { ATT_UUID_SIZE, TankMeterServiceUUID };

// Characteristic "Tank Level" Properties (for declaration)
static uint8_t tms_Tank_LevelProps = GATT_PROP_READ;

// Characteristic "Tank Level" Value variable
static uint8_t tms_Tank_LevelVal[TMS_TANK_LEVEL_LEN] = {0};

// Length of data in characteristic "Tank Level" Value variable, initialized to minimal size.
static uint16_t tms_Tank_LevelValLen = TMS_TANK_LEVEL_LEN_MIN;



// Characteristic "Manufacture Date" Properties (for declaration)
static uint8_t tms_Manufacture_DateProps = GATT_PROP_READ;

// Characteristic "Manufacture Date" Value variable
static uint8_t tms_Manufacture_DateVal[TMS_MANUFACTURE_DATE_LEN] = {0};

// Length of data in characteristic "Manufacture Date" Value variable, initialized to minimal size.
static uint16_t tms_Manufacture_DateValLen = TMS_MANUFACTURE_DATE_LEN_MIN;



// Characteristic "Comms Board" Properties (for declaration)
static uint8_t tms_Comms_BoardProps = GATT_PROP_READ;

// Characteristic "Comms Board" Value variable
static uint8_t tms_Comms_BoardVal[TMS_COMMS_BOARD_LEN] = {0};

// Length of data in characteristic "Comms Board" Value variable, initialized to minimal size.
static uint16_t tms_Comms_BoardValLen = TMS_COMMS_BOARD_LEN_MIN;



// Characteristic "Time" Properties (for declaration)
static uint8_t tms_TimeProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic "Time" Value variable
static uint8_t tms_TimeVal[TMS_TIME_LEN] = {0};

// Length of data in characteristic "Time" Value variable, initialized to minimal size.
static uint16_t tms_TimeValLen = TMS_TIME_LEN_MIN;



// Characteristic "LED" Properties (for declaration)
static uint8_t tms_LEDProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic "LED" Value variable
static uint8_t tms_LEDVal[TMS_LED_LEN] = {0};

// Length of data in characteristic "LED" Value variable, initialized to minimal size.
static uint16_t tms_LEDValLen = TMS_LED_LEN_MIN;



// Characteristic "Historic Level" Properties (for declaration)
static uint8_t tms_Historic_LevelProps = GATT_PROP_NOTIFY;

// Characteristic "Historic Level" Value variable
static uint8_t tms_Historic_LevelVal[TMS_HISTORIC_LEVEL_LEN] = {0};

// Length of data in characteristic "Historic Level" Value variable, initialized to minimal size.
static uint16_t tms_Historic_LevelValLen = TMS_HISTORIC_LEVEL_LEN_MIN;

// Characteristic "Historic Level" Client Characteristic Configuration Descriptor
static gattCharCfg_t *tms_Historic_LevelConfig;



// Characteristic "Battery Level" Properties (for declaration)
static uint8_t tms_Battery_LevelProps = GATT_PROP_READ;

// Characteristic "Battery Level" Value variable
static uint8_t tms_Battery_LevelVal[TMS_BATTERY_LEVEL_LEN] = {0};

// Length of data in characteristic "Battery Level" Value variable, initialized to minimal size.
static uint16_t tms_Battery_LevelValLen = TMS_BATTERY_LEVEL_LEN_MIN;



/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t Tank_Meter_ServiceAttrTbl[] =
{
  // Tank_Meter_Service Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&TankMeterServiceDecl
  },
    // Tank Level Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &tms_Tank_LevelProps
    },
      // Tank Level Characteristic Value
      {
        { ATT_UUID_SIZE, tms_Tank_LevelUUID },
        GATT_PERMIT_READ,
        0,
        tms_Tank_LevelVal
      },
    // Manufacture Date Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &tms_Manufacture_DateProps
    },
      // Manufacture Date Characteristic Value
      {
        { ATT_UUID_SIZE, tms_Manufacture_DateUUID },
        GATT_PERMIT_READ,
        0,
        tms_Manufacture_DateVal
      },
    // Comms Board Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &tms_Comms_BoardProps
    },
      // Comms Board Characteristic Value
      {
        { ATT_UUID_SIZE, tms_Comms_BoardUUID },
        GATT_PERMIT_READ,
        0,
        tms_Comms_BoardVal
      },
    // Time Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &tms_TimeProps
    },
      // Time Characteristic Value
      {
        { ATT_UUID_SIZE, tms_TimeUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        tms_TimeVal
      },
    // LED Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &tms_LEDProps
    },
      // LED Characteristic Value
      {
        { ATT_UUID_SIZE, tms_LEDUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        tms_LEDVal
      },
    // Historic Level Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &tms_Historic_LevelProps
    },
      // Historic Level Characteristic Value
      {
        { ATT_UUID_SIZE, tms_Historic_LevelUUID },
        0,
        0,
        tms_Historic_LevelVal
      },
      // Historic Level CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)&tms_Historic_LevelConfig
      },
    // Battery Level Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &tms_Battery_LevelProps
    },
      // Battery Level Characteristic Value
      {
        { ATT_UUID_SIZE, tms_Battery_LevelUUID },
        GATT_PERMIT_READ,
        0,
        tms_Battery_LevelVal
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t Tank_Meter_Service_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                           uint16_t maxLen, uint8_t method );
static bStatus_t Tank_Meter_Service_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                            uint8_t *pValue, uint16_t len, uint16_t offset,
                                            uint8_t method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t Tank_Meter_ServiceCBs =
{
  Tank_Meter_Service_ReadAttrCB,  // Read callback function pointer
  Tank_Meter_Service_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * TankMeterService_AddService- Initializes the TankMeterService service by registering
 *          GATT attributes with the GATT server.
 *
 *    rspTaskId - The ICall Task Id that should receive responses for Indications.
 */
extern bStatus_t TankMeterService_AddService( uint8_t rspTaskId )
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  tms_Historic_LevelConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( tms_Historic_LevelConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, tms_Historic_LevelConfig );
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( Tank_Meter_ServiceAttrTbl,
                                        GATT_NUM_ATTRS( Tank_Meter_ServiceAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &Tank_Meter_ServiceCBs );
  Log_info1("Registered service, %d attributes", (IArg)GATT_NUM_ATTRS( Tank_Meter_ServiceAttrTbl ));
  tms_icall_rsp_task_id = rspTaskId;

  return ( status );
}

/*
 * TankMeterService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t TankMeterService_RegisterAppCBs( TankMeterServiceCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pAppCBs = appCallbacks;
    Log_info1("Registered callbacks to application. Struct %p", (IArg)appCallbacks);
    return ( SUCCESS );
  }
  else
  {
    Log_warning0("Null pointer given for app callbacks.");
    return ( FAILURE );
  }
}

/*
 * TankMeterService_SetParameter - Set a TankMeterService parameter.
 *
 *    param - Profile parameter ID
 *    len   - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
bStatus_t TankMeterService_SetParameter( uint8_t param, uint16_t len, void *value )
{
  bStatus_t ret = SUCCESS;
  uint8_t  *pAttrVal;
  uint16_t *pValLen;
  uint16_t valMinLen;
  uint16_t valMaxLen;
  uint8_t sendNotiInd = FALSE;
  gattCharCfg_t *attrConfig;
  uint8_t needAuth;

  switch ( param )
  {
    case TMS_TANK_LEVEL_ID:
      pAttrVal  =  tms_Tank_LevelVal;
      pValLen   = &tms_Tank_LevelValLen;
      valMinLen =  TMS_TANK_LEVEL_LEN_MIN;
      valMaxLen =  TMS_TANK_LEVEL_LEN;
      Log_info2("SetParameter : %s len: %d", (IArg)"Tank_Level", (IArg)len);
      break;

    case TMS_MANUFACTURE_DATE_ID:
      pAttrVal  =  tms_Manufacture_DateVal;
      pValLen   = &tms_Manufacture_DateValLen;
      valMinLen =  TMS_MANUFACTURE_DATE_LEN_MIN;
      valMaxLen =  TMS_MANUFACTURE_DATE_LEN;
      Log_info2("SetParameter : %s len: %d", (IArg)"Manufacture_Date", (IArg)len);
      break;

    case TMS_COMMS_BOARD_ID:
      tms_Comms_BoardValLen = len;
      pAttrVal  =  tms_Comms_BoardVal;
      pValLen   = &tms_Comms_BoardValLen;
      valMinLen =  TMS_COMMS_BOARD_LEN_MIN;
      valMaxLen =  TMS_COMMS_BOARD_LEN;
      Log_info2("SetParameter : %s len: %d", (IArg)"Comms_Board", (IArg)len);
      break;

    case TMS_TIME_ID:
      pAttrVal  =  tms_TimeVal;
      pValLen   = &tms_TimeValLen;
      valMinLen =  TMS_TIME_LEN_MIN;
      valMaxLen =  TMS_TIME_LEN;
      Log_info2("SetParameter : %s len: %d", (IArg)"Time", (IArg)len);
      break;

    case TMS_LED_ID:
      pAttrVal  =  tms_LEDVal;
      pValLen   = &tms_LEDValLen;
      valMinLen =  TMS_LED_LEN_MIN;
      valMaxLen =  TMS_LED_LEN;
      Log_info2("SetParameter : %s len: %d", (IArg)"LED", (IArg)len);
      break;

    case TMS_HISTORIC_LEVEL_ID:
      pAttrVal  =  tms_Historic_LevelVal;
      pValLen   = &tms_Historic_LevelValLen;
      valMinLen =  TMS_HISTORIC_LEVEL_LEN_MIN;
      valMaxLen =  TMS_HISTORIC_LEVEL_LEN;
      sendNotiInd = TRUE;
      attrConfig  = tms_Historic_LevelConfig;
      needAuth    = FALSE; // Change if authenticated link is required for sending.
      Log_info2("SetParameter : %s len: %d", (IArg)"Historic_Level", (IArg)len);
      break;

    case TMS_BATTERY_LEVEL_ID:
      pAttrVal  =  tms_Battery_LevelVal;
      pValLen   = &tms_Battery_LevelValLen;
      valMinLen =  TMS_BATTERY_LEVEL_LEN_MIN;
      valMaxLen =  TMS_BATTERY_LEVEL_LEN;
      Log_info2("SetParameter : %s len: %d", (IArg)"Battery_Level", (IArg)len);
      break;

    default:
      Log_error1("SetParameter: Parameter #%d not valid.", (IArg)param);
      return INVALIDPARAMETER;
  }

  // Check bounds, update value and send notification or indication if possible.
  if ( len <= valMaxLen && len >= valMinLen )
  {
    memcpy(pAttrVal, value, len);
    *pValLen = len; // Update length for read and get.

    if (sendNotiInd)
    {
      Log_info2("Transmitting noti/ind- connHandle %d, %s", (IArg)attrConfig[0].connHandle,
                                                    (IArg)( (attrConfig[0].value==0)?"Noti/ind disabled":
                                                    (attrConfig[0].value==1)?"Notification enabled":
                                                     "Indication enabled" ) );
      // Try to send notification.
      GATTServApp_ProcessCharCfg( attrConfig, pAttrVal, needAuth,
                                  Tank_Meter_ServiceAttrTbl, GATT_NUM_ATTRS( Tank_Meter_ServiceAttrTbl ),
                                  tms_icall_rsp_task_id,  Tank_Meter_Service_ReadAttrCB);
    }
  }
  else
  {
    Log_error3("Length outside bounds: Len: %d MinLen: %d MaxLen: %d.", (IArg)len, (IArg)valMinLen, (IArg)valMaxLen);
    ret = bleInvalidRange;
  }

  return ret;
}


/*
 * TankMeterService_GetParameter - Get a TankMeterService parameter.
 *
 *    param - Profile parameter ID
 *    len   - pointer to a variable that contains the maximum length that can be written to *value.
              After the call, this value will contain the actual returned length.
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
bStatus_t TankMeterService_GetParameter( uint8_t param, uint16_t *len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case TMS_TIME_ID:
      Timestamp_Fetch(tms_TimeVal, tms_TimeValLen);
      *len = MIN(*len, tms_TimeValLen);
      memcpy(value, tms_TimeVal, *len);
      Log_info2("GetParameter : %s returning %d bytes", (IArg)"Time", (IArg)*len);
      break;

    case TMS_LED_ID:
      *len = MIN(*len, tms_LEDValLen);
      memcpy(value, tms_LEDVal, *len);
      Log_info2("GetParameter : %s returning %d bytes", (IArg)"LED", (IArg)*len);
      break;

    default:
      Log_error1("GetParameter: Parameter #%d not valid.", (IArg)param);
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}

/*********************************************************************
 * @internal
 * @fn          Tank_Meter_Service_findCharParamId
 *
 * @brief       Find the logical param id of an attribute in the service's attr table.
 *
 *              Works only for Characteristic Value attributes and
 *              Client Characteristic Configuration Descriptor attributes.
 *
 * @param       pAttr - pointer to attribute
 *
 * @return      uint8_t paramID (ref Tank_Meter_Service.h) or 0xFF if not found.
 */
static uint8_t Tank_Meter_Service_findCharParamId(gattAttribute_t *pAttr)
{
  // Is this a Client Characteristic Configuration Descriptor?
  if (ATT_BT_UUID_SIZE == pAttr->type.len && GATT_CLIENT_CHAR_CFG_UUID == *(uint16_t *)pAttr->type.uuid)
    return Tank_Meter_Service_findCharParamId(pAttr - 1); // Assume the value attribute precedes CCCD and recurse

  // Is this attribute in "Tank Level"?
  else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, tms_Tank_LevelUUID, pAttr->type.len))
    return TMS_TANK_LEVEL_ID;

  // Is this attribute in "Manufacture Date"?
  else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, tms_Manufacture_DateUUID, pAttr->type.len))
    return TMS_MANUFACTURE_DATE_ID;

  // Is this attribute in "Comms Board"?
  else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, tms_Comms_BoardUUID, pAttr->type.len))
    return TMS_COMMS_BOARD_ID;

  // Is this attribute in "Time"?
  else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, tms_TimeUUID, pAttr->type.len))
    return TMS_TIME_ID;

  // Is this attribute in "LED"?
  else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, tms_LEDUUID, pAttr->type.len))
    return TMS_LED_ID;

  // Is this attribute in "Historic Level"?
  else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, tms_Historic_LevelUUID, pAttr->type.len))
    return TMS_HISTORIC_LEVEL_ID;

  // Is this attribute in "Battery Level"?
  else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, tms_Battery_LevelUUID, pAttr->type.len))
    return TMS_BATTERY_LEVEL_ID;

  else
    return 0xFF; // Not found. Return invalid.
}

/*********************************************************************
 * @fn          Tank_Meter_Service_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t Tank_Meter_Service_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                       uint16_t maxLen, uint8_t method )
{
  bStatus_t status = SUCCESS;
  uint16_t valueLen;
  uint8_t paramID = 0xFF;

  // Find settings for the characteristic to be read.
  paramID = Tank_Meter_Service_findCharParamId( pAttr );
  switch ( paramID )
  {
    case TMS_TANK_LEVEL_ID:
      valueLen = tms_Tank_LevelValLen;

      Log_info4("ReadAttrCB : %s connHandle: %d offset: %d method: 0x%02x",
                 (IArg)"Tank_Level",
                 (IArg)connHandle,
                 (IArg)offset,
                 (IArg)method);
      /* Other considerations for Tank Level can be inserted here */
      break;

    case TMS_MANUFACTURE_DATE_ID:
      valueLen = tms_Manufacture_DateValLen;

      Log_info4("ReadAttrCB : %s connHandle: %d offset: %d method: 0x%02x",
                 (IArg)"Manufacture_Date",
                 (IArg)connHandle,
                 (IArg)offset,
                 (IArg)method);
      /* Other considerations for Manufacture Date can be inserted here */
      break;

    case TMS_COMMS_BOARD_ID:
      valueLen = tms_Comms_BoardValLen;

      Log_info4("ReadAttrCB : %s connHandle: %d offset: %d method: 0x%02x",
                 (IArg)"Comms_Board",
                 (IArg)connHandle,
                 (IArg)offset,
                 (IArg)method);
      /* Other considerations for Comms Board can be inserted here */
      break;

    case TMS_TIME_ID:
      valueLen = tms_TimeValLen;

      Log_info4("ReadAttrCB : %s connHandle: %d offset: %d method: 0x%02x",
                 (IArg)"Time",
                 (IArg)connHandle,
                 (IArg)offset,
                 (IArg)method);
      /* Other considerations for Time can be inserted here */
      Timestamp_Fetch(tms_TimeVal, tms_TimeValLen);
      break;

    case TMS_LED_ID:
      valueLen = tms_LEDValLen;

      Log_info4("ReadAttrCB : %s connHandle: %d offset: %d method: 0x%02x",
                 (IArg)"LED",
                 (IArg)connHandle,
                 (IArg)offset,
                 (IArg)method);
      /* Other considerations for LED can be inserted here */
      break;

    case TMS_HISTORIC_LEVEL_ID:
      valueLen = tms_Historic_LevelValLen;

      Log_info4("ReadAttrCB : %s connHandle: %d offset: %d method: 0x%02x",
                 (IArg)"Historic_Level",
                 (IArg)connHandle,
                 (IArg)offset,
                 (IArg)method);
      /* Other considerations for Historic Level can be inserted here */
      break;

    case TMS_BATTERY_LEVEL_ID:
      valueLen = tms_Battery_LevelValLen;

      Log_info4("ReadAttrCB : %s connHandle: %d offset: %d method: 0x%02x",
                 (IArg)"Battery_Level",
                 (IArg)connHandle,
                 (IArg)offset,
                 (IArg)method);
      /* Other considerations for Battery Level can be inserted here */
      app_setBatteryLevel();
      break;

    default:
      Log_error0("Attribute was not found.");
      return ATT_ERR_ATTR_NOT_FOUND;
  }
  // Check bounds and return the value
  if ( offset > valueLen )  // Prevent malicious ATT ReadBlob offsets.
  {
    Log_error0("An invalid offset was requested.");
    status = ATT_ERR_INVALID_OFFSET;
  }
  else
  {
    *pLen = MIN(maxLen, valueLen - offset);  // Transmit as much as possible
    memcpy(pValue, pAttr->pValue + offset, *pLen);
  }

  return status;
}

/*********************************************************************
 * @fn      Tank_Meter_Service_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t Tank_Meter_Service_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t len, uint16_t offset,
                                        uint8_t method )
{
  bStatus_t status  = SUCCESS;
  uint8_t   paramID = 0xFF;
  uint8_t   changeParamID = 0xFF;
  uint16_t writeLenMin;
  uint16_t writeLenMax;
  uint16_t *pValueLenVar;

  // See if request is regarding a Client Characterisic Configuration
  if (ATT_BT_UUID_SIZE == pAttr->type.len && GATT_CLIENT_CHAR_CFG_UUID == *(uint16_t *)pAttr->type.uuid)
  {
    // Allow notification and indication, but do not check if really allowed per CCCD.
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY |
                                                     GATT_CLIENT_CFG_INDICATE );
    if (SUCCESS == status && pAppCBs && pAppCBs->pfnCfgChangeCb)
       pAppCBs->pfnCfgChangeCb( connHandle, TANK_METER_SERVICE_SERV_UUID,
                                Tank_Meter_Service_findCharParamId(pAttr), pValue, len );

     return status;
  }

  // Find settings for the characteristic to be written.
  paramID = Tank_Meter_Service_findCharParamId( pAttr );
  switch ( paramID )
  {
    case TMS_TIME_ID:
      writeLenMin  = TMS_TIME_LEN_MIN;
      writeLenMax  = TMS_TIME_LEN;
      pValueLenVar = &tms_TimeValLen;

      Log_info5("WriteAttrCB : %s connHandle(%d) len(%d) offset(%d) method(0x%02x)",
                 (IArg)"Time",
                 (IArg)connHandle,
                 (IArg)len,
                 (IArg)offset,
                 (IArg)method);
      /* Other considerations for Time can be inserted here */
      break;

    case TMS_LED_ID:
      writeLenMin  = TMS_LED_LEN_MIN;
      writeLenMax  = TMS_LED_LEN;
      pValueLenVar = &tms_LEDValLen;

      Log_info5("WriteAttrCB : %s connHandle(%d) len(%d) offset(%d) method(0x%02x)",
                 (IArg)"LED",
                 (IArg)connHandle,
                 (IArg)len,
                 (IArg)offset,
                 (IArg)method);
      /* Other considerations for LED can be inserted here */
      break;

    default:
      Log_error0("Attribute was not found.");
      return ATT_ERR_ATTR_NOT_FOUND;
  }
  // Check whether the length is within bounds.
  if ( offset >= writeLenMax )
  {
    Log_error0("An invalid offset was requested.");
    status = ATT_ERR_INVALID_OFFSET;
  }
  else if ( offset + len > writeLenMax )
  {
    Log_error0("Invalid value length was received.");
    status = ATT_ERR_INVALID_VALUE_SIZE;
  }
  else if ( offset + len < writeLenMin && ( method == ATT_EXECUTE_WRITE_REQ || method == ATT_WRITE_REQ ) )
  {
    // Refuse writes that are lower than minimum.
    // Note: Cannot determine if a Reliable Write (to several chars) is finished, so those will
    //       only be refused if this attribute is the last in the queue (method is execute).
    //       Otherwise, reliable writes are accepted and parsed piecemeal.
    Log_error0("Invalid value length was received.");
    status = ATT_ERR_INVALID_VALUE_SIZE;
  }
  else
  {
    // Copy pValue into the variable we point to from the attribute table.
    memcpy(pAttr->pValue + offset, pValue, len);

    // Only notify application and update length if enough data is written.
    //
    // Note: If reliable writes are used (meaning several attributes are written to using ATT PrepareWrite),
    //       the application will get a callback for every write with an offset + len larger than _LEN_MIN.
    // Note: For Long Writes (ATT Prepare + Execute towards only one attribute) only one callback will be issued,
    //       because the write fragments are concatenated before being sent here.
    if ( offset + len >= writeLenMin )
    {
      changeParamID = paramID;
      *pValueLenVar = offset + len; // Update data length.
    }
  }

  // Let the application know something changed (if it did) by using the
  // callback it registered earlier (if it did).
  if (changeParamID != 0xFF)
    if ( pAppCBs && pAppCBs->pfnChangeCb )
      pAppCBs->pfnChangeCb( connHandle, TANK_METER_SERVICE_SERV_UUID, paramID, pValue, len+offset ); // Call app function from stack task context.

  return status;
}

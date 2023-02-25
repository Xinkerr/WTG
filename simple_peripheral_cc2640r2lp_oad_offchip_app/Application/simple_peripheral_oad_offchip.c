/******************************************************************************

 @file       simple_peripheral_oad_offchip.c

 @brief This file contains the Oad User sample application  based on
        simple_peripheral for use with the CC2650 Bluetooth Low Energy
        Protocol Stack.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2017-2017, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc2640r2_sdk_01_50_00_58
 Release Date: 2017-10-17 19:31:43
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <ti/display/Display.h>

#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>
// #include <UartLog.h>

#include <ti/drivers/ADC.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "devinfoservice.h"
#include "tank_meter_service.h"
#include "ll_common.h"

// Used for imgHdr_t structure
#include "oad_image_header.h"
// Needed for HAL_SYSTEM_RESET()
#include "hal_mcu.h"

#include "oad.h"
#include "Telemetry/data_logger.h"

#include "peripheral.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

#include "app_def.h"

#include "board.h"
#include "board_key.h"

#include "two_btn_menu.h"
#include "simple_peripheral_oad_offchip.h"
#include "timestamp.h"

#include "ble_user_config.h"

#include "sigfox.h"

/*********************************************************************
 * TODO THIS NEEDS TO GO IN BOARD.H
 */

#ifndef Board_Reed1
#define Board_Reed1              PIN_UNASSIGNED
#define Board_Reed2              PIN_UNASSIGNED
#define Board_Reed3              PIN_UNASSIGNED
#else
#define BOARD_REED_SUPPORTED     1
#endif // Board_Reed1

/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms) 15/01/2019 Changed to 1022.5mS 1600=1000ms per RH
// #define DEFAULT_ADVERTISING_INTERVAL          1636//1022.5ms as per Apple Dev Spec
//#define DEFAULT_ADVERTISING_INTERVAL          1920//1200ms as 2020.12.02s
#define DEFAULT_ADVERTISING_INTERVAL          1360//850ms as 2022.2.19

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL    12 //8

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     24//8

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          300//1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPROLE_LINK_PARAM_UPDATE_WAIT_REMOTE_PARAMS

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         3//6

// Offset into the scanRspData string the software version info is stored
#define OAD_SOFT_VER_OFFSET                   15

// Application specific event ID for HCI Connection Event End Events
#define SBP_HCI_CONN_EVT_END_EVT              0x0001
#define SBP_OAD_CONN_EVT_END_EVT              0x0002

// Use UART display
#if 0
#define SBP_DISPLAY_TYPE Display_Type_UART
#else
#define SBP_DISPLAY_TYPE Display_Type_INVALID
#endif

// Task configuration
#define SBP_TASK_PRIORITY                     1

// Warning! To optimize RAM, task stack size must be a multiple of 8 bytes
#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                  1024 //1000
#endif

// tank historic heartbeat period in milliseconds (10 mins) 10*60000
#define TANK_HISTORIC_HEARTBEAT_PERIOD        (10*60000) // change from 10min to 1 min for testing

// message send period in milliseconds (10 mins)
#define MESSAGE_TRIGGER_PERIOD        (10 * 60000)

// message send period in milliseconds (1440 mins) Once every 24 Hrs
//#define MESSAGE_HEARTBEAT_PERIOD        (1440 * 60000)

// length of string related to a single pulse (30 degree segment)
// #define SEGMENT_LENGTH                        16 //15/01/2019 changed per RH
#define SEGMENT_LENGTH                        164 //16.4mm  18/11/2020 changed

#define HEARTBEAT_LOG                          12 * 6 //Number of hours * tank historic heartbeat period in minutes between heartbeat log to flash

#define OLD_LOG_TIME_INTERVAL                  (6*30*24*3600)  //6 months
// #define OLD_LOG_TIME_INTERVAL                  (5*60)  //5minues for test

#define MULTIPLE_BUTTON_CHECK                  (1000)

// ADC Counts to uV on the Battery input (measured via 5 data points, no offset taken into account)
#define BATTERY_COUNTS_TO_MICROVOLTS          1060.31138

// Minimum battery voltage for the LED to go on (uV)
#define BATTERY_MIN_LED                       2000000

#define ZERO_LED                              Board_GLED
#define ZERO_SHORT_BLINK                      250 // number of milliseconds for short led blink
#define ZERO_LONG_BLINK                       2000 // number of milliseconds for long led blink
#define ZERO_XSHORT_BLINK                     125 // number of milliseconds for xshort led blink
#define ZERO_XLONG_BLINK                      5000 // number of milliseconds for xlong led blink
#define ZERO_BT_ON                            30000 // number of milliseconds on when turned on via BT
#define SIGFOX_TEST_BLINK                     150
#define ZERO_LED_LOWER                        3 // number of seconds (min) for zero
#define ZERO_LED_UPPER                        30 // number of seconds (max) for zero
#define ZERO_LED_STATE_IDLE                   0 // state is idle. turn led off
#define ZERO_LED_STATE_BLINK_1                1 // state is blink 1 (first led blink on zero)
#define ZERO_LED_STATE_BLINK_2                2 // state is blink 1 (second led blink on zero)
#define ZERO_LED_STATE_BLINK_3                3 // state is blink 3 (second led blink on zero)
#define ZERO_LED_STATE_BLINK_4                4 // state is blink 1 (first led blink on zero)
#define ZERO_LED_STATE_BLINK_5                5 // state is blink 1 (second led blink on zero)
#define ZERO_LED_STATE_BLINK_6                6 // state is blink 3 (second led blink on zero)
#define CALIBRATE_COMPLETE                    7 // state is on due to setting of bluetooth characteristic
#define ZERO_LED_STATE_BT                     8 // state is on due to setting of bluetooth characteristic
#define STARTUP_TEST_SIGFOX                   9 // device startup, sigfox send message for test



// Comms Module defines
#define COMMS_BOARD_I2C_ADDRESS               0x50  // I2C Address of the comms module
#define COMMS_BOARD_I2C_REG_TYPE              0x00  // Module Type 
#define COMMS_BOARD_I2C_REG_REGION            0x01  // Module Region
#define COMMS_BOARD_I2C_REG_MANUFACTURER      0x02  // Radio Manufacturer
#define COMMS_BOARD_I2C_REG_VERSION           0x03  // Module Version

#define COMMS_BOARD_SIGFOX                    0x53  // Board type for Sigfox

// Application events used with app queue (appEvtHdr_t)
// These are not related to RTOS evts, but instead enqueued via state change CBs
#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_PASSCODE_NEEDED_EVT               0x0002
#define APP_MSG_SERVICE_WRITE                 0x0003 /* A characteristic value has been written     */
#define APP_MSG_SERVICE_CFG                   0x0004 /* A characteristic configuration has changed  */
#define APP_MSG_UPDATE_CHARVAL                0x0005 /* Request from ourselves to update a value    */
#define APP_LOG_TANK_HISTORIC                 0x0006 /* Debug message for logging a dummy sensor value from the debug blink led timer */
#define APP_READ_TANK_HISTORIC                0x0007 /* Called from the clock swi to read the next history value from flash */
#define APP_MSG_BUTTON_DEBOUNCED              0x0008 /* A button has been debounced with new value  */
#define APP_TOGGLE_ZERO_LED                   0x0009 /* Toggles the zero led to indicate state events, i.e. zero tank length etc */
#define APP_MSG_SEND                          0x000A /* Sends a Message via the comms module.*/
#define APP_CALIBRATE_LED              0x000B /* Check if gauge is calibrated. If not blinks led 3 fast 5 second gap*/
//#define APP_MSG_HEARTBEAT                     0x000B /* Sends a Message via the comms module at the heatbeat interval.*/
#define APP_SIGFOX_MSG_TEST                   0x000C  /*sigfox send msg for test*/
#define APP_MULTIPLE_BUTTON                   0x000D  /*single button multi-function*/
#define APP_10MIN_TIMER                       0x000E
#define APP_ADV_UPDATE                        0x000F

#define SBP_OAD_QUEUE_EVT                     OAD_QUEUE_EVT       // Event_Id_01
#define SBP_OAD_COMPLETE_EVT                  OAD_DL_COMPLETE_EVT // Event_Id_02

// Internal Events for RTOS application
#define SBP_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SBP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define SBP_ALL_EVENTS                        (SBP_ICALL_EVT        | \
                                               SBP_QUEUE_EVT        | \
                                               SBP_OAD_QUEUE_EVT    | \
                                               SBP_OAD_COMPLETE_EVT)

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
  uint8_t *pData;   // Event payload
} sbpEvt_t;

// Struct for messages about characteristic data
typedef struct
{
  uint16_t svcUUID; // UUID of the service
  uint16_t dataLen; //
  uint8_t  paramID; // Index of the characteristic
  uint8_t  data[];  // Flexible array member, extended to malloc - sizeof(.)
} char_data_t;

typedef struct 
{
  uint8_t last_reed_switch;
  uint8_t first_reading;
  int32_t line_length;
  int32_t previous_line_length;
  uint32_t zero_button_ts; // Zero button timestamp
  uint32_t led_state; // the state of the button led (i.e. blink on event)
} tank_level_t;

typedef struct 
{
  uint32_t notify; // indicates the notification service is active
  uint32_t oad; // indicates the oad service is active
  Semaphore_Struct semStruct;
  Semaphore_Handle semHandle;
} tank_historic_t;

// Struct for message about button state
typedef struct
{
  PIN_Id   pinId;
  uint8_t  state;
} button_state_t;

// Struct for details on comms board
typedef struct
{
  uint8_t type;         // type of board, 0 means no board
  uint8_t region;       // the module region
  uint8_t manufacturer; // the module manufacturer
  uint8_t version;      // the module version
} comms_board_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

// Watchdog
Watchdog_Handle watchdogHandle;

extern const imgHdr_t _imgHdr;


/* PIN_Id for active reed (in debounce period) */
PIN_Id activereedPinId;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct sbpTask;

#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(sbpTaskStack, 8)
#elif __IAR_SYSTEMS_ICC__
#pragma data_alignment=8
#endif    //__TI_COMPILER_VERSION__
uint8_t sbpTaskStack[SBP_TASK_STACK_SIZE];


// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0x12,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'T',
  'A',
  'N',
  'K',
  ' ',
  'M',
  'E',
  'T',
  'E',
  'R',
  ' ',
  ' ',
  'v',
  ' ', // These 4 octets are placeholders for the SOFTVER field
  ' ', // which will be updated at init time
  ' ',
  ' ',

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 10ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 10ms
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertising)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,  // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16(OAD_SERVICE_UUID),
  HI_UINT16(OAD_SERVICE_UUID),

  0x09,
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,
    //Company Identifier
    0x0d,
    0x00,

    //Additional Data
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Tank Meter off-chip";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

// Variable used to store the number of messages pending once OAD completes
// The application cannot reboot until all pending messages are sent
static uint8_t numPendingMsgs = 0;

// Flag to be stored in NV that tracks whether service changed
// indications needs to be sent out
static uint32_t  sendSvcChngdOnNextBoot = FALSE;

// Clock structs for periodic notifications
static Clock_Struct tank_historic_clock;
static Clock_Struct heartbeat_clock;
static Clock_Struct led_clock;
static Clock_Struct comms_board_reset_clock;
static Clock_Struct message_trigger_clock;
static Clock_Struct multiple_presses_clock;
static Clock_Struct task_10min_clock;
#if CALIBRATE_LED_ENABLE
static Clock_Struct calibrate_clock;
#endif
//static Clock_Struct message_heartbeat_clock;

// Pin driver handles
static PIN_Handle ledPinHandle;
static PIN_Handle buttonPinHandle;
static PIN_Handle reedPinHandle;
static PIN_Handle commsPinHandle;

// Global memory storage for a PIN_Config table
static PIN_State ledPinState;
static PIN_State buttonPinState;
static PIN_State reedPinState;
static PIN_State commsPinState;

// Global tank data
static tank_level_t tank_level;
static tank_historic_t tank_historic;

// Global Battery data
static uint32_t adcValueMicroVolt = 0;

static  int rescheck;
#if CALIBRATE_LED_ENABLE
static uint32_t calibrate_status;
#endif

// Initial LED pin configuration table
// LEDs Board_LED0 & Board_LED1 are off.
PIN_Config ledPinTable[] = {
  Board_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  Board_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

// State variable for debugging LEDs
PIN_State  sbpLedState;

// Initial comms pin configuration table
// Comms pin is low by default (comms board in reset)// change back to PIN_OPENDRAIN
PIN_Config commsPinTable[] = {
  Board_Comms | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_OPENDRAIN | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config buttonPinTable[] = {
    Board_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

// Clock objects for debouncing the buttons
static Clock_Struct button_debounce_clock;

// Clock objects for debouncing the reed
static Clock_Struct reed_debounce_clock;


// State of the buttons
static uint8_t button0_state = 0;


// State Send message pending
static uint8_t send_message = 0;

static uint8_t send_message_periodic = 0;
static uint8_t log_message_periodic = 0;
static uint8_t TimeIsSet = 0;
static uint8_t Calibrated = 0;

/*
 * Reed switch inputs for line length calculation
 *   - interrupts are triggered on falling edge need to change to NOPULL from PULLUP with 100K Resitors
 */
PIN_Config reedPinTable[] = {
     Board_Reed1 | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_NEGEDGE | PIN_HYSTERESIS,
     Board_Reed2 | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_NEGEDGE | PIN_HYSTERESIS,
     Board_Reed3 | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_NEGEDGE | PIN_HYSTERESIS,
     PIN_TERMINATE
 };

static comms_board_t comms_board;
static bool comms_board_present = false;
static volatile bool comms_board_reset_timer = false;

static uint8_t button_press_cnt = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void app_init(void);
static void app_taskFxn(UArg a0, UArg a1);

static uint8_t app_processStackMsg(ICall_Hdr *pMsg);
static uint8_t app_processGATTMsg(gattMsgEvent_t *pMsg);
static void app_processAppMsg(sbpEvt_t *pMsg);
static void app_processApplicationMessage(sbpEvt_t *pMsg);
static void app_processStateChangeEvt(gaprole_States_t newState);
static void app_sendAttRsp(void);
static void app_freeAttRsp(uint8_t status);
static void app_stateChangeCB(gaprole_States_t newState);
void app_enqueueMsg(uint8_t event, uint8_t state, uint8_t *pData);
static void app_enqueueCharDataMsg( uint8_t event,
                                     uint16_t connHandle,
                                     uint16_t serviceUUID, uint8_t paramID,
                                     uint8_t *pValue, uint16_t len );
static void app_enqueueBtnMsg(uint8_t event, button_state_t *button_state);
static void app_processOadWriteCB(uint8_t event, uint16_t arg);
static uint8_t app_processL2CAPMsg(l2capSignalEvent_t *pMsg);
static void app_processPasscode(gapPasskeyNeededEvent_t *pData);
static void app_passcodeCB(uint8_t *deviceAddr,
                                            uint16_t connHandle,
                                            uint8_t uiInputs, uint8_t uiOutputs,
                                            uint32_t numComparison);

// Watchdog callback handler
static void app_watchdog_callback(uintptr_t unused);

// Generic callback handlers for value changes in services.
static void app_service_valueChangeCB( uint16_t connHandle, uint16_t svcUuid, uint8_t paramID, uint8_t *pValue, uint16_t len );
static void app_service_cfgChangeCB( uint16_t connHandle, uint16_t svcUuid, uint8_t paramID, uint8_t *pValue, uint16_t len );

// Task context handlers for generated services.
static void app_tankMeterService_valueChangeHandler(char_data_t *pCharData);
static void app_tankMeterService_cfgChangeHandler(char_data_t *pCharData);

// Callback handler(s) for the clock object(s) for notifications
static void app_tankHistoric_clockSwiHandler(UArg paramID);
static void app_heartbeat_clockSwiHandler(UArg paramID);
static void app_comms_board_reset_clockSwiHandler(UArg paramID);
static void app_button_debounce_clockSwiHandler(UArg buttonId);
static void app_led_clockSwiHandler(UArg paramID);
static void task_10min_clockSwiHandler(UArg paramID);
static void multiple_presses_clockSwiHandler(UArg paramID);
#if CALIBRATE_LED_ENABLE
static void app_calibrate_led_clockSwiHandler(UArg paramID);
#endif
static void app_message_trigger_clockSwiHandler(UArg paramID);
//static void app_message_heartbeat_clockSwiHandler(UArg paramID);
// Add in for Reed switch clock handler
//static void app_reed_debounce_clockSwiHandler(UArg reedId);


// GPIO callback handler(s)
static void app_button_handler(PIN_Handle handle, PIN_Id pinId);
static void app_reed_handler(PIN_Handle handle, PIN_Id pinId);

static void app_updateCharVal(char_data_t *pCharData);

static void app_updateReed(uint8_t reedSwitch, uint8_t incReedSwitch);
static void app_resetTankLevel(void);
static void app_setTankLevel(void);

static void app_getBatteryLevel(void);

static void app_logTankHistoric(void);
static void app_startTankHistoric(void);
static void app_stopTankHistoric(void);
static void app_notifyTankHistoric(void);
static int app_fetchTankHistoric(uint8_t *object_value, size_t size);

static void app_onButtonPressed(button_state_t *button_state);
static void sigfox_test_led_blink(void);
static void app_toggle_zero_led();
static void multiple_presses_process(void);
#if CALIBRATE_LED_ENABLE
static void app_calibrate_led();
#endif
static void app_set_zero_led(uint8_t led_state);
static void app_check_comms_module(void);
static void app_send_message(void);
static void app_comms_reset(void);
static void app_send_test_msg(void);

static char *util_convertArrayToHexString(uint8_t const *src, uint8_t src_len,
                                          uint8_t *dst, uint8_t dst_len);

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8_t assertCause, uint8_t assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t app_gapRoleCBs =
{
  app_stateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t app_BondMgrCBs =
{
  (pfnPasscodeCB_t)app_passcodeCB, // Passcode callback,
  NULL  // Pairing / Bonding state Callback (not used by application)
};

static oadTargetCBs_t app_oadCBs =
{
  app_processOadWriteCB // Write Callback.
};

/*
 * Callbacks in the user application for events originating from BLE services.
 */
// Tank Meter Service callback handler.
// The type Tank_Meter_ServiceCBs_t is defined in Tank_Meter_Service.h
static TankMeterServiceCBs_t user_Tank_Meter_ServiceCBs =
{
  .pfnChangeCb    = app_service_valueChangeCB, // Characteristic value change callback handler
  .pfnCfgChangeCb = app_service_cfgChangeCB, // Noti/ind configuration callback handler
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void advertising_data_update(void)
{
    uint32_t batteryLevel = adcValueMicroVolt / 1000;
    uint16_t tmp_battery = (uint16_t)batteryLevel;
    int32_t length = (int32_t)(tank_level.line_length * SEGMENT_LENGTH / 10);

    memcpy(&advertData[11], &length, 4);
    memcpy(&advertData[15], &tmp_battery, 2);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
}

static void app_adv_update_sched(void)
{
    app_enqueueMsg(APP_ADV_UPDATE, NULL, NULL);
}
/*********************************************************************
 * @fn      app_createTask
 *
 * @brief   Task creation function for the OAD User App.
 *
 * @param   None.
 *
 * @return  None.
 */
void app_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = SBP_TASK_STACK_SIZE;
  taskParams.priority = SBP_TASK_PRIORITY;

  Task_construct(&sbpTask, app_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      app_setCommsBoardDetails
 *
 * @brief   Sets the comms board details in the bluetooth characteristic
 *
 * @param   id - null terminated string with the id of the board
 *         pac - null terminated string with the pac of the board
 *
 * @return  None.
 */
void app_setCommsBoardDetails(char* id, char* pac)
{
    // create the fields and write to Bluetooth
    // Fields in characteristic "Comms Board"
    //   Field "Installed" format: boolean, bits: 8
    //   Field "Serial_Number" format: utf8s, bits: -1
    //   Field "Hardware_Version" format: utf8s, bits: -1
    //   Field "Firmware_Version" format: utf8s, bits: -1
    //   Field "Type" format: uint8, bits: 8
    //   Field "PAC" format: utf8s, bits: -1
    //   Field "DEVICE_ID" format: utf8s, bits: -1

    // a buffer to hold the characteristic fields max size of 100 bytes
    uint8_t characteristics[100];
    int index = 0;

    // initialiase it
    for (index = 0; index < 100; ++index)
    {
        characteristics[index] = 0;
    }
    index = 0;

    if (comms_board_present)
    {
        int i = 0;

        characteristics[index] = 1; // board is installed
        characteristics[++index] = 0; // no serial number
        characteristics[++index] = comms_board.version + '0'; // version converted to character (assumes version < 10)
        characteristics[++index] = comms_board.region;// changed to region from 0; // null terminate the version
        characteristics[++index] = comms_board.manufacturer;// changed to manufacturer from 0;0; // no firmware version
        characteristics[++index] = comms_board.type; // module type

        // copy the PAC and null terminate
        if (pac != NULL)
        {
            while ((pac[i] != 0) && (i < SIGFOX_PAC_LEN))
            {
                characteristics[++index] = pac[i++];
            }
        }
        characteristics[++index] = 0;

        // copy the Device ID and null terminate
        if (id != NULL)
        {
            i = 0;
            while ((id[i] != 0) && (i < SIGFOX_DEVICE_ID_LEN))
            {
                characteristics[++index] = id[i++];
            }
        }
        characteristics[++index] = 0;
    }
    else
    {
        // no board so write the minimum
        index = 7;
    }

    TankMeterService_SetParameter(TMS_COMMS_BOARD_ID, index, characteristics);
}


/*********************************************************************
 * @fn      app_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void app_init(void)
{
  // ******************************************************************
  // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  Clock_Params clockParams;
  Clock_Params_init(&clockParams);

  Log_info0("Initializing the user task, hardware, BLE stack and services.");

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // dispHandle = Display_open(SBP_DISPLAY_TYPE, NULL);
  
  Watchdog_init();
  Watchdog_Params params;
  Watchdog_Params_init(&params);
  params.callbackFxn = (Watchdog_Callback)app_watchdog_callback;
  params.resetMode = Watchdog_RESET_ON;
  watchdogHandle = Watchdog_open(Board_WATCHDOG0, &params);
  if (watchdogHandle == NULL) {
    Log_error0("Error initializing watchdog");
    Task_exit();
  }

  // Read in the OAD Software version
  uint8_t swVer[OAD_SW_VER_LEN];
  OAD_getSWVersion(swVer, OAD_SW_VER_LEN);

  // ******************************************************************
  // Initialization of tank and tank history parameters
  // ******************************************************************    
  tank_level.last_reed_switch = 0;
  tank_level.first_reading = 0;
  tank_level.line_length = 0;
  tank_level.previous_line_length = 0;
  tank_level.zero_button_ts = 0;
  tank_level.led_state = ZERO_LED_STATE_IDLE;

  tank_historic.notify = 0;
  tank_historic.oad = 0;

  Semaphore_Params semParams;
  Semaphore_Params_init(&semParams);
  Semaphore_construct(&tank_historic.semStruct, 1, &semParams);
  tank_historic.semHandle = Semaphore_handle(&tank_historic.semStruct);

  // ******************************************************************
  // Hardware initialization
  // ******************************************************************

  // set the rtc to something, this needs to be set by the phone app
  // // Human time (GMT): 00:00:00 Thursday, 1 January 1970
  Timestamp_Set(0);

  // Initialise the ADC
  ADC_init();


  reedPinHandle = PIN_open(&reedPinState, reedPinTable);
  if(!reedPinHandle) {
    Log_error0("Error initializing reed pins");
    Task_exit();
  }

  // Setup callback for reed pins
  if (PIN_registerIntCb(reedPinHandle, &app_reed_handler) != 0) {
    Log_error0("Error registering reed callback function");
    Task_exit();
  }



  // Open LED pins
  ledPinHandle = PIN_open(&ledPinState, ledPinTable);
  if(!ledPinHandle) {
    Log_error0("Error initializing board LED pins");
    Task_exit();
  }

  // Set the output value equal to the received value. 0 is off, not 0 is on
  PIN_setOutputValue(ledPinHandle, Board_GLED, 0);
  PIN_setOutputValue(ledPinHandle, Board_RLED, 0);

  buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
  if(!buttonPinHandle) {
    Log_error0("Error initializing button pins");
    Task_exit();
  }

    clockParams.period = 0; 
    Clock_construct(&multiple_presses_clock, multiple_presses_clockSwiHandler,
                  MULTIPLE_BUTTON_CHECK * (1000 / Clock_tickPeriod),
                  &clockParams);
  //   Clock_start((Clock_Handle)&multiple_presses_clock);
  // }

  // Setup callback for button pins
  if (PIN_registerIntCb(buttonPinHandle, &app_button_handler) != 0) {
    Log_error0("Error registering button callback function");
    Task_exit();
  }



  // Open Comm reset pin
  commsPinHandle = PIN_open(&commsPinState, commsPinTable);
  if(!commsPinHandle) {
    Log_error0("Error initializing board comms pins");
    Task_exit();
  }



  // ******************************************************************
  // Initialization of clock objects used for notifiable characterisics
  // ******************************************************************
  // Clock_Params clockParams;
  // Clock_Params_init(&clockParams);
  clockParams.period = 5000 * (1000 / Clock_tickPeriod); // 5000ms period

  // Clock struct initialization for periodic notification example
  // Clock callbacks only have one parameter, so make one callback handler per service
  // and one Clock Struct per noti/ind characteristic.

  // Create one-shot clocks for tank historic
  clockParams.arg = TMS_HISTORIC_LEVEL_ID;
  clockParams.period = 0;
  Clock_construct(&tank_historic_clock,
                  app_tankHistoric_clockSwiHandler,
                  10 * (1000 / Clock_tickPeriod),
                  &clockParams);

  // Create the debounce clock objects for Button 0 and Button 1
  clockParams.arg = Board_BUTTON0;

  // Initialize to 50 ms timeout when Clock_start is called.
  // Timeout argument is in ticks, so convert from ms to ticks via tickPeriod.
  Clock_construct(&button_debounce_clock, app_button_debounce_clockSwiHandler,
                  50 * (1000/Clock_tickPeriod),
                  &clockParams);


  // Create the message send clock objects
  clockParams.arg = 0;//need to determine this

  // Initialize to 1 ms timeout when Clock_start is called.
  // Timeout argument is in ticks, so convert from ms to ticks via tickPeriod.
  Clock_construct(&message_trigger_clock, app_message_trigger_clockSwiHandler,
                  MESSAGE_TRIGGER_PERIOD  * (1000/Clock_tickPeriod),
                  &clockParams);


  // Initialize to 100 ms timeout when Clock_start is called for comms board reset.
  // Timeout argument is in ticks, so convert from ms to ticks via tickPeriod.
  clockParams.arg = 0;
  Clock_construct(&comms_board_reset_clock, app_comms_board_reset_clockSwiHandler,
                  100 * (1000/Clock_tickPeriod),
                  &clockParams);

  // Create the heartbeat sensor reading clock
  Clock_Params_init(&clockParams);
  clockParams.period = TANK_HISTORIC_HEARTBEAT_PERIOD * (1000 / Clock_tickPeriod);
  clockParams.arg = 0;
  Clock_construct(&heartbeat_clock,
                  app_heartbeat_clockSwiHandler,
                  0,
                  &clockParams);
  Clock_start((Clock_Handle)&heartbeat_clock);

  // Create the led toggle clock
  clockParams.arg = 0;  
  clockParams.period = 0;
  Clock_construct(&led_clock,
                  app_led_clockSwiHandler,
                  0,
                  &clockParams);

  // Create the 10min period task clock
   clockParams.arg = 0;
   clockParams.period =10 * 60 * 1000 * (1000/Clock_tickPeriod);
   Clock_construct(&task_10min_clock,
                   task_10min_clockSwiHandler,
                   clockParams.period,
                   &clockParams);
   Clock_start((Clock_Handle)&task_10min_clock);

  // Create the led toggle clock
  clockParams.arg = 0;
  clockParams.period = 0;
  #if CALIBRATE_LED_ENABLE
  Clock_construct(&calibrate_clock,
                  app_calibrate_led_clockSwiHandler,
                  0,
                  &clockParams);
  #endif
  // Put the comms board in reset
  PIN_setOutputValue(commsPinHandle, Board_Comms, 0);
  Clock_start((Clock_Handle)&comms_board_reset_clock);


  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    // Setup the dyanmic portion of the scanRspData
    scanRspData[OAD_SOFT_VER_OFFSET] = swVer[0];
    scanRspData[OAD_SOFT_VER_OFFSET + 1] = swVer[1];
    scanRspData[OAD_SOFT_VER_OFFSET + 2] = swVer[2];
    scanRspData[OAD_SOFT_VER_OFFSET + 3] = swVer[3];

    // Set scanRspData
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }

  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager
  {
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = TRUE;
    uint8_t scMode = GAPBOND_SECURE_CONNECTION_ALLOW;

    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    GAPBondMgr_SetParameter(GAPBOND_SECURE_CONNECTION, sizeof(uint8_t), &scMode);
  }

   // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service

  // Open the OAD module and add the OAD service to the application
  if(OAD_SUCCESS != OAD_open(OAD_DEFAULT_INACTIVITY_TIME))
  {
    Log_info0("OAD failed to open");
  }
  else
  {
    // Resiter the OAD callback with the application
    OAD_register(&app_oadCBs);
  }

  // Open the Data Logging module
  if (dl_init() != DL_SUCCESS)
  {
      Log_info0("Data logger failed to open");
  }

  // Add services to GATT server and give ID of this task for Indication acks.
  TankMeterService_AddService( selfEntity );

  // Register callbacks with the generated services that
  // can generate events (writes received) to the application
  TankMeterService_RegisterAppCBs( &user_Tank_Meter_ServiceCBs );

  // Initialisation of characteristics in Tank_Meter_Service
  uint8_t initVal[40] = {0};
  uint32_t initTime = 0; // Changed to 00:00:00 Thursday, 1 January 1970 from 1526437680;
  app_setBatteryLevel();
  app_resetTankLevel();
  Calibrated = 0;
  TankMeterService_SetParameter(TMS_MANUFACTURE_DATE_ID, TMS_MANUFACTURE_DATE_LEN, &initTime);

  // Check for the comms module and set the service parameters
  while(!comms_board_reset_timer);
  comms_board_reset_timer = false;
  PIN_setOutputValue(commsPinHandle, Board_Comms, 1);
  Clock_start((Clock_Handle)&comms_board_reset_clock);
  app_check_comms_module();
  while(!comms_board_reset_timer);
  if (comms_board.type == COMMS_BOARD_SIGFOX)
  {
      comms_board_present = sigfox_init(comms_board.region);
  }

  TankMeterService_SetParameter(TMS_TIME_ID, TMS_TIME_LEN, &initTime);
  TankMeterService_SetParameter(TMS_LED_ID, TMS_LED_LEN, &initVal);
  TankMeterService_SetParameter(TMS_HISTORIC_LEVEL_ID, TMS_HISTORIC_LEVEL_LEN, &initTime);


  // Start the Device
  VOID GAPRole_StartDevice(&app_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&app_BondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

#if !defined (USE_LL_CONN_PARAM_UPDATE)
  // Get the currently set local supported LE features
  // The HCI will generate an HCI event that will get received in the main
  // loop
  HCI_LE_ReadLocalSupportedFeaturesCmd();
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

  uint8_t versionStr[OAD_SW_VER_LEN + 1];

  memcpy(versionStr, swVer, OAD_SW_VER_LEN);

  // Add in Null terminator
  versionStr[OAD_SW_VER_LEN] = NULL;

  // Display Image version
  Log_info1("Tank Meter Off-chip v%s", versionStr);

  /*
   * When switching from persistent app back to the user application for the
   * for the first time after an OAD the device must send a service changed
   * indication. This will cause any peers to rediscover services.
   *
   * To prevent sending a service changed IND on every boot, a flag is stored
   * in NV to determine whether or not the service changed IND needs to be
   * sent
   */
  uint8_t status = osal_snv_read(BLE_NVID_CUST_START,
                                  sizeof(sendSvcChngdOnNextBoot),
                                  (uint8 *)&sendSvcChngdOnNextBoot);
  if(status != SUCCESS)
  {
    /*
     * On first boot the NV item will not have yet been initialzed, and the read
     * will fail. Do a write to set the initial value of the flash in NV
     */
     osal_snv_write(BLE_NVID_CUST_START, sizeof(sendSvcChngdOnNextBoot),
                    (uint8 *)&sendSvcChngdOnNextBoot);
  }
  #if CALIBRATE_LED_ENABLE
  app_calibrate_led();
  #endif

  app_adv_update_sched();
}

/*********************************************************************
 * @fn      app_taskFxn
 *
 * @brief   Application task entry point for the OAD User App.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void app_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  app_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, SBP_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & SBP_HCI_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              app_sendAttRsp();
            }
            else if(pEvt->event_flag & SBP_OAD_CONN_EVT_END_EVT)
            {
              // Wait until all pending messages are sent
              if(numPendingMsgs == 0)
              {
                // Store the flag to indicate that a service changed IND will
                // be sent at the next boot
                //app_saveTankLevel(); // Saves Tank level before an OAD update. removed 15/01/2019 as per RH
                sendSvcChngdOnNextBoot = TRUE;

                uint8_t status = osal_snv_write(BLE_NVID_CUST_START,
                                                sizeof(sendSvcChngdOnNextBoot),
                                                (uint8 *)&sendSvcChngdOnNextBoot);
                if(status != SUCCESS)
                {
                  Log_info1("SNV WRITE FAIL: %d", status);
                }

                // Reset the system
                HAL_SYSTEM_RESET();
              }
              numPendingMsgs--;
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = app_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & SBP_QUEUE_EVT)
      {

        // Get the first message from the Queue
        sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);

        while (pMsg != NULL)
        {
          if (pMsg)
          {
            // Process message.
            app_processAppMsg(pMsg);

            if (pMsg->pData != NULL)
            {
              // Free the Queue payload if there is one
              ICall_free(pMsg->pData);
            }

            // Free the space from the message.
            ICall_free(pMsg);
          }

          // Dequeue the next message
          pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
        }
      }
      // OAD events
      if(events & SBP_OAD_QUEUE_EVT)
      {
        tank_historic.oad = 1;
        PIN_setOutputValue(ledPinHandle, Board_GLED, 1);        

        // Process the OAD Message Queue
        uint8_t status = OAD_processQueue();

        // If the OAD state machine encountered an error, print it
        // Return codes can be found in oad_constants.h
        if(status == OAD_DL_COMPLETE)
        {
          Log_info0("OAD DL Complete, wait for Enable");
        }
        else if(status == OAD_IMG_ID_TIMEOUT)
        {
          Log_info0("ImgID Timeout, disconnecting");

          // This may be an attack, terminate the link
          GAPRole_TerminateConnection();
        }
        else if(status != OAD_SUCCESS)
        {
          Log_info1("OAD Error: %d", status);
        }

      }
      if(events & SBP_OAD_COMPLETE_EVT)
      {
        // Register for L2CAP Flow Control Events
        L2CAP_RegisterFlowCtrlTask(selfEntity);
      }
    }
  }
}

/*********************************************************************
 * @fn      app_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message payload.
 *
 * @return  None.
 */
void app_enqueueMsg(uint8_t event, uint8_t state, uint8_t *pData)
{
  sbpEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, syncEvent, (uint8*)pMsg);
  }
}

/*
 * @brief  Generic message constructor for characteristic data.
 *
 *         Sends a message to the application for handling in Task context where
 *         the message payload is a char_data_t struct.
 *
 *         From service callbacks the appMsgType is APP_MSG_SERVICE_WRITE or
 *         APP_MSG_SERVICE_CFG, and functions running in another context than
 *         the Task itself, can set the type to APP_MSG_UPDATE_CHARVAL to
 *         make the user Task loop invoke app_updateCharVal function for them.
 *
 * @param  event         message being sent
 * @param  connHandle    GAP Connection handle of the relevant connection
 * @param  serviceUUID   16-bit part of the relevant service UUID
 * @param  paramID       Index of the characteristic in the service
 * @oaram  *pValue       Pointer to characteristic value
 * @param  len           Length of characteristic data
 */
static void app_enqueueCharDataMsg( uint8_t event,
                                     uint16_t connHandle,
                                     uint16_t serviceUUID,
                                     uint8_t paramID,
                                     uint8_t *pValue,
                                     uint16_t len )
{
    uint16_t readLen = len; // How much data was written to the attribute
    char_data_t *pData;

    if ((pData = ICall_malloc(sizeof(char_data_t) + readLen)))
    {
      pData->svcUUID = serviceUUID; // Use 16-bit part of UUID.
      pData->paramID = paramID;
      memcpy(pData->data, pValue, readLen);
      pData->dataLen = readLen;

      // Enqueue the event.
      app_enqueueMsg(event, NULL, (uint8_t *) pData);
    }
}

static void app_enqueueBtnMsg(uint8_t event, button_state_t *button_state)
{
    button_state_t *pData;

    if ((pData = ICall_malloc(sizeof(button_state_t))))
    {
      pData->pinId = button_state->pinId;
      pData->state = button_state->state;
      app_enqueueMsg(event, NULL, (uint8_t *) pData);
    }
}

/*********************************************************************
 * @fn      app_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t app_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = app_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {

        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            {

#if !defined (USE_LL_CONN_PARAM_UPDATE)
              // This code will disable the use of the LL_CONNECTION_PARAM_REQ
              // control procedure (for connection parameter updates, the
              // L2CAP Connection Parameter Update procedure will be used
              // instead). To re-enable the LL_CONNECTION_PARAM_REQ control
              // procedures, define the symbol USE_LL_CONN_PARAM_UPDATE

              // Parse Command Complete Event for opcode and status
              hciEvt_CmdComplete_t* command_complete = (hciEvt_CmdComplete_t*) pMsg;
              uint8_t   pktStatus = command_complete->pReturnParam[0];

              //find which command this command complete is for
              switch (command_complete->cmdOpcode)
              {
                case HCI_LE_READ_LOCAL_SUPPORTED_FEATURES:
                  {
                    if (pktStatus == SUCCESS)
                    {
                      uint8_t featSet[8];

                      // get current feature set from received event (bits 1-9 of
                      // the returned data
                      memcpy( featSet, &command_complete->pReturnParam[1], 8 );

                      // Clear bit 1 of byte 0 of feature set to disable LL
                      // Connection Parameter Updates
                      CLR_FEATURE_FLAG( featSet[0], LL_FEATURE_CONN_PARAMS_REQ );

                      // Update controller with modified features
                      HCI_EXT_SetLocalSupportedFeaturesCmd( featSet );
                    }
                  }
                  break;

                default:
                  //do nothing
                  break;
              }
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

            }
            break;

          case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
            break;

          default:
            break;
        }
      }
      break;

    case L2CAP_SIGNAL_EVENT:
      // Process L2CAP signal
      safeToDealloc = app_processL2CAPMsg((l2capSignalEvent_t *)pMsg);
      break;

      default:
        // do nothing
        break;

    }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      app_processL2CAPMsg
 *
 * @brief   Process L2CAP messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t app_processL2CAPMsg(l2capSignalEvent_t *pMsg)
{
  uint8_t safeToDealloc = TRUE;
  static bool firstRun = TRUE;

  switch (pMsg->opcode)
  {
    case L2CAP_NUM_CTRL_DATA_PKT_EVT:
    {
      /*
      * We cannot reboot the device immediately after receiving
      * the enable command, we must allow the stack enough time
      * to process and respond to the OAD_EXT_CTRL_ENABLE_IMG
      * command. This command will determine the number of
      * packets currently queued up by the LE controller.
      * BIM var is already set via OadPersistApp_processOadWriteCB
      */
      if(firstRun)
      {
        firstRun = false;

        // We only want to set the numPendingMsgs once
        numPendingMsgs = MAX_NUM_PDU - pMsg->cmd.numCtrlDataPktEvt.numDataPkt;

        // Wait the number of connection events
        HCI_EXT_ConnEventNoticeCmd(OAD_getactiveCxnHandle(), selfEntity,
                                        SBP_OAD_CONN_EVT_END_EVT);
      }

      break;
    }
    default:
      break;
  }

  // It's safe to free the incoming message
  return (safeToDealloc);
}

/*********************************************************************
 * @fn      app_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t app_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   SBP_HCI_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      app_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    Log_info1("FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    OAD_setBlockSize(pMsg->msg.mtuEvt.MTU);
    Log_info1("MTU Size: %d", pMsg->msg.mtuEvt.MTU);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      app_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void app_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

      // We're done with the response message
      app_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      Log_info1("Rsp send retry: %d", rspTxRetry);
    }
  }
}

/*********************************************************************
 * @fn      app_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void app_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      Log_info1("Rsp sent retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

      Log_info1("Rsp retry failed: %d", rspTxRetry);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      app_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void app_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBP_STATE_CHANGE_EVT:
    {
      app_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;
    }

    case SBP_PASSCODE_NEEDED_EVT:
    {
      app_processPasscode((gapPasskeyNeededEvent_t*)pMsg->pData);
      // Free the app data
      ICall_free(pMsg->pData);
      break;
    }

    case APP_MSG_SERVICE_WRITE:
    case APP_MSG_SERVICE_CFG:
    case APP_MSG_UPDATE_CHARVAL:
      app_processApplicationMessage(pMsg);
      ICall_free(pMsg->pData);
      break;

    case APP_LOG_TANK_HISTORIC:
      app_logTankHistoric();
      break;

    case APP_READ_TANK_HISTORIC:
      app_notifyTankHistoric();
      break;

    case APP_MSG_BUTTON_DEBOUNCED:
      app_onButtonPressed((button_state_t*)pMsg->pData);
      ICall_free(pMsg->pData);
      break;

    case APP_MULTIPLE_BUTTON:
      multiple_presses_process();
      break;

    case APP_TOGGLE_ZERO_LED:
      app_toggle_zero_led();
      break;
    
    case APP_CALIBRATE_LED:
      #if CALIBRATE_LED_ENABLE
      app_calibrate_led();
      #endif
      break;

    case APP_RX_SIGFOX:
      sigfox_tick();
      break;

    case APP_MSG_SEND:
      app_send_message();
      break;

    case APP_SIGFOX_MSG_TEST:
        //LED blink
        tank_level.led_state = STARTUP_TEST_SIGFOX;
        sigfox_test_led_blink();
        //sigfox send message for test
        // app_comms_reset();
        // sigfox_test_send();
        app_send_test_msg();
      break;

    case APP_10MIN_TIMER:
      app_getBatteryLevel();
      advertising_data_update();
      break;

    case APP_ADV_UPDATE:
      advertising_data_update();
      break;

    default:
      // Do nothing.
      break;
  }
}

/*
 * @brief   Handle application messages
 *
 *          These are messages not from the BLE stack, but from the
 *          application itself.
 *
 *          For example, in a Software Interrupt (Swi) it is not possible to
 *          call any BLE APIs, so instead the Swi function must send a message
 *          to the application Task for processing in Task context.
 *
 * @param   pMsg  Pointer to the message of type app_msg_t.
 *
 * @return  None.
 */
static void app_processApplicationMessage(sbpEvt_t *pMsg)
{
  char_data_t *pCharData = (char_data_t *)pMsg->pData;

  switch (pMsg->hdr.event)
  {
    case APP_MSG_SERVICE_WRITE: /* Message about received value write */
      /* Call different handler per service */

      switch(pCharData->svcUUID) {
        case TANK_METER_SERVICE_SERV_UUID:
          app_tankMeterService_valueChangeHandler(pCharData);
          break;
      }
      break;

    case APP_MSG_SERVICE_CFG: /* Message about received CCCD write */
      /* Call different handler per service */
      switch(pCharData->svcUUID) {
        case TANK_METER_SERVICE_SERV_UUID:
          app_tankMeterService_cfgChangeHandler(pCharData);
          break;
      }
      break;

    case APP_MSG_UPDATE_CHARVAL: /* Message from ourselves to send  */
      app_updateCharVal(pCharData);
      break;
  }
}

/*********************************************************************
 * @fn      app_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void app_processStateChangeEvt(gaprole_States_t newState)
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        //systemId[4] = 0x00;
        //systemId[3] = 0x00;

        // shift three bytes up
        systemId[5] = ownAddress[5];
        systemId[4] = ownAddress[4];
        systemId[3] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
        DevInfo_SetParameter(DEVINFO_SERIAL_NUMBER, DEVINFO_SERIAL_NUMBER_LEN, systemId); // added
        // Display device address
        Log_info1("BD Addr: %s", Util_convertBdAddr2Str(ownAddress));
        Log_info0("GAPRole Initialized");
      }
      break;

    case GAPROLE_ADVERTISING:
      Log_info0("Advertising");
      break;

    case GAPROLE_CONNECTED:
      {
        linkDBInfo_t linkInfo;
        uint8_t numActive = 0;
        uint16_t connHandle = 0;

        GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connHandle);

        numActive = linkDB_NumActive();

        // Use numActive to determine the connection handle of the last
        // connection
        if ( linkDB_GetInfo( numActive - 1, &linkInfo ) == SUCCESS )
        {
          Log_info1("Connected to: %s", Util_convertBdAddr2Str(linkInfo.addr));
        }
        else
        {
          uint8_t peerAddress[B_ADDR_LEN];

          GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

          Log_info1("Connected to : %s", Util_convertBdAddr2Str(peerAddress));
        }

        if(sendSvcChngdOnNextBoot == TRUE)
        {
          GAPBondMgr_ServiceChangeInd( connHandle, TRUE);

          sendSvcChngdOnNextBoot = FALSE;
        }
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      Log_info0("Connected Advertising");
      break;

    case GAPROLE_WAITING:
      app_freeAttRsp(bleNotConnected);

      Log_info0("Disconnected");

      // Cancel the OAD if one is going on
      // A disconnect forces the peer to re-identify
      OAD_cancel();
      app_stopTankHistoric();
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      app_freeAttRsp(bleNotConnected);
      Log_info0("Timed Out");
      app_stopTankHistoric();
      break;

    case GAPROLE_ERROR:
      Log_info0("Error");
      break;

    default:
      break;
  }
}

/*********************************************************************
* @fn      app_processPasscode
*
* @brief   Process the Passcode request.
*
* @return  none
*/
static void app_processPasscode(gapPasskeyNeededEvent_t *pData)
{
  // Use static passcode
  uint32_t passcode = 123456;
  Log_info1("Passcode: %d", passcode);
  // Send passcode to GAPBondMgr
  GAPBondMgr_PasscodeRsp(pData->connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * Callback Functions - These run in the calling thread's context
 *********************************************************************/

 /*********************************************************************
* @fn      app_passcodeCB
*
* @brief   Passcode callback.
*
* @param   deviceAddr - pointer to device address
*
* @param   connHandle - the connection handle
*
* @param   uiInputs - pairing User Interface Inputs
*
* @param   uiOutputs - pairing User Interface Outputs
*
* @param   numComparison - numeric Comparison 20 bits
*
* @return  none
*/
static void app_passcodeCB(uint8_t *deviceAddr,
                                            uint16_t connHandle,
                                            uint8_t uiInputs, uint8_t uiOutputs,
                                            uint32_t numComparison)
{
  gapPasskeyNeededEvent_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(gapPasskeyNeededEvent_t))))
  {
    memcpy(pData->deviceAddr, deviceAddr, B_ADDR_LEN);
    pData->connectionHandle = connHandle;
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    app_enqueueMsg(SBP_PASSCODE_NEEDED_EVT, NULL,
                                    (uint8_t *) pData);
  }
}

/*********************************************************************
 * @fn      app_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void app_stateChangeCB(gaprole_States_t newState)
{
  app_enqueueMsg(SBP_STATE_CHANGE_EVT, newState, NULL);
}

/*********************************************************************
 * @fn      app_processOadWriteCB
 *
 * @brief   Process a write request to the OAD reset service
 *
 * @param   connHandle - the connection Handle this request is from.
 * @param   bim_var    - bim_var to set before resetting.
 *
 * @return  None.
 */
void app_processOadWriteCB(uint8_t event, uint16_t arg)
{  
  Event_post(syncEvent, event);
}

/**
 * Callback handler for watchdog
 */
static void app_watchdog_callback(uintptr_t unused)
{
  Watchdog_clear(watchdogHandle);
}

/**
 * Callback handler for characteristic value changes in services.
 */
static void app_service_valueChangeCB( uint16_t connHandle, uint16_t svcUuid,
                                        uint8_t paramID, uint8_t *pValue,
                                        uint16_t len )
{
  // See the service header file to compare paramID with characteristic.
  Log_info2("(CB) Characteristic value change: svc(0x%04x) paramID(%d). "
            "Sending msg to app.", (IArg)svcUuid, (IArg)paramID);
  app_enqueueCharDataMsg(APP_MSG_SERVICE_WRITE, connHandle, svcUuid, paramID,
                          pValue, len);
}

/**
 * Callback handler for characteristic configuration changes in services.
 */
static void app_service_cfgChangeCB( uint16_t connHandle, uint16_t svcUuid,
                                      uint8_t paramID, uint8_t *pValue,
                                      uint16_t len )
{
  Log_info2("(CB) Char config change: svc(0x%04x) paramID(%d). "
            "Sending msg to app.", (IArg)svcUuid, (IArg)paramID);
  app_enqueueCharDataMsg(APP_MSG_SERVICE_CFG, connHandle, svcUuid, paramID,
                          pValue, len);
}


/*
 * @brief   Handle a write request sent from a peer device.
 *
 *          Invoked by the Task based on a message received from a callback.
 *
 *          When we get here, the request has already been accepted by the
 *          service and is valid from a BLE protocol perspective as well as
 *          having the correct length as defined in the service implementation.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void app_tankMeterService_valueChangeHandler(char_data_t *pCharData)
{
  static uint8_t pretty_data_holder[16]; // 5 bytes as hex string "AA:BB:CC:DD:EE"
  uint32_t timestamp;
  util_convertArrayToHexString(pCharData->data, pCharData->dataLen,
                               pretty_data_holder, sizeof(pretty_data_holder));

  switch (pCharData->paramID)
  {
    case TMS_TIME_ID:
      Log_info3("Value Change msg: %s %s: %s",
                (IArg)"Tank Meter Service",
                (IArg)"Time",
                (IArg)pretty_data_holder);
      if (pCharData->dataLen == TMS_TIME_LEN)
      {
        timestamp = pCharData->data[0] << 24 | 
          pCharData->data[1] << 16 | 
          pCharData->data[2] << 8 | 
          pCharData->data[3];
        if((timestamp > 1500000000) && (timestamp < 2500000000)) //time between years 2017 and 2049 is valid
        {
        Timestamp_Set(timestamp);
        TimeIsSet = 1;
        }
      }
      break;

    case TMS_LED_ID:
      Log_info3("Value Change msg: %s %s: %s",
                (IArg)"Tank Meter Service",
                (IArg)"LED",
                (IArg)pretty_data_holder);
      if (pCharData->dataLen == TMS_LED_LEN)
      {
          tank_level.led_state = pCharData->data[0] ? ZERO_LED_STATE_BT : ZERO_LED_STATE_IDLE;
          app_toggle_zero_led();
      }
      break;

  default:
    return;
  }
}


/*
 * @brief   Handle a CCCD (configuration change) write received from a peer
 *          device. This tells us whether the peer device wants us to send
 *          Notifications or Indications.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void app_tankMeterService_cfgChangeHandler(char_data_t *pCharData)
{
  // Cast received data to uint16, as that's the format for CCCD writes.
  uint16_t configValue = *(uint16_t *)pCharData->data;
  char *configValString;

  // Determine what to tell the user
  switch(configValue)
  {
  case GATT_CFG_NO_OPERATION:
    configValString = "Noti/Ind disabled";
    break;
  case GATT_CLIENT_CFG_NOTIFY:
    configValString = "Notifications enabled";
    break;
  case GATT_CLIENT_CFG_INDICATE:
    configValString = "Indications enabled";
    break;
  }

  switch (pCharData->paramID)
  {
    case TMS_HISTORIC_LEVEL_ID:
      Log_info3("CCCD Change msg: %s %s: %s",
                (IArg)"Tank Meter Service",
                (IArg)"Historic Level",
                (IArg)configValString);
      // -------------------------
      // Do something useful with configValue here. It tells you whether someone
      // wants to know the state of this characteristic.
      if (configValue) // 0x0001 and 0x0002 both indicate turned on.
      {
        app_startTankHistoric();
      }
      else
      {
        app_stopTankHistoric();
      }
      break;

  default:
    return;
  }
}

/*********************************************************************
 * SWI Functions - These functions run at higher priority than any task
 *********************************************************************/

/*
 * Swi handler for clock object(s) used by Tank Service.
 * paramID is stored in the clock object for each characteristic.
 */
static void app_tankHistoric_clockSwiHandler(UArg paramID)
{
  app_enqueueMsg(APP_READ_TANK_HISTORIC, NULL, NULL);
}


/*
 * Swi handler for clock object(s) used by Tank Service.
 * paramID is stored in the clock object for each characteristic.
 */
static void app_message_trigger_clockSwiHandler(UArg paramID)
{

     app_enqueueMsg(APP_MSG_SEND, NULL, NULL);
    send_message = 0;
}


/*
 *  Swi handler for logging the tank sensor reading (heartbeat reading)
 */
static void app_heartbeat_clockSwiHandler(UArg paramID)
{
    app_enqueueMsg(APP_LOG_TANK_HISTORIC, NULL, NULL);
}

/*
 *  Swi handler for blinking the led.
 */
static void app_led_clockSwiHandler(UArg paramID)
{
  app_enqueueMsg(APP_TOGGLE_ZERO_LED, NULL, NULL);
}

static void task_10min_clockSwiHandler(UArg paramID)
{
    app_enqueueMsg(APP_10MIN_TIMER, NULL, NULL);
}

#if CALIBRATE_LED_ENABLE
/*
 *  Swi handler for blinking the led.
 */
static void app_calibrate_led_clockSwiHandler(UArg paramID)
{
  app_enqueueMsg(APP_CALIBRATE_LED, NULL, NULL);
}
#endif

/*
 *  Swi handler for resetting the comms board.
 */
static void app_comms_board_reset_clockSwiHandler(UArg paramID)
{
    comms_board_reset_timer = true;
}

/*
 * @brief  Callback from Clock module on timeout
 *
 *         Determines new state after debouncing
 *
 * @param  buttonId    The pin being debounced
 */
static void app_button_debounce_clockSwiHandler(UArg buttonId)
{
  // Used to send message to app
  button_state_t buttonMsg = { .pinId = buttonId };
  uint8_t        sendMsg   = FALSE;

  // Get current value of the button pin after the clock timeout
  uint8_t buttonPinVal = PIN_getInputValue(buttonId);

  // Set interrupt direction to opposite of debounced state
  // If button is now released (button is active low, so release is high)
  if (buttonPinVal)
  {
    // Enable negative edge interrupts to wait for press
    PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, buttonId | PIN_IRQ_NEGEDGE);
  }
  else
  {
    // Enable positive edge interrupts to wait for relesae
    PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, buttonId | PIN_IRQ_POSEDGE);
  }

  switch(buttonId)
  {
    case Board_BUTTON0:
      // If button is now released (buttonPinVal is active low, so release is 1)
      // and button state was pressed (buttonstate is active high so press is 1)
      if (buttonPinVal && button0_state)
      {
        // Button was released
        buttonMsg.state = button0_state = 0;
        sendMsg = TRUE;
      }
      else if (!buttonPinVal && !button0_state)
      {
        // Button was pressed
        buttonMsg.state = button0_state = 1;
        sendMsg = TRUE;
      }
      break;
  }

  if (sendMsg == TRUE)
  {
    app_enqueueBtnMsg(APP_MSG_BUTTON_DEBOUNCED, &buttonMsg);
  }
}




/*
 *  Callbacks from Hwi-context
 *****************************************************************************/

/*
 * @brief  Callback from PIN driver on interrupt
 *
 *         Sets in motion the debouncing.
 *
 * @param  handle    The PIN_Handle instance this is about
 * @param  pinId     The pin that generated the interrupt
 */
static void app_button_handler(PIN_Handle handle, PIN_Id pinId)
{
  Log_info1("Button interrupt: %s",
            (IArg)((pinId == Board_BUTTON0)?"Button 0":"UNKNOWN"));


  // Disable interrupt on that pin for now. Re-enabled after debounce.
  PIN_setConfig(handle, PIN_BM_IRQ, pinId | PIN_IRQ_DIS);

  // Start debounce timer
  switch (pinId)
  {
    case Board_BUTTON0:
      Clock_start(Clock_handle(&button_debounce_clock));
      break;
  }
}

/*
 * @brief  Callback from PIN driver on interrupt
 *
 *         Updates the line length.
 *         Increasing length happens in the order Reed1, Reed2, Reed3
 *         Decreasing length happens in the order Reed3, Reed2, Reed1
 *
 * @param  handle    The PIN_Handle instance this is about
 * @param  pinId     The pin that generated the interrupt
 */
static void app_reed_handler(PIN_Handle handle, PIN_Id pinId)
{
#ifdef BOARD_REED_SUPPORTED

    switch (pinId)
     {
       case Board_Reed1:
         Log_info0("Reed interrupt: Reed 1");
         app_updateReed(1, 3);
         break;

       case Board_Reed2:
         Log_info0("Reed interrupt: Reed 2");
         app_updateReed(2, 1);
         break;

       case Board_Reed3:
         Log_info0("Reed interrupt: Reed 3");
         app_updateReed(3, 2);
         break;

       default:
         // this shouldn't happen
         break;
     }

     app_setTankLevel();


#endif // BOARD_REED_SUPPORTED
}


/*********************************************************************
*********************************************************************/

/*
 * @brief  Convenience function for updating characteristic data via char_data_t
 *         structured message.
 *
 * @note   Must run in Task context in case BLE Stack APIs are invoked.
 *
 * @param  *pCharData  Pointer to struct with value to update.
 */
static void app_updateCharVal(char_data_t *pCharData)
{
  switch(pCharData->svcUUID) {
    case TANK_METER_SERVICE_SERV_UUID:
      TankMeterService_SetParameter( pCharData->paramID,
              pCharData->dataLen, pCharData->data );
    break;

  }
}

static void app_updateReed(uint8_t reedSwitch, uint8_t decReedSwitch)
{

    if (tank_level.last_reed_switch == 0)
  {
    // just restarted, update last_reed_switch 
    // and indicate the next reading is the first
    tank_level.first_reading = 1;
  }
  else if (tank_level.last_reed_switch == decReedSwitch)
  {
    // we're decreasing
    --tank_level.line_length;
    if (tank_level.first_reading)
    {
      --tank_level.line_length;
      tank_level.first_reading = 0;
    }
    if(send_message != 1)
    {
        send_message = 1;
        Clock_start(Clock_handle(&message_trigger_clock));
    }
  }
  else if (tank_level.last_reed_switch == reedSwitch)
  {
      // do nothing
  }
  else
  {
    // we're increasing
    ++tank_level.line_length;
    if (tank_level.first_reading)
    {
      ++tank_level.line_length;
      tank_level.first_reading = 0;
    }
    if(send_message != 1)
    {
        send_message = 1;
        Clock_start(Clock_handle(&message_trigger_clock));
    }
  }
  tank_level.last_reed_switch = reedSwitch;  

}

static void app_resetTankLevel()
{
  // TODO: disable and re-enable interrupts around the next three lines
  tank_level.last_reed_switch = 0;
  tank_level.first_reading = 0;
  tank_level.line_length = 0;
  tank_level.previous_line_length = 0;
  app_setTankLevel();
  Calibrated = 1;
}

static void app_setTankLevel(void)
{
  int32_t length = (int32_t)(tank_level.line_length * SEGMENT_LENGTH / 10); //SEGMENT_LENGTH/10 mm
  TankMeterService_SetParameter(TMS_TANK_LEVEL_ID, TMS_TANK_LEVEL_LEN, &length);  
  app_adv_update_sched();
}

static void app_getBatteryLevel(void)
{
    ADC_Handle   adc;
    ADC_Params   params;
    int_fast16_t res;
    uint16_t adcValue;

    ADC_Params_init(&params);
    adc = ADC_open(Board_ADC1, &params);

    if (adc == NULL) {
        Log_info0("Error initializing ADC channel 1\n");
        while (1);
    }

    res = ADC_convert(adc, &adcValue);

    if (res == ADC_STATUS_SUCCESS)
    {
        // scalar worked out through initial testing
        // TODO: verify scalar
        adcValueMicroVolt = adcValue * BATTERY_COUNTS_TO_MICROVOLTS;
    }
    else
    {
        Log_info0("ADC convert failed\n");
    }

    ADC_close(adc);
}

void app_setBatteryLevel()
{
  app_getBatteryLevel();
  uint32_t batteryLevel = adcValueMicroVolt / 1000;
  TankMeterService_SetParameter(TMS_BATTERY_LEVEL_ID, TMS_BATTERY_LEVEL_LEN, &batteryLevel);
}


/*
 * @brief  Logs a sensor value to flash
 *
 * @note   use a message to call this function from the interrupt:
 *         app_enqueueMsg(APP_LOG_TANK_HISTORICLL, NULL);
 *         it will read the timestamp, tank sensor and battery level
 *         and log them to flash
 *
 * @param  no parameters
 */
static void app_logTankHistoric(void)
{
  uint32_t timestamp;
  uint32_t older_log_time;
  int32_t sensorValue;
  uint32_t batteryLevel;

  app_setBatteryLevel();


  if (!tank_historic.oad && TimeIsSet && Calibrated)
  {
      send_message_periodic++;
      log_message_periodic++;

      timestamp = Timestamp_Get();
      older_log_time = timestamp - OLD_LOG_TIME_INTERVAL;
      //detele older log

      if(tank_historic.notify == 0)
      {
          Semaphore_pend(tank_historic.semHandle, BIOS_WAIT_FOREVER);
          dl_delete(older_log_time);
          dl_rewind();
          Semaphore_post(tank_historic.semHandle);
      }

      if((tank_level.previous_line_length != tank_level.line_length) || (log_message_periodic >= HEARTBEAT_LOG))
      {
          // timestamp = Timestamp_Get();
          sensorValue = (int32_t)(tank_level.line_length * SEGMENT_LENGTH / 10); //SEGMENT_LENGTH/10 mm
          batteryLevel = adcValueMicroVolt / 1000;
          Semaphore_pend(tank_historic.semHandle, BIOS_WAIT_FOREVER);
          dl_append(timestamp, sensorValue, batteryLevel);
          Semaphore_post(tank_historic.semHandle);
          log_message_periodic = 0;
          tank_level.previous_line_length = tank_level.line_length;
      }
      if(send_message_periodic == 144)
      {
          app_enqueueMsg(APP_MSG_SEND, NULL, NULL);
          send_message_periodic = 0;
      }
  }



}


static void app_send_message(void)
{

  uint32_t timestamp;
  int32_t sensorValue;
  uint32_t batteryLevel;

  if (TimeIsSet && Calibrated)
  {
      app_setBatteryLevel();
      timestamp = Timestamp_Get();
      sensorValue = (int32_t)(tank_level.line_length * SEGMENT_LENGTH / 10); //SEGMENT_LENGTH/10 mm
      batteryLevel = adcValueMicroVolt / 1000;
      if (comms_board_present)
      {
        app_comms_reset();
        sigfox_log(timestamp, sensorValue, 0, batteryLevel);
      }
  }
}


/*resets the comms module
 * For sigfox module use before any send command as module should be in deep sleep
 */

static void app_comms_reset(void)
{
    comms_board_reset_timer = false;
    PIN_setOutputValue(commsPinHandle, Board_Comms, 0);
    Clock_start((Clock_Handle)&comms_board_reset_clock);
    while(!comms_board_reset_timer);
    comms_board_reset_timer = false;
    PIN_setOutputValue(commsPinHandle, Board_Comms, 1);
    Clock_start((Clock_Handle)&comms_board_reset_clock);
    while(!comms_board_reset_timer);
}




static void app_startTankHistoric(void)
{
  tank_historic.notify = 1;
  Semaphore_pend(tank_historic.semHandle, BIOS_WAIT_FOREVER);
  dl_fastforward();
  Semaphore_post(tank_historic.semHandle);
  Clock_start((Clock_Handle)&tank_historic_clock);
}

static void app_stopTankHistoric(void)
{
  tank_historic.notify = 0;
  Semaphore_pend(tank_historic.semHandle, BIOS_WAIT_FOREVER);
  dl_rewind();
  Semaphore_post(tank_historic.semHandle);
}

/*
 * @brief  Fetches the next history value from flash and updates the historic level value
 *
 * @note   This uses the dl flash read cursor, which is auto incremented each time we fetch
 *         If dl_fetch_raw fails we assume we are done. We could get the exact count and use
 *         that to determine if we are finished?
 *
 * @param  no parameters
 */
static void app_notifyTankHistoric(void)
{
  if (tank_historic.notify)
  {
    uint8_t notiData[TMS_HISTORIC_LEVEL_LEN];
    // uint8_t notiData[10]; // until we modify the service
    if (app_fetchTankHistoric(notiData, TMS_HISTORIC_LEVEL_LEN) == DL_SUCCESS)
    //if (app_fetchTankHistoric(notiData, 10) == DL_SUCCESS)
    {
      // Send message to application that it should 
      // update the value of the characteristic from Task context
      app_enqueueCharDataMsg(APP_MSG_UPDATE_CHARVAL, 0xFFFF, 
        TANK_METER_SERVICE_SERV_UUID, TMS_HISTORIC_LEVEL_ID, 
        notiData, TMS_HISTORIC_LEVEL_LEN);

      // Start the one-shot timer to process the next history value
      Clock_start((Clock_Handle)&tank_historic_clock);      
    }
  }
}

static int app_fetchTankHistoric(uint8_t *object_value, size_t size)
{
  int res = -1;
  if (!tank_historic.oad)
  {
    Semaphore_pend(tank_historic.semHandle, BIOS_WAIT_FOREVER);
    res = dl_fetch_raw_LIFO(object_value, size);
    rescheck = res;
    Semaphore_post(tank_historic.semHandle);  
  }
  return res;
}

/*
 *  @brief multiple presses check timer handler. 
 */
static void multiple_presses_clockSwiHandler(UArg paramID)
{
    app_enqueueMsg(APP_MULTIPLE_BUTTON, NULL, NULL);
}

/*
 *  @brief multiple presses process.
 *  @details press once, battery led display.
 *           press three times, send sigfox test messages.
 */
static void multiple_presses_process(void)
{
    switch (button_press_cnt)
    {
    case 1:
        app_setBatteryLevel();
        if (adcValueMicroVolt > BATTERY_MIN_LED)
        {
          // show battery still ok long led blink
          app_set_zero_led(1);
          tank_level.led_state = ZERO_LED_STATE_IDLE;
          Util_restartClock(&led_clock, ZERO_LONG_BLINK);
        }
      break;

    case 3:
      app_enqueueMsg(APP_SIGFOX_MSG_TEST, NULL, NULL);
      break;
    
    default:
      break;
    }

    button_press_cnt = 0;
}

/*
  增加1个定时器
  松手的时候计数累加并开启定时器(重新计时，单周期)，大约是时间1秒。
  回调后，判断当前按的次数，根据次数执行。次数的累加值清0.
*/

static void app_onButtonPressed(button_state_t *button_state)
{
  uint32_t timestamp;
  timestamp = Timestamp_Get();

  if (button_state->state)
  {
    // button pressed
    tank_level.zero_button_ts = timestamp;
    app_set_zero_led(0);
  }
  else
  {
    // button released
    uint32_t elapsed; 
    elapsed = timestamp - tank_level.zero_button_ts;

    if (elapsed <= ZERO_LED_UPPER)
    {
      if (elapsed >= ZERO_LED_LOWER)
      {
        // reset (zero) tank level
        app_resetTankLevel();
        app_set_zero_led(1);

        tank_level.led_state = ZERO_LED_STATE_BLINK_1;
        Util_restartClock(&led_clock, ZERO_SHORT_BLINK);
        
      }
      else
      {
        // app_setBatteryLevel();
        // //app_enqueueMsg(APP_MSG_SEND, NULL, NULL);
        // if (adcValueMicroVolt > BATTERY_MIN_LED)
        // {
        //   // show battery still ok long led blink
        //   app_set_zero_led(1);
        //   tank_level.led_state = ZERO_LED_STATE_IDLE;
        //   Util_restartClock(&led_clock, ZERO_LONG_BLINK);
        // }
        button_press_cnt ++;
        Util_restartClock(&multiple_presses_clock, MULTIPLE_BUTTON_CHECK);

      }
    }
  }  
} 

static void app_toggle_zero_led()
{
	switch (tank_level.led_state)
  {
  case ZERO_LED_STATE_IDLE:
    app_set_zero_led(0);
    break;

  case ZERO_LED_STATE_BLINK_1:
    app_set_zero_led(0);
    tank_level.led_state = ZERO_LED_STATE_BLINK_2;
    Util_restartClock(&led_clock, ZERO_SHORT_BLINK);
    break;

  case ZERO_LED_STATE_BLINK_2:
    app_set_zero_led(1);
    tank_level.led_state = ZERO_LED_STATE_IDLE;
    Util_restartClock(&led_clock, ZERO_SHORT_BLINK);
    break;

  case ZERO_LED_STATE_BT:
    app_set_zero_led(1);
    tank_level.led_state = ZERO_LED_STATE_IDLE;
    Util_restartClock(&led_clock, ZERO_BT_ON);
    break;

  case STARTUP_TEST_SIGFOX:
    sigfox_test_led_blink();
    break;
  
  default:
    app_set_zero_led(0);
    tank_level.led_state = ZERO_LED_STATE_IDLE;
    break;
  }
}

static void sigfox_test_led_blink(void)
{
  static uint8_t step = 0;
  static uint8_t state = 0;
  step ++;
  state = !state;
  app_set_zero_led(state);
  Util_restartClock(&led_clock, SIGFOX_TEST_BLINK);
  if(step >= 5)
  {
    step = 0;
    state = 0;
    tank_level.led_state = ZERO_LED_STATE_IDLE;
  }
}

#if CALIBRATE_LED_ENABLE
static void app_calibrate_led()
{
    if(TimeIsSet && Calibrated)
    {
        calibrate_status = CALIBRATE_COMPLETE;
    }

    switch(calibrate_status)
    {
    case ZERO_LED_STATE_IDLE:
        app_set_zero_led(1);
        calibrate_status = ZERO_LED_STATE_BLINK_1;
        Util_restartClock(&calibrate_clock, ZERO_XSHORT_BLINK);
        break;
    case ZERO_LED_STATE_BLINK_1:
        app_set_zero_led(0);
        calibrate_status = ZERO_LED_STATE_BLINK_2;
        Util_restartClock(&calibrate_clock, ZERO_XSHORT_BLINK);
        break;
    case ZERO_LED_STATE_BLINK_2:
        app_set_zero_led(1);
        calibrate_status = ZERO_LED_STATE_BLINK_3;
        Util_restartClock(&calibrate_clock, ZERO_XSHORT_BLINK);
        break;
    case ZERO_LED_STATE_BLINK_3:
        app_set_zero_led(0);
        calibrate_status = ZERO_LED_STATE_BLINK_4;
        Util_restartClock(&calibrate_clock, ZERO_XSHORT_BLINK);
        break;
    case ZERO_LED_STATE_BLINK_4:
        app_set_zero_led(1);
        calibrate_status = ZERO_LED_STATE_BLINK_5;
        Util_restartClock(&calibrate_clock, ZERO_XSHORT_BLINK);
        break;
    case ZERO_LED_STATE_BLINK_5:
        app_set_zero_led(0);
        calibrate_status = ZERO_LED_STATE_BLINK_6;
        Util_restartClock(&calibrate_clock, ZERO_XSHORT_BLINK);
        break;
    case ZERO_LED_STATE_BLINK_6:
        app_set_zero_led(0);
        calibrate_status = ZERO_LED_STATE_IDLE;
        Util_restartClock(&calibrate_clock, ZERO_XLONG_BLINK);
        break;
    case CALIBRATE_COMPLETE:
        app_set_zero_led(0);
        break;
    default:
        calibrate_status = CALIBRATE_COMPLETE;
        app_set_zero_led(0);
        break;

    }




}
#endif


static void app_set_zero_led(uint8_t led_state)
{
    PIN_setOutputValue(ledPinHandle, ZERO_LED, led_state);
    TankMeterService_SetParameter(TMS_LED_ID, TMS_LED_LEN, &led_state);
}

static void app_check_comms_module(void)
{
    uint8_t txBuffer[1];
    uint8_t rxBuffer[20];
    I2C_Handle i2c;
    I2C_Params i2cParams;
    I2C_Transaction i2cTransaction;

    I2C_init();
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(Board_I2C0, &i2cParams);

    if (i2c == NULL)
    {
        comms_board.type = 0;
        comms_board.region = 0;
        comms_board.manufacturer = 0;
        comms_board.version = 0;
        return;
    }

    txBuffer[0] = COMMS_BOARD_I2C_REG_TYPE;
    i2cTransaction.slaveAddress = COMMS_BOARD_I2C_ADDRESS;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 1;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
      comms_board.type = rxBuffer[0];
    }

    txBuffer[0] = COMMS_BOARD_I2C_REG_REGION;
    i2cTransaction.slaveAddress = COMMS_BOARD_I2C_ADDRESS;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 1;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
      comms_board.region = rxBuffer[0];
    }

    txBuffer[0] = COMMS_BOARD_I2C_REG_MANUFACTURER;
    i2cTransaction.slaveAddress = COMMS_BOARD_I2C_ADDRESS;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 1;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
      comms_board.manufacturer = rxBuffer[0];
    }

    txBuffer[0] = COMMS_BOARD_I2C_REG_VERSION;
    i2cTransaction.slaveAddress = COMMS_BOARD_I2C_ADDRESS;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 1;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
      comms_board.version = rxBuffer[0];
    }

    I2C_close(i2c);
}


/*
 * @brief   Convert {0x01, 0x02} to "01:02"
 *
 * @param   src - source byte-array
 * @param   src_len - length of array
 * @param   dst - destination string-array
 * @param   dst_len - length of array
 *
 * @return  array as string
 */
static char *util_convertArrayToHexString(uint8_t const *src, uint8_t src_len,
                                          uint8_t *dst, uint8_t dst_len)
{
  char        hex[] = "0123456789ABCDEF";
  uint8_t     *pStr = dst;
  uint8_t     avail = dst_len-1;

  memset(dst, 0, avail);

  while (src_len && avail > 3)
  {
    if (avail < dst_len-1) { *pStr++ = ':'; avail -= 1; };
    *pStr++ = hex[*src >> 4];
    *pStr++ = hex[*src++ & 0x0F];
    avail -= 2;
    src_len--;
  }

  if (src_len && avail)
    *pStr++ = ':'; // Indicate not all data fit on line.

  return (char *)dst;
}

static void app_send_test_msg(void)
{
    uint32_t timestamp;
    int32_t sensorValue;
    uint32_t batteryLevel;

    app_setBatteryLevel();
    timestamp = Timestamp_Get();
    sensorValue = (int32_t)(tank_level.line_length * SEGMENT_LENGTH / 10); //SEGMENT_LENGTH/10 mm
    batteryLevel = adcValueMicroVolt / 1000;
    app_comms_reset();
    sigfox_log(timestamp, sensorValue, 0, batteryLevel);
}

/*
 * Copyright (c) 2016, Texas Instruments Incorporated
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
 */

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#define xdc_runtime_Log_DISABLE_ALL 1  // Add to disable logs from this file

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/drivers/PIN.h>
#include <ti/mw/display/Display.h>

#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>

// Stack headers
#include <hci_tl.h>
#include <gap.h>
#include <gatt.h>
#include <gapgattserver.h>
#include <gattservapp.h>
#include <osal_snv.h>
#include <gapbondmgr.h>
#include <peripheral.h>
#include <icall_apimsg.h>

#include <devinfoservice.h>

#include "util.h"

#include "Board.h"
#include "project_zero.h"

// Bluetooth Developer Studio services
#include "LED_Service.h"
#include "Button_Service.h"
#include "Data_Service.h"

//t.n.tmp 161102 include SPI
#include <ti/drivers/SPI.h>
//t.n.tmp 161111 include adc
#include "scif_osal_tirtos.h"
#include "scif_framework.h"
#include "scif.h"
#include <stdio.h>

/*********************************************************************
 * CONSTANTS
 */
// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Default pass-code used for pairing.
#define DEFAULT_PASSCODE                      000000

// Task configuration
#define PRZ_TASK_PRIORITY                     1

#ifndef PRZ_TASK_STACK_SIZE
#define PRZ_TASK_STACK_SIZE                   800
#endif

// Internal Events for RTOS application
#define PRZ_STATE_CHANGE_EVT                  0x0001
#define PRZ_CHAR_CHANGE_EVT                   0x0002
#define PRZ_PERIODIC_EVT                      0x0004
#define PRZ_CONN_EVT_END_EVT                  0x0008

//t.n.tmp 161107 delay defined
//TODO check definition
/* Delay */
//#ifdef TI_DRIVERS_I2C_INCLUDED
//#define delay_ms(i) Task_sleep( ((i) * 1000) / Clock_tickPeriod )
//#define MS_2_TICKS(ms) ( ((ms) * 1000) / Clock_tickPeriod )
//#else
#define delay_ms(i) ( CPUdelay(12000*(i)) )
//#endif
//t.n.tmp kokomade

/*********************************************************************
 * TYPEDEFS
 */
// Types of messages that can be sent to the user application task from other
// tasks or interrupts. Note: Messages from BLE Stack are sent differently.
typedef enum
{
  APP_MSG_SERVICE_WRITE = 0,   /* A characteristic value has been written     */
  APP_MSG_SERVICE_CFG,         /* A characteristic configuration has changed  */
  APP_MSG_UPDATE_CHARVAL,      /* Request from ourselves to update a value    */
  APP_MSG_GAP_STATE_CHANGE,    /* The GAP / connection state has changed      */
  APP_MSG_BUTTON_DEBOUNCED,    /* A button has been debounced with new value  */
  APP_MSG_SEND_PASSCODE,       /* A pass-code/PIN is requested during pairing */
} app_msg_types_t;

// Struct for messages sent to the application task
typedef struct
{
  Queue_Elem       _elem;
  app_msg_types_t  type;
  uint8_t          pdu[];
} app_msg_t;

// Struct for messages about characteristic data
typedef struct
{
  uint16_t svcUUID; // UUID of the service
  uint16_t dataLen; //
  uint8_t  paramID; // Index of the characteristic
  uint8_t  data[];  // Flexible array member, extended to malloc - sizeof(.)
} char_data_t;

// Struct for message about sending/requesting passcode from peer.
typedef struct
{
  uint16_t connHandle;
  uint8_t  uiInputs;
  uint8_t  uiOutputs;
} passcode_req_t;

// Struct for message about button state
typedef struct
{
  PIN_Id   pinId;
  uint8_t  state;
} button_state_t;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Queue object used for application messages.
static Queue_Struct applicationMsgQ;
static Queue_Handle hApplicationMsgQ;

// Task configuration
Task_Struct przTask;
Char przTaskStack[PRZ_TASK_STACK_SIZE];


// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // No scan response data provided.
  0x00 // Placeholder to keep the compiler happy.
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) or general
  // discoverable mode (advertises indefinitely), depending
  // on the DEFAULT_DISCOVERY_MODE define.
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // complete name
  13,
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'P', 'r', 'o', 'j', 'e', 'c', 't', ' ', 'Z', 'e', 'r', 'o',

};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Project Zero";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;


/* Pin driver handles */
static PIN_Handle buttonPinHandle;
static PIN_Handle ledPinHandle;
//t.n.tmp 161107 DAC chip select pin handle
static PIN_Handle daccsPinHandle;
//t.n.tmp 161111 ADC pin handle
static PIN_Handle adcPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State buttonPinState;
static PIN_State ledPinState;
//t.n.tmp 161107 DAC chip select pin state definition
static PIN_State daccsPinState;
//t.n.tmp 161111 ADC pin state definition
static PIN_State adcPinState;

/*
 * Initial LED pin configuration table
 *   - LEDs Board_LED0 & Board_LED1 are off.
 */
PIN_Config ledPinTable[] = {
  Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

//t.n.tmp 161107 DAC chip select pin configulation table
/*
 * Initial DAC CS pin configuration table
 *   - DAC CS are HI
 */
PIN_Config daccsPinTable[] = {
  Board_SPI0_DAC_CSN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  PIN_TERMINATE
};

//t.n.tmp 161111 ADC pin configuration table
/*
 * Initial ADC pin configuration table
 *   - ADC input
 */
PIN_Config adcPinTable[] = {
  Board_ADC_IN0      | PIN_INPUT_DIS | PIN_GPIO_OUTPUT_DIS ,
  PIN_TERMINATE
};

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config buttonPinTable[] = {
    Board_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

// Clock objects for debouncing the buttons
static Clock_Struct button0DebounceClock;
static Clock_Struct button1DebounceClock;

// State of the buttons
static uint8_t button0State = 0;
static uint8_t button1State = 0;

//t.n.tmp 161115 adc data sengen
#define SWV_ADC_BUFFER_SIZE 512 //ADC buffer size
static unsigned short adcDataReady = 0;
static unsigned short adcDataDisplayed = 0;
static unsigned short adcValue[SWV_ADC_BUFFER_SIZE];
//t.n.tmp 170227 adc data definition
#define AMPERO_ADC_BUFFER_SIZE 32 //ADC buffer size
static unsigned short amperoAdcValue[AMPERO_ADC_BUFFER_SIZE];

//t.n.tmp 161206 control definition
//SWV control hikisuu
#define SWV_CTRL_LED_OFF        0x00
#define SWV_CTRL_LED_ON         0x01
#define SWV_CTRL_SPIADC_INIT    0x02
#define SWV_CTRL_SWV_MEAS       0x03
#define SWV_CTRL_SWV_DATADISP   0x04
#define SWV_CTRL_CV_MEAS        0x05
#define SWV_CTRL_CV_DATADISP    0x06
#define SWV_CTRL_SWV_MEAS05     0x07
//t.n.tmp 170131
#define SWV_CTRL_AMPERO_MEAS    0x08
#define SWV_CTRL_SWV_MEAS06     0x09
#define SWV_CTRL_AMPERO_STOP    0x0A
//t.n.tmp 170210
#define SWV_CTRL_SWV_MEAS07     0x0B

//t.n.tmp 161208
//SWV param address
#define SWV_PARAM_ADDR_INIT_WAIT       0x0001
#define SWV_PARAM_ADDR_FREQ            0x0002
#define SWV_PARAM_ADDR_PERIOD          0x0003
#define SWV_PARAM_ADDR_HIGH_DURATION   0x0004
#define SWV_PARAM_ADDR_INIT_VOLTAGE    0x0005
#define SWV_PARAM_ADDR_FINAL_VOLTAGE   0x0006
#define SWV_PARAM_ADDR_AMPLITUDE       0x0007
#define SWV_PARAM_ADDR_STEP_VOLTAGE    0x0008
#define SWV_PARAM_ADDR_OFFSET_VOLTAGE  0x0009
#define SWV_PARAM_ADDR_INIT_VOLTAGE_OFFSET    0x000A
#define SWV_PARAM_ADDR_FINAL_VOLTAGE_OFFSET   0x000B

//t.n.tmp 170131
//AMPERO param address
#define SWV_PARAM_ADDR_AMPERO_POINTS   0x000C
#define SWV_PARAM_ADDR_AMPERO_PERIOD   0x000D
#define SWV_PARAM_ADDR_AMPERO_OFFSET_VOLTAGE    0x000E
#define SWV_PARAM_ADDR_AMPERO_POTENTIAL_OFFSET  0x000F

//SWV Parameters (default)
#define SWV_INIT_WAIT       500     //wait time at initial in 10ms
#define SWV_FREQ            10       //frequency in Hz
#define SWV_PERIOD          1000/SWV_FREQ   //period in ms
#define SWV_HIGH_DURATION   SWV_PERIOD/2    //high duration in ms
#define SWV_INIT_VOLTAGE    300     //initial voltage in mV
#define SWV_FINAL_VOLTAGE   1200    //initial voltage in mV
#define SWV_AMPLITUDE       250     //amplitude in mV
#define SWV_STEP_VOLTAGE    40      //step voltage in mV
#define SWV_OFFSET_VOLTAGE  1500    //offset voltage in mV

static unsigned short swvInitWait       = SWV_INIT_WAIT;
static unsigned short swvFreq           = SWV_FREQ;
static unsigned short swvPeriod         = SWV_PERIOD;
static unsigned short swvHighDuration   = SWV_HIGH_DURATION;
static unsigned short swvInitVoltage    = SWV_INIT_VOLTAGE;
static unsigned short swvFinalVoltage   = SWV_FINAL_VOLTAGE;
static unsigned short swvAmplitude      = SWV_AMPLITUDE;
static unsigned short swvStepVoltage    = SWV_STEP_VOLTAGE;
static unsigned short swvOffsetVoltage  = SWV_OFFSET_VOLTAGE;

static unsigned short swvInitVoltageOffset    = SWV_INIT_VOLTAGE + SWV_OFFSET_VOLTAGE;
static unsigned short swvFinalVoltageOffset   = SWV_FINAL_VOLTAGE + SWV_OFFSET_VOLTAGE;

//t.n.tmp 161209
//CV Parameters (default)
#define CV_SCAN_RATE        100     //scan rate (mV/sec)
#define CV_LOWER_POTENTIAL  0    //lower potential in mV
#define CV_UPPER_POTENTIAL  800     //upper potential in mV
#define CV_STEP_POTENTIAL   5      //step potential in mV
#define CV_OFFSET_VOLTAGE   2200    //offset voltage in mV

static int cvScanRate        = CV_SCAN_RATE;
static int cvLowerPotential  = CV_LOWER_POTENTIAL;
static int cvUpperPotential  = CV_UPPER_POTENTIAL;
static int cvStepPotential   = CV_STEP_POTENTIAL;
static int cvOffsetVoltage   = CV_OFFSET_VOLTAGE;
//t.n.tmp kokomade

//t.n.tmp 170131
//Ampero parameters (default)
#define AMPERO_POINTS           60      //measurement points
#define AMPERO_PERIOD           1000    //period in ms
#define AMPERO_OFFSET_VOLTAGE   1500    //offset voltage in mV
#define AMPERO_POTENTIAL_OFFSET 1400    //potential in mV

static unsigned short amperoPoints          = AMPERO_POINTS;
static unsigned short amperoPeriod          = AMPERO_PERIOD;
static unsigned short amperoOffsetVoltage   = AMPERO_OFFSET_VOLTAGE;
static unsigned short amperoPotentialOffset = AMPERO_POTENTIAL_OFFSET;

static unsigned short amperoStopFlag        = 1;
//t.n.tmp kokomade

// Global display handle
Display_Handle dispHandle;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void ProjectZero_init( void );
static void ProjectZero_taskFxn(UArg a0, UArg a1);

static void user_processApplicationMessage(app_msg_t *pMsg);
static uint8_t ProjectZero_processStackMsg(ICall_Hdr *pMsg);
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg);

static void ProjectZero_sendAttRsp(void);
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg);
static void ProjectZero_freeAttRsp(uint8_t status);

static void user_processGapStateChangeEvt(gaprole_States_t newState);
static void user_gapStateChangeCB(gaprole_States_t newState);
static void user_gapBondMgr_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                       uint8_t uiInputs, uint8_t uiOutputs);
static void user_gapBondMgr_pairStateCB(uint16_t connHandle, uint8_t state,
                                        uint8_t status);

static void buttonDebounceSwiFxn(UArg buttonId);
static void user_handleButtonPress(button_state_t *pState);

// Generic callback handlers for value changes in services.
static void user_service_ValueChangeCB( uint16_t connHandle, uint16_t svcUuid, uint8_t paramID, uint8_t *pValue, uint16_t len );
static void user_service_CfgChangeCB( uint16_t connHandle, uint16_t svcUuid, uint8_t paramID, uint8_t *pValue, uint16_t len );

// Task context handlers for generated services.
static void user_LedService_ValueChangeHandler(char_data_t *pCharData);
static void user_ButtonService_CfgChangeHandler(char_data_t *pCharData);
static void user_DataService_ValueChangeHandler(char_data_t *pCharData);
static void user_DataService_CfgChangeHandler(char_data_t *pCharData);

// Task handler for sending notifications.
static void user_updateCharVal(char_data_t *pCharData);

// Utility functions
static void user_enqueueRawAppMsg(app_msg_types_t appMsgType, uint8_t *pData, uint16_t len );
static void user_enqueueCharDataMsg(app_msg_types_t appMsgType, uint16_t connHandle,
                                    uint16_t serviceUUID, uint8_t paramID,
                                    uint8_t *pValue, uint16_t len);
static void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId);

static char *Util_convertArrayToHexString(uint8_t const *src, uint8_t src_len,
                                          uint8_t *dst, uint8_t dst_len);
static char *Util_getLocalNameStr(const uint8_t *data);

// t.n.tmp functions
unsigned short calcDacDataLSB(unsigned short swvOffsetVoltageLSB, unsigned short swvVreDataLSB);
unsigned short calcDacDataLSBInt(int cvOffsetVoltageLSB, int cvVreDataLSB);
int AMPERO_Meas02(void);   //t.n.tmp 170227
int AMPERO_Meas01(void);   //t.n.tmp 170131
int CV_Meas01(void);
int SWV_Meas07(void);   //t.n.tmp 170210
int SWV_Meas06(void);   //t.n.tmp 170131
int SWV_Meas05(void);
int SWV_Meas04(void);
int SWV_Meas03(void);
int SWV_Meas02(void);
int SWV_Meas01(void);
int ADC_Data_Disp02(void);
int ADC_Data_Disp01(void);
int ADC_Init(void);
unsigned short ADC_Sample(void);
int DAC_SPI_SWV(void);
int DAC_SPI_Init(void);
int DAC_SPI_WriteUpdateA(unsigned short data);
int DAC_SPI_WriteReg(unsigned char cmd, unsigned short data);
int bspSpiWrite(const uint8_t *buf, size_t len);
int bspSpiRead(uint8_t *buf, size_t len);
int bspSpiWriteRead(uint8_t *buf, uint8_t wlen, uint8_t rlen);
void bspSpiOpen(void);
void bspSpiClose(void);
void scTaskAlertCallback(void);
void scCtrlReadyCallback(void);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t user_gapRoleCBs =
{
  user_gapStateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t user_bondMgrCBs =
{
  user_gapBondMgr_passcodeCB, // Passcode callback
  user_gapBondMgr_pairStateCB // Pairing / Bonding state Callback
};

/*
 * Callbacks in the user application for events originating from BLE services.
 */
// LED Service callback handler.
// The type LED_ServiceCBs_t is defined in LED_Service.h
static LedServiceCBs_t user_LED_ServiceCBs =
{
  .pfnChangeCb    = user_service_ValueChangeCB, // Characteristic value change callback handler
  .pfnCfgChangeCb = NULL, // No notification-/indication enabled chars in LED Service
};

// Button Service callback handler.
// The type Button_ServiceCBs_t is defined in Button_Service.h
static ButtonServiceCBs_t user_Button_ServiceCBs =
{
  .pfnChangeCb    = NULL, // No writable chars in Button Service, so no change handler.
  .pfnCfgChangeCb = user_service_CfgChangeCB, // Noti/ind configuration callback handler
};

// Data Service callback handler.
// The type Data_ServiceCBs_t is defined in Data_Service.h
static DataServiceCBs_t user_Data_ServiceCBs =
{
  .pfnChangeCb    = user_service_ValueChangeCB, // Characteristic value change callback handler
  .pfnCfgChangeCb = user_service_CfgChangeCB, // Noti/ind configuration callback handler
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*
 * @brief   Task creation function for the user task.
 *
 * @param   None.
 *
 * @return  None.
 */
void ProjectZero_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = przTaskStack;
  taskParams.stackSize = PRZ_TASK_STACK_SIZE;
  taskParams.priority = PRZ_TASK_PRIORITY;

  Task_construct(&przTask, ProjectZero_taskFxn, &taskParams, NULL);
}

/*
 * @brief   Called before the task loop and contains application-specific
 *          initialization of the BLE stack, hardware setup, power-state
 *          notification if used, and BLE profile/service initialization.
 *
 * @param   None.
 *
 * @return  None.
 */
static void ProjectZero_init(void)
{
  // ******************************************************************
  // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages via ICall to Stack.
  ICall_registerApp(&selfEntity, &sem);

  Log_info0("Initializing the user task, hardware, BLE stack and services.");

  // Initialize queue for application messages.
  // Note: Used to transfer control to application thread from e.g. interrupts.
  Queue_construct(&applicationMsgQ, NULL);
  hApplicationMsgQ = Queue_handle(&applicationMsgQ);

  // ******************************************************************
  // Hardware initialization
  // ******************************************************************

  // Open LED pins
  ledPinHandle = PIN_open(&ledPinState, ledPinTable);
  if(!ledPinHandle) {
    Log_error0("Error initializing board LED pins");
    Task_exit();
  }

  PIN_setOutputValue(ledPinHandle, Board_LED0, 1);

  //t.n.tmp 161107 Open DAC chip select pins
  // Open DAC chip select pins
  daccsPinHandle = PIN_open(&daccsPinState, daccsPinTable);
  if(!daccsPinHandle) {
    Log_error0("Error initializing board DAC CS pins");
    Task_exit();
  }
  //t.n.tmp kokomade
  //t.n.tmp 161111 Open ADC pins
  // Open ADC chip select pins
  adcPinHandle = PIN_open(&adcPinState, adcPinTable);
  if(!adcPinHandle) {
    Log_error0("Error initializing board ADC pins");
    Task_exit();
  }
  //t.n.tmp kokomade

  buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
  if(!buttonPinHandle) {
    Log_error0("Error initializing button pins");
    Task_exit();
  }

  // Setup callback for button pins
  if (PIN_registerIntCb(buttonPinHandle, &buttonCallbackFxn) != 0) {
    Log_error0("Error registering button callback function");
    Task_exit();
  }

  // Create the debounce clock objects for Button 0 and Button 1
  Clock_Params clockParams;
  Clock_Params_init(&clockParams);

  // Both clock objects use the same callback, so differentiate on argument
  // given to the callback in Swi context
  clockParams.arg = Board_BUTTON0;

  // Initialize to 50 ms timeout when Clock_start is called.
  // Timeout argument is in ticks, so convert from ms to ticks via tickPeriod.
  Clock_construct(&button0DebounceClock, buttonDebounceSwiFxn,
                  50 * (1000/Clock_tickPeriod),
                  &clockParams);

  // Second button
  clockParams.arg = Board_BUTTON1;
  Clock_construct(&button1DebounceClock, buttonDebounceSwiFxn,
                  50 * (1000/Clock_tickPeriod),
                  &clockParams);

  //t.n.tmp 170106 SPI, ADC init
  Log_info0("SPI, ADC init");
  DAC_SPI_Init();
  ADC_Init();

  // ******************************************************************
  // BLE Stack initialization
  // ******************************************************************

  // Setup the GAP Peripheral Role Profile
  uint8_t initialAdvertEnable = TRUE;  // Advertise on power-up

  // By setting this to zero, the device will go into the waiting state after
  // being discoverable. Otherwise wait this long [ms] before advertising again.
  uint16_t advertOffTime = 0; // miliseconds

  // Set advertisement enabled.
  GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                       &initialAdvertEnable);

  // Configure the wait-time before restarting advertisement automatically
  GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                       &advertOffTime);

  // Initialize Scan Response data
  GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);

  // Initialize Advertisement data
  GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

  Log_info1("Name in advertData array: \x1b[33m%s\x1b[0m",
            (IArg)Util_getLocalNameStr(advertData));

  // Set advertising interval
  uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

  GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
  GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
  GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
  GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);

  // Set duration of advertisement before stopping in Limited adv mode.
  GAP_SetParamValue(TGAP_LIM_ADV_TIMEOUT, 30); // Seconds

  // ******************************************************************
  // BLE Bond Manager initialization
  // ******************************************************************
  uint32_t passkey = 0; // passkey "000000"
  uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
  uint8_t mitm = TRUE;
  uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
  uint8_t bonding = TRUE;

  GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                          &passkey);
  GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
  GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
  GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
  GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);

  // ******************************************************************
  // BLE Service initialization
  // ******************************************************************

  // Add services to GATT server
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service

  // Set the device name characteristic in the GAP Profile
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Add services to GATT server and give ID of this task for Indication acks.
  LedService_AddService( selfEntity );
  ButtonService_AddService( selfEntity );
  DataService_AddService( selfEntity );

  // Register callbacks with the generated services that
  // can generate events (writes received) to the application
  LedService_RegisterAppCBs( &user_LED_ServiceCBs );
  ButtonService_RegisterAppCBs( &user_Button_ServiceCBs );
  DataService_RegisterAppCBs( &user_Data_ServiceCBs );

  // Placeholder variable for characteristic intialization
  uint8_t initVal[40] = {0};
  uint8_t initString[] = "This is a pretty long string, isn't it!";

  // Initalization of characteristics in LED_Service that can provide data.
  LedService_SetParameter(LS_LED0_ID, LS_LED0_LEN, initVal);
  LedService_SetParameter(LS_LED1_ID, LS_LED1_LEN, initVal);

  // Initalization of characteristics in Button_Service that can provide data.
  ButtonService_SetParameter(BS_BUTTON0_ID, BS_BUTTON0_LEN, initVal);
  ButtonService_SetParameter(BS_BUTTON1_ID, BS_BUTTON1_LEN, initVal);

  // Initalization of characteristics in Data_Service that can provide data.
  DataService_SetParameter(DS_STRING_ID, sizeof(initString), initString);
  DataService_SetParameter(DS_STREAM_ID, DS_STREAM_LEN, initVal);

  // Start the stack in Peripheral mode.
  VOID GAPRole_StartDevice(&user_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&user_bondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);
}


/*
 * @brief   Application task entry point.
 *
 *          Invoked by TI-RTOS when BIOS_start is called. Calls an init function
 *          and enters an infinite loop waiting for messages.
 *
 *          Messages can be either directly from the BLE stack or from user code
 *          like Hardware Interrupt (Hwi) or a callback function.
 *
 *          The reason for sending messages to this task from e.g. Hwi's is that
 *          some RTOS and Stack APIs are not available in callbacks and so the
 *          actions that may need to be taken is dispatched to this Task.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void ProjectZero_taskFxn(UArg a0, UArg a1)
{
    // Initialize application
    ProjectZero_init();

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      // Check if we got a signal because of a stack message
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for event flags received (event signature 0xffff)
          if (pEvt->signature == 0xffff)
          {
            // Event received when a connection event is completed
            if (pEvt->event_flag & PRZ_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              ProjectZero_sendAttRsp();
            }
          }
          else // It's a message from the stack and not an event.
          {
            // Process inter-task message
            safeToDealloc = ProjectZero_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // Process messages sent from another task or another context.
      while (!Queue_empty(hApplicationMsgQ))
      {
        app_msg_t *pMsg = Queue_dequeue(hApplicationMsgQ);

        // Process application-layer message probably sent from ourselves.
        user_processApplicationMessage(pMsg);

        // Free the received message.
        ICall_free(pMsg);
      }
    }
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
static void user_processApplicationMessage(app_msg_t *pMsg)
{
  char_data_t *pCharData = (char_data_t *)pMsg->pdu;

  switch (pMsg->type)
  {
    case APP_MSG_SERVICE_WRITE: /* Message about received value write */
      /* Call different handler per service */
      switch(pCharData->svcUUID) {
        case LED_SERVICE_SERV_UUID:
          user_LedService_ValueChangeHandler(pCharData);
          break;
        case DATA_SERVICE_SERV_UUID:
          user_DataService_ValueChangeHandler(pCharData);
          break;

      }
      break;

    case APP_MSG_SERVICE_CFG: /* Message about received CCCD write */
      /* Call different handler per service */
      switch(pCharData->svcUUID) {
        case BUTTON_SERVICE_SERV_UUID:
          user_ButtonService_CfgChangeHandler(pCharData);
          break;
        case DATA_SERVICE_SERV_UUID:
          user_DataService_CfgChangeHandler(pCharData);
          break;
      }
      break;

    case APP_MSG_UPDATE_CHARVAL: /* Message from ourselves to send  */
      user_updateCharVal(pCharData);
      break;

    case APP_MSG_GAP_STATE_CHANGE: /* Message that GAP state changed  */
      user_processGapStateChangeEvt( *(gaprole_States_t *)pMsg->pdu );
      break;

    case APP_MSG_SEND_PASSCODE: /* Message about pairing PIN request */
      {
        passcode_req_t *pReq = (passcode_req_t *)pMsg->pdu;
        Log_info2("BondMgr Requested passcode. We are %s passcode %06d",
                  (IArg)(pReq->uiInputs?"Sending":"Displaying"),
                  DEFAULT_PASSCODE);
        // Send passcode response.
        GAPBondMgr_PasscodeRsp(pReq->connHandle, SUCCESS, DEFAULT_PASSCODE);
      }
      break;

    case APP_MSG_BUTTON_DEBOUNCED: /* Message from swi about pin change */
      {
        button_state_t *pButtonState = (button_state_t *)pMsg->pdu;
        user_handleButtonPress(pButtonState);
      }
      break;
  }
}


/******************************************************************************
 *****************************************************************************
 *
 *  Handlers of system/application events deferred to the user Task context.
 *  Invoked from the application Task function above.
 *
 *  Further down you can find the callback handler section containing the
 *  functions that defer their actions via messages to the application task.
 *
 ****************************************************************************
 *****************************************************************************/


/*
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void user_processGapStateChangeEvt(gaprole_States_t newState)
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
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // Display device address
        char *cstr_ownAddress = Util_convertBdAddr2Str(ownAddress);
        Log_info1("GAP is started. Our address: \x1b[32m%s\x1b[0m", (IArg)cstr_ownAddress);
      }
      break;

    case GAPROLE_ADVERTISING:
      Log_info0("Advertising");
      break;

    case GAPROLE_CONNECTED:
      {
        uint8_t peerAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

        char *cstr_peerAddress = Util_convertBdAddr2Str(peerAddress);
        Log_info1("Connected. Peer address: \x1b[32m%s\x1b[0m", (IArg)cstr_peerAddress);
       }
      break;

    case GAPROLE_CONNECTED_ADV:
      Log_info0("Connected and advertising");
      break;

    case GAPROLE_WAITING:
      Log_info0("Disconnected / Idle");
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      Log_info0("Connection timed out");
      break;

    case GAPROLE_ERROR:
      Log_info0("Error");
      break;

    default:
      break;
  }
}


/*
 * @brief   Handle a debounced button press or release in Task context.
 *          Invoked by the taskFxn based on a message received from a callback.
 *
 * @see     buttonDebounceSwiFxn
 * @see     buttonCallbackFxn
 *
 * @param   pState  pointer to button_state_t message sent from debounce Swi.
 *
 * @return  None.
 */
static void user_handleButtonPress(button_state_t *pState)
{
  Log_info2("%s %s",
    (IArg)(pState->pinId == Board_BUTTON0?"Button 0":"Button 1"),
    (IArg)(pState->state?"\x1b[32mpressed\x1b[0m":
                         "\x1b[33mreleased\x1b[0m"));

  // Update the service with the new value.
  // Will automatically send notification/indication if enabled.
  switch (pState->pinId)
  {
    case Board_BUTTON0:
      ButtonService_SetParameter(BS_BUTTON0_ID,
                                 sizeof(pState->state),
                                 &pState->state);
      break;
    case Board_BUTTON1:
      ButtonService_SetParameter(BS_BUTTON1_ID,
                                 sizeof(pState->state),
                                 &pState->state);
      break;
  }
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
void user_LedService_ValueChangeHandler(char_data_t *pCharData)
{
  static uint8_t pretty_data_holder[16]; // 5 bytes as hex string "AA:BB:CC:DD:EE"

    //t.n.tmp 161208
    unsigned short swvParamAddress;
    unsigned short swvParamData;

  Util_convertArrayToHexString(pCharData->data, pCharData->dataLen,
                               pretty_data_holder, sizeof(pretty_data_holder));

  switch (pCharData->paramID)
  {
    case LS_LED0_ID:
      Log_info3("Value Change msg: %s %s: %s",
                (IArg)"LED Service",
                (IArg)"LED0",
                (IArg)pretty_data_holder);

      // Do something useful with pCharData->data here
      // -------------------------
        //t.n.tmp 161206 kokokara
        switch (pCharData->data[0])
        {
            case SWV_CTRL_LED_OFF:
                PIN_setOutputValue(ledPinHandle, Board_LED0, pCharData->data[0]);
                break;
            case SWV_CTRL_LED_ON:
                PIN_setOutputValue(ledPinHandle, Board_LED0, pCharData->data[0]);
                break;
            case SWV_CTRL_SPIADC_INIT: //
                Log_info0("SPI, ADC init");
                DAC_SPI_Init();
                ADC_Init();
                break;
            case SWV_CTRL_SWV_MEAS:
                DataService_SetParameter(DS_STRING_ID, sizeof("Measuring SWV!!!"), "Measuring SWV!!!");
                SWV_Meas04();
                DataService_SetParameter(DS_STRING_ID, sizeof("SWV Done!!!"), "SWV Done!!!");
                break;
            case SWV_CTRL_SWV_MEAS05:
                DataService_SetParameter(DS_STRING_ID, sizeof("Measuring SWV(05)!!!"), "Measuring SWV(05)!!!");
                SWV_Meas05();
                DataService_SetParameter(DS_STRING_ID, sizeof("SWV Done!!!"), "SWV Done!!!");
                break;
            case SWV_CTRL_SWV_MEAS06:   //t.n.tmp 170131
                DataService_SetParameter(DS_STRING_ID, sizeof("Measuring SWV(06)!!!"), "Measuring SWV(06)!!!");
                SWV_Meas06();
                DataService_SetParameter(DS_STRING_ID, sizeof("SWV Done!!!"), "SWV Done!!!");
                break;
            case SWV_CTRL_SWV_MEAS07:   //t.n.tmp 170210
                DataService_SetParameter(DS_STRING_ID, sizeof("Measuring SWV(07)!!!"), "Measuring SWV(07)!!!");
                SWV_Meas07();
                DataService_SetParameter(DS_STRING_ID, sizeof("SWV Done!!!"), "SWV Done!!!");
                break;
            case SWV_CTRL_SWV_DATADISP:
                if (adcDataReady >= 1)
                {
                    ADC_Data_Disp02();
                }
                else
                {
                }
                break;
            case SWV_CTRL_CV_MEAS:
                DataService_SetParameter(DS_STRING_ID, sizeof("Measuring CV!!!"), "Measuring CV!!!");
                CV_Meas01();
                DataService_SetParameter(DS_STRING_ID, sizeof("CV Done!!!"), "CV Done!!!");
                break;
            case SWV_CTRL_CV_DATADISP:
                if (adcDataReady >= 1)
                {
                    ADC_Data_Disp02();
                }
                else
                {
                }
                break;
            case SWV_CTRL_AMPERO_MEAS:  //t.n.tmp 170131
                DataService_SetParameter(DS_STRING_ID, sizeof("Measuring Ampero!!!"), "Measuring Ampero!!!");
                //AMPERO_Meas01();
                AMPERO_Meas02();    //t.n.tmp 170227
                break;
            case SWV_CTRL_AMPERO_STOP:  //t.n.tmp 170131
                amperoStopFlag = 1;
                break;
        }
        //t.n.tmp 161206 kokomade

      // Set the output value equal to the received value. 0 is off, not 0 is on
      //t.n.tmp 161206
      //PIN_setOutputValue(ledPinHandle, Board_LED0, pCharData->data[0]);
      //Log_info2("Turning %s %s",
        //        (IArg)"\x1b[31mLED0\x1b[0m",
        //        (IArg)(pCharData->data[0]?"on":"off"));
      break;

    case LS_LED1_ID:
      Log_info3("Value Change msg: %s %s: %s",
                (IArg)"LED Service",
                (IArg)"LED1",
                (IArg)pretty_data_holder);

      // Do something useful with pCharData->data here
      // -------------------------
        //t.n.tmp 161208 kokokara
        swvParamAddress = (pCharData->data[1])*256 + pCharData->data[0];
        swvParamData = (pCharData->data[3])*256 + pCharData->data[2];
        Log_info4("SWV Parameter --- %d, %d, %d, %d",pCharData->data[0], pCharData->data[1],pCharData->data[2], pCharData->data[3]);
        Log_info2("SWV Parameter --- Address: 0x%x, Data: %d",swvParamAddress, swvParamData);

        switch (swvParamAddress)
        {
            case SWV_PARAM_ADDR_INIT_WAIT:
                swvInitWait = swvParamData;
                break;
            case SWV_PARAM_ADDR_FREQ:
                swvFreq = swvParamData;
                swvPeriod = 1000/swvFreq;
                swvHighDuration = swvPeriod/2;
                break;
            case SWV_PARAM_ADDR_INIT_VOLTAGE:
                swvInitVoltage = swvParamData;
                break;
            case SWV_PARAM_ADDR_FINAL_VOLTAGE:
                swvFinalVoltage = swvParamData;
                break;
            case SWV_PARAM_ADDR_AMPLITUDE:
                swvAmplitude = swvParamData;
                break;
            case SWV_PARAM_ADDR_STEP_VOLTAGE:
                swvStepVoltage = swvParamData;
                break;
            case SWV_PARAM_ADDR_OFFSET_VOLTAGE:
                swvOffsetVoltage = swvParamData;
                break;
            case SWV_PARAM_ADDR_INIT_VOLTAGE_OFFSET:
                swvInitVoltageOffset = swvParamData;
                break;
            case SWV_PARAM_ADDR_FINAL_VOLTAGE_OFFSET:
                swvFinalVoltageOffset = swvParamData;
                break;
            //ampero    //t.n.tmp 170131
            case SWV_PARAM_ADDR_AMPERO_POINTS:
                amperoPoints = swvParamData;
                break;
            case SWV_PARAM_ADDR_AMPERO_PERIOD:
                amperoPeriod = swvParamData;
                break;
            case SWV_PARAM_ADDR_AMPERO_OFFSET_VOLTAGE:
                amperoOffsetVoltage = swvParamData;
                break;
            case SWV_PARAM_ADDR_AMPERO_POTENTIAL_OFFSET:
                amperoPotentialOffset = swvParamData;
                break;
        }
        //t.n.tmp 161208 kokomade

      // Set the output value equal to the received value. 0 is off, not 0 is on
      //t.n.tmp 161208 comment out
      //PIN_setOutputValue(ledPinHandle, Board_LED1, pCharData->data[0]);
      //Log_info2("Turning %s %s",
        //        (IArg)"\x1b[32mLED1\x1b[0m",
        //        (IArg)(pCharData->data[0]?"on":"off"));
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
void user_ButtonService_CfgChangeHandler(char_data_t *pCharData)
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
    case BS_BUTTON0_ID:
      Log_info3("CCCD Change msg: %s %s: %s",
                (IArg)"Button Service",
                (IArg)"BUTTON0",
                (IArg)configValString);
      // -------------------------
      // Do something useful with configValue here. It tells you whether someone
      // wants to know the state of this characteristic.
      // ...
      break;

    case BS_BUTTON1_ID:
      Log_info3("CCCD Change msg: %s %s: %s",
                (IArg)"Button Service",
                (IArg)"BUTTON1",
                (IArg)configValString);
      // -------------------------
      // Do something useful with configValue here. It tells you whether someone
      // wants to know the state of this characteristic.
      // ...
      break;
  }
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
void user_DataService_ValueChangeHandler(char_data_t *pCharData)
{
  // Value to hold the received string for printing via Log, as Log printouts
  // happen in the Idle task, and so need to refer to a global/static variable.
  static uint8_t received_string[DS_STRING_LEN] = {0};

  switch (pCharData->paramID)
  {
    case DS_STRING_ID:
      // Do something useful with pCharData->data here
      // -------------------------
      // Copy received data to holder array, ensuring NULL termination.
      memset(received_string, 0, DS_STRING_LEN);
      memcpy(received_string, pCharData->data, DS_STRING_LEN-1);
      // Needed to copy before log statement, as the holder array remains after
      // the pCharData message has been freed and reused for something else.
      Log_info3("Value Change msg: %s %s: %s",
                (IArg)"Data Service",
                (IArg)"String",
                (IArg)received_string);
      break;

    case DS_STREAM_ID:
      Log_info3("Value Change msg: Data Service Stream: %02x:%02x:%02x...",
                (IArg)pCharData->data[0],
                (IArg)pCharData->data[1],
                (IArg)pCharData->data[2]);
      // -------------------------
      // Do something useful with pCharData->data here
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
void user_DataService_CfgChangeHandler(char_data_t *pCharData)
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
    case DS_STREAM_ID:
      Log_info3("CCCD Change msg: %s %s: %s",
                (IArg)"Data Service",
                (IArg)"Stream",
                (IArg)configValString);
      // -------------------------
      // Do something useful with configValue here. It tells you whether someone
      // wants to know the state of this characteristic.
      // ...
      break;
  }
}


/*
 * @brief   Process an incoming BLE stack message.
 *
 *          This could be a GATT message from a peer device like acknowledgement
 *          of an Indication we sent, or it could be a response from the stack
 *          to an HCI message that the user application sent.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t ProjectZero_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = ProjectZero_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            Log_info0("HCI Command Complete Event received");
            break;

          default:
            break;
        }
      }
      break;

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}


/*
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    Log_warning1("Outgoing RF FIFO full. Re-schedule transmission of msg with opcode 0x%02x",
      pMsg->method);

    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   PRZ_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      ProjectZero_freeAttRsp(FAILURE);

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

    // Log the opcode of the message that caused the violation.
    Log_error1("Flow control violated. Opcode of offending ATT msg: 0x%02x",
      pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Log_info1("MTU Size change: %d bytes", pMsg->msg.mtuEvt.MTU);
  }
  else
  {
    // Got an expected GATT message from a peer.
    Log_info1("Recevied GATT Message. Opcode: 0x%02x", pMsg->method);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}




/*
 *  Application error handling functions
 *****************************************************************************/

/*
 * @brief   Send a pending ATT response message.
 *
 *          The message is one that the stack was trying to send based on a
 *          peer request, but the response couldn't be sent because the
 *          user application had filled the TX queue with other data.
 *
 * @param   none
 *
 * @return  none
 */
static void ProjectZero_sendAttRsp(void)
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
      ProjectZero_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      Log_warning2("Retrying message with opcode 0x%02x. Attempt %d",
        pAttRsp->method, rspTxRetry);
    }
  }
}

/*
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void ProjectZero_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      Log_info2("Sent message with opcode 0x%02x. Attempt %d",
        pAttRsp->method, rspTxRetry);
    }
    else
    {
      Log_error2("Gave up message with opcode 0x%02x. Status: %d",
        pAttRsp->method, status);

      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}


/******************************************************************************
 *****************************************************************************
 *
 *  Handlers of direct system callbacks.
 *
 *  Typically enqueue the information or request as a message for the
 *  application Task for handling.
 *
 ****************************************************************************
 *****************************************************************************/


/*
 *  Callbacks from the Stack Task context (GAP or Service changes)
 *****************************************************************************/

/**
 * Callback from GAP Role indicating a role state change.
 */
static void user_gapStateChangeCB(gaprole_States_t newState)
{
  Log_info1("(CB) GAP State change: %d, Sending msg to app.", (IArg)newState);
  user_enqueueRawAppMsg( APP_MSG_GAP_STATE_CHANGE, (uint8_t *)&newState, sizeof(newState) );
}

/*
 * @brief   Passcode callback.
 *
 * @param   connHandle - connection handle
 * @param   uiInputs   - input passcode?
 * @param   uiOutputs  - display passcode?
 *
 * @return  none
 */
static void user_gapBondMgr_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                       uint8_t uiInputs, uint8_t uiOutputs)
{
  passcode_req_t req =
  {
    .connHandle = connHandle,
    .uiInputs = uiInputs,
    .uiOutputs = uiOutputs
  };

  // Defer handling of the passcode request to the application, in case
  // user input is required, and because a BLE API must be used from Task.
  user_enqueueRawAppMsg(APP_MSG_SEND_PASSCODE, (uint8_t *)&req, sizeof(req));
}

/*
 * @brief   Pairing state callback.
 *
 * @param   connHandle - connection handle
 * @param   state      - pairing state
 * @param   status     - pairing status
 *
 * @return  none
 */
static void user_gapBondMgr_pairStateCB(uint16_t connHandle, uint8_t state,
                                        uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Log_info0("Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      Log_info0("Pairing completed successfully.");
    }
    else
    {
      Log_error1("Pairing failed. Error: %02x", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
     Log_info0("Re-established pairing from stored bond info.");
    }
  }
}

/**
 * Callback handler for characteristic value changes in services.
 */
static void user_service_ValueChangeCB( uint16_t connHandle, uint16_t svcUuid,
                                        uint8_t paramID, uint8_t *pValue,
                                        uint16_t len )
{
  // See the service header file to compare paramID with characteristic.
  Log_info2("(CB) Characteristic value change: svc(0x%04x) paramID(%d). "
            "Sending msg to app.", (IArg)svcUuid, (IArg)paramID);
  user_enqueueCharDataMsg(APP_MSG_SERVICE_WRITE, connHandle, svcUuid, paramID,
                          pValue, len);
}

/**
 * Callback handler for characteristic configuration changes in services.
 */
static void user_service_CfgChangeCB( uint16_t connHandle, uint16_t svcUuid,
                                      uint8_t paramID, uint8_t *pValue,
                                      uint16_t len )
{
  Log_info2("(CB) Char config change: svc(0x%04x) paramID(%d). "
            "Sending msg to app.", (IArg)svcUuid, (IArg)paramID);
  user_enqueueCharDataMsg(APP_MSG_SERVICE_CFG, connHandle, svcUuid,
                          paramID, pValue, len);
}

/*
 *  Callbacks from Swi-context
 *****************************************************************************/

/*
 * @brief  Callback from Clock module on timeout
 *
 *         Determines new state after debouncing
 *
 * @param  buttonId    The pin being debounced
 */
static void buttonDebounceSwiFxn(UArg buttonId)
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
      if (buttonPinVal && button0State)
      {
        // Button was released
        buttonMsg.state = button0State = 0;
        sendMsg = TRUE;
        //t.n.tmp 161104
        DAC_SPI_Init();
        //DAC_SPI_SWV();
        //t.n.tmp 161111
        ADC_Init();
      }
      else if (!buttonPinVal && !button0State)
      {
        // Button was pressed
        buttonMsg.state = button0State = 1;
        sendMsg = TRUE;
      }
      break;

    case Board_BUTTON1:
      // If button is now released (buttonPinVal is active low, so release is 1)
      // and button state was pressed (buttonstate is active high so press is 1)
      if (buttonPinVal && button1State)
      {
        // Button was released
        buttonMsg.state = button1State = 0;
        sendMsg = TRUE;
        //t.n.tmp 161115
        //t.n.tmp 161111
        if (adcDataReady >= 1)
        {
            //ADC_Data_Disp01();
            ADC_Data_Disp02();
        }
        else
        {
            //SWV_Meas01();
            //SWV_Meas02();
            //SWV_Meas03();
            SWV_Meas04();
        }
      }
      else if (!buttonPinVal && !button0State)
      {
        // Button was pressed
        buttonMsg.state = button1State = 1;
        sendMsg = TRUE;
      }
      break;
  }

  if (sendMsg == TRUE)
  {
    user_enqueueRawAppMsg(APP_MSG_BUTTON_DEBOUNCED,
                      (uint8_t *)&buttonMsg, sizeof(buttonMsg));
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
static void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId)
{
  Log_info1("Button interrupt: %s",
            (IArg)((pinId == Board_BUTTON0)?"Button 0":"Button 1"));

  // Disable interrupt on that pin for now. Re-enabled after debounce.
  PIN_setConfig(handle, PIN_BM_IRQ, pinId | PIN_IRQ_DIS);

  // Start debounce timer
  switch (pinId)
  {
    case Board_BUTTON0:
      Clock_start(Clock_handle(&button0DebounceClock));
      break;
    case Board_BUTTON1:
      Clock_start(Clock_handle(&button1DebounceClock));
      break;
  }
}


/******************************************************************************
 *****************************************************************************
 *
 *  Utility functions
 *
 ****************************************************************************
 *****************************************************************************/

/*
 * @brief  Generic message constructor for characteristic data.
 *
 *         Sends a message to the application for handling in Task context where
 *         the message payload is a char_data_t struct.
 *
 *         From service callbacks the appMsgType is APP_MSG_SERVICE_WRITE or
 *         APP_MSG_SERVICE_CFG, and functions running in another context than
 *         the Task itself, can set the type to APP_MSG_UPDATE_CHARVAL to
 *         make the user Task loop invoke user_updateCharVal function for them.
 *
 * @param  appMsgType    Enumerated type of message being sent.
 * @param  connHandle    GAP Connection handle of the relevant connection
 * @param  serviceUUID   16-bit part of the relevant service UUID
 * @param  paramID       Index of the characteristic in the service
 * @oaram  *pValue       Pointer to characteristic value
 * @param  len           Length of characteristic data
 */
static void user_enqueueCharDataMsg( app_msg_types_t appMsgType,
                                     uint16_t connHandle,
                                     uint16_t serviceUUID, uint8_t paramID,
                                     uint8_t *pValue, uint16_t len )
{
  // Called in Stack's Task context, so can't do processing here.
  // Send message to application message queue about received data.
  uint16_t readLen = len; // How much data was written to the attribute

  // Allocate memory for the message.
  // Note: The pCharData message doesn't have to contain the data itself, as
  //       that's stored in a variable in the service implementation.
  //
  //       However, to prevent data loss if a new value is received before the
  //       service's container is read out via the GetParameter API is called,
  //       we copy the characteristic's data now.
  app_msg_t *pMsg = ICall_malloc( sizeof(app_msg_t) + sizeof(char_data_t) +
                                  readLen );

  if (pMsg != NULL)
  {
    pMsg->type = appMsgType;

    char_data_t *pCharData = (char_data_t *)pMsg->pdu;
    pCharData->svcUUID = serviceUUID; // Use 16-bit part of UUID.
    pCharData->paramID = paramID;
    // Copy data from service now.
    memcpy(pCharData->data, pValue, readLen);
    // Update pCharData with how much data we received.
    pCharData->dataLen = readLen;
    // Enqueue the message using pointer to queue node element.
    Queue_enqueue(hApplicationMsgQ, &pMsg->_elem);
    // Let application know there's a message.
    Semaphore_post(sem);
  }
}

/*
 * @brief  Generic message constructor for application messages.
 *
 *         Sends a message to the application for handling in Task context.
 *
 * @param  appMsgType    Enumerated type of message being sent.
 * @oaram  *pValue       Pointer to characteristic value
 * @param  len           Length of characteristic data
 */
static void user_enqueueRawAppMsg(app_msg_types_t appMsgType, uint8_t *pData,
                                  uint16_t len)
{
  // Allocate memory for the message.
  app_msg_t *pMsg = ICall_malloc( sizeof(app_msg_t) + len );

  if (pMsg != NULL)
  {
    pMsg->type = appMsgType;

    // Copy data into message
    memcpy(pMsg->pdu, pData, len);

    // Enqueue the message using pointer to queue node element.
    Queue_enqueue(hApplicationMsgQ, &pMsg->_elem);
    // Let application know there's a message.
    Semaphore_post(sem);
  }
}


/*
 * @brief  Convenience function for updating characteristic data via char_data_t
 *         structured message.
 *
 * @note   Must run in Task context in case BLE Stack APIs are invoked.
 *
 * @param  *pCharData  Pointer to struct with value to update.
 */
static void user_updateCharVal(char_data_t *pCharData)
{
  switch(pCharData->svcUUID) {
    case LED_SERVICE_SERV_UUID:
      LedService_SetParameter(pCharData->paramID, pCharData->dataLen,
                              pCharData->data);
    break;

    case BUTTON_SERVICE_SERV_UUID:
      ButtonService_SetParameter(pCharData->paramID, pCharData->dataLen,
                                 pCharData->data);
    break;

  }
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
static char *Util_convertArrayToHexString(uint8_t const *src, uint8_t src_len,
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

/*
 * @brief   Extract the LOCALNAME from Scan/AdvData
 *
 * @param   data - Pointer to the advertisement or scan response data
 *
 * @return  Pointer to null-terminated string with the adv local name.
 */
static char *Util_getLocalNameStr(const uint8_t *data) {
  uint8_t nuggetLen = 0;
  uint8_t nuggetType = 0;
  uint8_t advIdx = 0;

  static char localNameStr[32] = { 0 };
  memset(localNameStr, 0, sizeof(localNameStr));

  for (advIdx = 0; advIdx < 32;) {
    nuggetLen = data[advIdx++];
    nuggetType = data[advIdx];
    if ( (nuggetType == GAP_ADTYPE_LOCAL_NAME_COMPLETE ||
          nuggetType == GAP_ADTYPE_LOCAL_NAME_SHORT) && nuggetLen < 31) {
      memcpy(localNameStr, &data[advIdx + 1], nuggetLen - 1);
      break;
    } else {
      advIdx += nuggetLen;
    }
  }

  return localNameStr;
}

/////////////////////////////////////////////////////////////////////
//t.n.tmp 161102
/////////////////////////////////////////////////////////////////////

//DAC command
#define DAC_CMD_SOFTRESET_AB    0x2F
#define DAC_CMD_INTERNALREF     0x3F
#define DAC_CMD_POWERCNTROL_AB  0x27

#define DAC_CMD_WRITEUPDATE_A   0x18
#define DAC_CMD_WRITEUPDATE_B   0x19
#define DAC_CMD_WRITEUPDATE_AB  0x1F

//DAC data
#define DAC_DATA_SOFTRESET_AB_DAC       0x0000
#define DAC_DATA_SOFTRESET_AB_ALL       0x0001
#define DAC_DATA_INTERNALREF_DISABLE    0x0000
#define DAC_DATA_INTERNALREF_ENBL       0x0001
#define DAC_DATA_POWERCNTROL_A_ON       0x0001
#define DAC_DATA_POWERCNTROL_B_ON       0x0002
#define DAC_DATA_POWERCNTROL_AB_ON      0x0003
#define DAC_DATA_POWERCNTROL_A_OFF      0x0011  //power down with 1kOhm
#define DAC_DATA_POWERCNTROL_B_OFF      0x0012  //power down with 1kOhm
#define DAC_DATA_POWERCNTROL_AB_OFF     0x0013  //power down with 1kOhm

//DAC Parameters
#define DAC_REF_VOLTAGE     2500    //DAC reference voltage in mV
#define DAC_RESOLUTION      65536   //DAC resolution
#define DAC_GAIN            2       //DAC gain

#define DAC_DEFAULT_VOUT_A  1500    //DAC A default output voltage in mV

/////////////////////////////////////////////////////////////////////
// AMPERO_Meas02
/////////////////////////////////////////////////////////////////////
int AMPERO_Meas02(void)
{
    int ret;
    int itmp;
    int jtmp;
    int numtmp;
    uint8_t testString[40];

    unsigned short tail = 0;
    unsigned short head = 0;

    static unsigned short dacDataLSB;
    static unsigned short amperoVreDataLSB;
    static unsigned short dacLSBuV  = DAC_REF_VOLTAGE*1000/DAC_RESOLUTION*DAC_GAIN;

    unsigned short amperoPotentialOffsetLSB     = amperoPotentialOffset*1000/dacLSBuV;
    unsigned short amperoOffsetVoltageLSB   = amperoOffsetVoltage*1000/dacLSBuV;

    int adcDataNumTotal = 0;
    int adcDataNumTmp = 0;

    //amperoAdcValue init
    for (itmp=0; itmp<AMPERO_ADC_BUFFER_SIZE; itmp++)
    {
        amperoAdcValue[itmp] = 0;
    }

    //string init
    for (itmp=0; itmp<40; itmp++)
    {
        testString[itmp] = 0xFF;
    }

    //DAC initial value A and B (set offset voltage)
    dacDataLSB = amperoOffsetVoltageLSB;
    ret = DAC_SPI_WriteReg(DAC_CMD_WRITEUPDATE_AB, dacDataLSB);
    if (ret) return -1;

    delay_ms(100);

    //DAC initial value A
    amperoVreDataLSB = amperoPotentialOffsetLSB;
    dacDataLSB = calcDacDataLSB(0, amperoVreDataLSB);
    ret = DAC_SPI_WriteReg(DAC_CMD_WRITEUPDATE_A, dacDataLSB);
    if (ret) return -1;

    delay_ms(100);

    amperoStopFlag = 0;

    //loop
    while ( adcDataNumTotal < amperoPoints )
    {
        //stop flag check and break
        if (amperoStopFlag == 1) break;

        amperoAdcValue[adcDataNumTmp] = ADC_Sample();
        adcDataNumTmp++;
        if ( adcDataNumTmp >= AMPERO_ADC_BUFFER_SIZE )
        {
            adcDataNumTmp = 0;
        }
        adcDataNumTotal++;

        //delay
        delay_ms(amperoPeriod);

        //update data
        //header
        jtmp = 0;
        testString[jtmp*2] = 'A';   //Amperometry
        testString[jtmp*2+1] = 'M'; //Measuring

        jtmp = 1;   //data no.
        testString[jtmp*2] = 0x00FF&adcDataNumTotal;
        testString[jtmp*2+1] = 0x00FF&(adcDataNumTotal>>8);

        //data
        for (jtmp=2; jtmp<20; jtmp++)
        {
            numtmp = adcDataNumTmp - jtmp + 1;
            if (numtmp < 0)
            {
                numtmp += AMPERO_ADC_BUFFER_SIZE;
            }
            testString[jtmp*2] = 0x00FF&amperoAdcValue[numtmp];
            testString[jtmp*2+1] = 0x00FF&(amperoAdcValue[numtmp]>>8);
        }

        DataService_SetParameter(DS_STRING_ID, sizeof(testString), testString);
        Log_info2("ADC data total (ready to send): %d (%d)", adcDataNumTmp, jtmp );

    }


    //initial value; only offset (apply 0 V)
    dacDataLSB = amperoOffsetVoltageLSB;
    ret = DAC_SPI_WriteReg(DAC_CMD_WRITEUPDATE_A, dacDataLSB);
    if (ret) return -1;

    //data set
    jtmp = 0;
    testString[jtmp*2] = 'A';   //Amperometry
    testString[jtmp*2+1] = 'F'; //Finished
    DataService_SetParameter(DS_STRING_ID, sizeof(testString), testString);

    //log
    Log_info3("ADC result: adc %d, head %d, tail %d",
        adcDataNumTmp, head, tail);

    //number of adc data
    adcDataReady = adcDataNumTmp;

    return 0;
}

/////////////////////////////////////////////////////////////////////
// AMPERO_Meas01
/////////////////////////////////////////////////////////////////////
int AMPERO_Meas01(void)
{
    int ret;
    int itmp;
    int jtmp;
    int numtmp;
    uint8_t testString[40];

    unsigned short tail = 0;
    unsigned short head = 0;

    static unsigned short dacDataLSB;
    static unsigned short amperoVreDataLSB;
    static unsigned short dacLSBuV  = DAC_REF_VOLTAGE*1000/DAC_RESOLUTION*DAC_GAIN;

    unsigned short amperoPotentialOffsetLSB     = amperoPotentialOffset*1000/dacLSBuV;
    unsigned short amperoOffsetVoltageLSB   = amperoOffsetVoltage*1000/dacLSBuV;

    //adcValue init
    for (itmp=0; itmp<SWV_ADC_BUFFER_SIZE; itmp++)
    {
        adcValue[itmp] = 0;
    }

    //string init
    for (itmp=0; itmp<40; itmp++)
    {
        testString[itmp] = 0xFF;
    }

    //DAC initial value A and B (set offset voltage)
    dacDataLSB = amperoOffsetVoltageLSB;
    ret = DAC_SPI_WriteReg(DAC_CMD_WRITEUPDATE_AB, dacDataLSB);
    if (ret) return -1;

    delay_ms(100);

    //DAC initial value A
    amperoVreDataLSB = amperoPotentialOffsetLSB;
    dacDataLSB = calcDacDataLSB(0, amperoVreDataLSB);
    ret = DAC_SPI_WriteReg(DAC_CMD_WRITEUPDATE_A, dacDataLSB);
    if (ret) return -1;

    delay_ms(100);

    int adcDataNumTmp = 0;

    amperoStopFlag = 0;

    //loop
    while ( adcDataNumTmp < amperoPoints )
    {
        //stop flag check and break
        if (amperoStopFlag == 1) break;

        adcValue[adcDataNumTmp] = ADC_Sample();
        adcDataNumTmp++;
        //TODO overflow!!!

        //delay
        delay_ms(amperoPeriod);

        //update data
        //header
        jtmp = 0;
        testString[jtmp*2] = 'A';   //Amperometry
        testString[jtmp*2+1] = 'M'; //Measuring

        jtmp = 1;   //data no.
        testString[jtmp*2] = 0x00FF&adcDataNumTmp;
        testString[jtmp*2+1] = 0x00FF&(adcDataNumTmp>>8);

        //data
        for (jtmp=2; jtmp<20; jtmp++)
        {
            numtmp = adcDataNumTmp - jtmp + 1;
            if (numtmp < 0)
            {
                break;
            }
            else
            {
                testString[jtmp*2] = 0x00FF&adcValue[numtmp];
                testString[jtmp*2+1] = 0x00FF&(adcValue[numtmp]>>8);
            }
        }

        DataService_SetParameter(DS_STRING_ID, sizeof(testString), testString);
        Log_info2("ADC data total (ready to send): %d (%d)", adcDataNumTmp, jtmp );

    }


    //initial value; only offset (apply 0 V)
    dacDataLSB = amperoOffsetVoltageLSB;
    ret = DAC_SPI_WriteReg(DAC_CMD_WRITEUPDATE_A, dacDataLSB);
    if (ret) return -1;

    //data set
    jtmp = 0;
    testString[jtmp*2] = 'A';   //Amperometry
    testString[jtmp*2+1] = 'F'; //Finished
    DataService_SetParameter(DS_STRING_ID, sizeof(testString), testString);

    //log
    Log_info3("ADC result: adc %d, head %d, tail %d",
        adcDataNumTmp, head, tail);

    //number of adc data
    adcDataReady = adcDataNumTmp;

    return 0;
}

/////////////////////////////////////////////////////////////////////
// CV_Meas01
/////////////////////////////////////////////////////////////////////
int CV_Meas01(void)
{
    return 0;
}

/////////////////////////////////////////////////////////////////////
// ADC_Data_Disp
/////////////////////////////////////////////////////////////////////
int ADC_Data_Disp02(void)
{
    uint8_t testString[40];
    int itmp;

    //data exist?
    if (adcDataDisplayed == adcDataReady || adcDataReady == 0)
    {
        adcDataDisplayed = 0;
        adcDataReady = 0;
        Log_error2("No ADC data: %d / %d", adcDataDisplayed, adcDataReady );
        return -1;
    }

    //init
    for (itmp=0; itmp<40; itmp++)
    {
        testString[itmp] = 0xFF;
    }

    //header
    itmp = 0;
    testString[itmp*2] = 0x00FF&adcDataReady;
    testString[itmp*2+1] = 0x00FF&(adcDataReady>>8);

    itmp = 1;
    testString[itmp*2] = 0x00FF&adcDataDisplayed;
    testString[itmp*2+1] = 0x00FF&(adcDataDisplayed>>8);

    //data
    for (itmp=2; itmp<20; itmp++)
    {
        testString[itmp*2] = 0x00FF&adcValue[adcDataDisplayed];
        testString[itmp*2+1] = 0x00FF&(adcValue[adcDataDisplayed]>>8);
        adcDataDisplayed++;

        if (adcDataDisplayed == adcDataReady)
        {
            adcDataDisplayed = 0;
            adcDataReady = 0;
            break;
        }
    }

    DataService_SetParameter(DS_STRING_ID, sizeof(testString), testString);

    Log_info2("ADC data: %d / %d", adcDataDisplayed, adcDataReady );

    return 0;
}

/////////////////////////////////////////////////////////////////////
// ADC_Data_Disp
/////////////////////////////////////////////////////////////////////
int ADC_Data_Disp01(void)
{
    //TODO change
    int itmp;

    for (itmp=((adcDataDisplayed+1)*125/5); itmp<((adcDataDisplayed+2)*125/5); itmp++)
    {
        Log_info5("ADC value: %d, %d, %d, %d, %d",
            adcValue[5*itmp],
            adcValue[5*itmp+1],
            adcValue[5*itmp+2],
            adcValue[5*itmp+3],
            adcValue[5*itmp+4] );
    }

    adcDataDisplayed++;

    if (adcDataDisplayed == adcDataReady)
    {
        adcDataDisplayed = 0;
        adcDataReady = 0;
    }

    return 0;
}

/////////////////////////////////////////////////////////////////////
// calcDacDataLSBInt
/////////////////////////////////////////////////////////////////////
unsigned short calcDacDataLSBInt(int cvOffsetVoltageLSB, int cvVreDataLSB)
{
    unsigned short dacDataLSB;
    dacDataLSB = (unsigned short)(cvOffsetVoltageLSB + cvVreDataLSB);
    return dacDataLSB;
}

/////////////////////////////////////////////////////////////////////
// calcDacDataLSB
/////////////////////////////////////////////////////////////////////
unsigned short calcDacDataLSB(unsigned short swvOffsetVoltageLSB, unsigned short swvVreDataLSB)
{
    unsigned short dacDataLSB;
    dacDataLSB = swvOffsetVoltageLSB + swvVreDataLSB;
    return dacDataLSB;
}

/////////////////////////////////////////////////////////////////////
// SWV_Meas07
// t.n.tmp 170210
// change amplitude definition
/////////////////////////////////////////////////////////////////////
int SWV_Meas07(void)
{
    return 0;

}

/////////////////////////////////////////////////////////////////////
// SWV_Meas06
/////////////////////////////////////////////////////////////////////
int SWV_Meas06(void)
{
    return 0;

}

/////////////////////////////////////////////////////////////////////
// SWV_Meas05
/////////////////////////////////////////////////////////////////////
int SWV_Meas05(void)
{
    return 0;
}

/////////////////////////////////////////////////////////////////////
// SWV_Meas04
/////////////////////////////////////////////////////////////////////
int SWV_Meas04(void)
{
    return 0;
}

/////////////////////////////////////////////////////////////////////
// SWV_Meas03
/////////////////////////////////////////////////////////////////////
int SWV_Meas03(void)
{
    return 0;
}

/////////////////////////////////////////////////////////////////////
// SWV_Meas02
/////////////////////////////////////////////////////////////////////
int SWV_Meas02(void)
{
    return 0;
}

/////////////////////////////////////////////////////////////////////
// SWV_Meas01
/////////////////////////////////////////////////////////////////////
int SWV_Meas01(void)
{
//t.n.tmp 170131 comment out
/*
    int ret;
    int itmp;

    unsigned short tail = 0;

    static unsigned short data;
    static unsigned short dacLSBuV  = DAC_REF_VOLTAGE*1000/DAC_RESOLUTION*DAC_GAIN;

    unsigned short swvInitVoltageLSB     = SWV_INIT_VOLTAGE*1000/dacLSBuV;
    unsigned short swvFinalVoltageLSB    = SWV_FINAL_VOLTAGE*1000/dacLSBuV;
    unsigned short swvAmplitudeLSB       = SWV_AMPLITUDE*1000/dacLSBuV;
    unsigned short swvStepVoltageLSB     = SWV_STEP_VOLTAGE*1000/dacLSBuV;
    unsigned short swvOffsetVoltageLSB   = SWV_OFFSET_VOLTAGE*1000/dacLSBuV;

    unsigned int swvAdcRtcPeriod = 0x00010000 / SWV_FREQ / 2;

    //adcValue init
    for (itmp=0; itmp<SWV_ADC_BUFFER_SIZE; itmp++)
    {
        adcValue[itmp] = 0;
    }

    //DAC initial value A and B (set offset voltage)
    data = swvOffsetVoltageLSB;
    ret = DAC_SPI_WriteReg(DAC_CMD_WRITEUPDATE_AB, data);
    if (ret) return -1;

    //DAC initial value A
    data = swvInitVoltageLSB + swvOffsetVoltageLSB;
    ret = DAC_SPI_WriteReg(DAC_CMD_WRITEUPDATE_A, data);
    if (ret) return -1;

    delay_ms(SWV_INIT_WAIT*10);

    //ADC trigger
    //unsigned int adcFreq = 1000 / SWV_PERIOD * 2;
    //TODO change to synchronize. 2^?
    scifStartRtcTicksNow(swvAdcRtcPeriod);
    scifStartTasksNbl(BV(SCIF_ADC_DATA_LOGGER_TASK_ID));

    //delay
    delay_ms(SWV_PERIOD*10+1);

    int dacDataNumTmp = 0;

    //loop
    while ( data < (swvFinalVoltageLSB + swvOffsetVoltageLSB - swvAmplitudeLSB) )
    {
        data -= (swvAmplitudeLSB - swvStepVoltageLSB);
        ret = DAC_SPI_WriteReg(DAC_CMD_WRITEUPDATE_A, data);
        delay_ms(SWV_PERIOD-SWV_HIGH_DURATION);

        dacDataNumTmp++;

        data += swvAmplitudeLSB;
        ret = DAC_SPI_WriteReg(DAC_CMD_WRITEUPDATE_A, data);
        delay_ms(SWV_HIGH_DURATION);

        dacDataNumTmp++;
    }

    //initial value; only offset (apply 0 V)
    data = swvOffsetVoltageLSB;
    ret = DAC_SPI_WriteReg(DAC_CMD_WRITEUPDATE_A, data);
    if (ret) return -1;

    // Fetch the current head index
    unsigned short head = scifTaskData.adcDataLogger.output.head;
    //TODO buffer is enough?

    //log
    //Log_info4("ADC result: dac %d, adc %d, head %d, tail %d",
    //    dacDataNumTmp, head-tail, head, tail);

    //store ADC data
    int adcDataNumTmp = 0;

    while (tail != head) {
        //AUXADCGenManualTrigger();
        adcValue[adcDataNumTmp] = scifTaskData.adcDataLogger.output.pSamples[tail];

        // Increment the tail index
        if (++tail >= SCIF_ADC_DATA_LOGGER_BUFFER_SIZE) {   //TODO buffer is enough?
            tail = 0;
        }

        adcDataNumTmp++;
    }

    //ADC stop
    if (scifStopTasksNbl(BV(SCIF_ADC_DATA_LOGGER_TASK_ID)) == SCIF_SUCCESS)
    {
        scifWaitOnNbl(42);
        scifResetTaskStructs(BV(SCIF_ADC_DATA_LOGGER_TASK_ID), BV(SCIF_STRUCT_OUTPUT));
        scifStopRtcTicks();
        //scifUninit(); // not neccesary if you plan to restart the task again soon.
    }

//Display ADC data
//TODO change!!!
    Log_info4("ADC result: dac %d, adc %d, head %d, tail %d",
        dacDataNumTmp, adcDataNumTmp, head, tail);

    if (adcDataNumTmp>125)
    {
        adcDataReady = 1;
        adcDataNumTmp = 125;
    }

    for (itmp=0; itmp<(adcDataNumTmp/5); itmp++)
    {
        Log_info5("ADC value: %d, %d, %d, %d, %d",
            adcValue[5*itmp],
            adcValue[5*itmp+1],
            adcValue[5*itmp+2],
            adcValue[5*itmp+3],
            adcValue[5*itmp+4] );
    }
*/
    return 0;
}

/////////////////////////////////////////////////////////////////////
// ADC_Init
/////////////////////////////////////////////////////////////////////
int ADC_Init(void)
{
    // Initialize the Sensor Controller
    scifOsalInit();
    scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
    scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
    scifInit(&scifDriverSetup);

    return 0;
}

/////////////////////////////////////////////////////////////////////
// ADC_Sample
/////////////////////////////////////////////////////////////////////
unsigned short ADC_Sample(void)
{
    unsigned short head = 0;
    unsigned short value = 0;

    //ADC trigger
    scifStartRtcTicksNow(0x00010000);   //dummy
    scifStartTasksNbl(BV(SCIF_ADC_DATA_LOGGER_TASK_ID));

    //wait
    delay_ms(1);

    // Fetch the current head index
    head = scifTaskData.adcDataLogger.output.head;
    if (head>0) value = scifTaskData.adcDataLogger.output.pSamples[head-1];
    else value = 9999;

    //ADC stop
    if (scifStopTasksNbl(BV(SCIF_ADC_DATA_LOGGER_TASK_ID)) == SCIF_SUCCESS)
    {
        scifWaitOnNbl(42);
        scifResetTaskStructs(BV(SCIF_ADC_DATA_LOGGER_TASK_ID), BV(SCIF_STRUCT_OUTPUT));
        scifStopRtcTicks();
        //scifUninit(); // not neccesary if you plan to restart the task again soon.
    }

    return value;
}

/////////////////////////////////////////////////////////////////////
// scTaskAlertCallback
/////////////////////////////////////////////////////////////////////
void scTaskAlertCallback(void) {
    //t.n.tmp 161116
    //Log_info0("scTaskAlertCallback");
} // taskAlertCallback

/////////////////////////////////////////////////////////////////////
// scCtrlReadyCallback
/////////////////////////////////////////////////////////////////////
void scCtrlReadyCallback(void) {
    //t.n.tmp 161116
    //Log_info0("scCtrlReadyCallback");
} // ctrlReadyCallback

/////////////////////////////////////////////////////////////////////
// DAC_SPI_SWV
/////////////////////////////////////////////////////////////////////
int DAC_SPI_SWV(void)
{
    int ret;
    static unsigned short data;

    static unsigned short dacLSBuV  = DAC_REF_VOLTAGE*1000/DAC_RESOLUTION*DAC_GAIN;

    unsigned short swvInitVoltageLSB     = SWV_INIT_VOLTAGE*1000/dacLSBuV;
    unsigned short swvFinalVoltageLSB    = SWV_FINAL_VOLTAGE*1000/dacLSBuV;
    unsigned short swvAmplitudeLSB       = SWV_AMPLITUDE*1000/dacLSBuV;
    unsigned short swvStepVoltageLSB     = SWV_STEP_VOLTAGE*1000/dacLSBuV;
    unsigned short swvOffsetVoltageLSB   = SWV_OFFSET_VOLTAGE*1000/dacLSBuV;

    //initial value
    data = swvInitVoltageLSB + swvOffsetVoltageLSB;
    ret = DAC_SPI_WriteReg(DAC_CMD_WRITEUPDATE_A, data);
    if (ret) return -1;

    delay_ms(SWV_INIT_WAIT*10);

    //loop
    while ( data < (swvFinalVoltageLSB + swvOffsetVoltageLSB - swvAmplitudeLSB) )
    {
        data += swvAmplitudeLSB;
        ret = DAC_SPI_WriteReg(DAC_CMD_WRITEUPDATE_A, data);
        delay_ms(SWV_HIGH_DURATION);

        data -= (swvAmplitudeLSB - swvStepVoltageLSB);
        ret = DAC_SPI_WriteReg(DAC_CMD_WRITEUPDATE_A, data);
        delay_ms(SWV_PERIOD-SWV_HIGH_DURATION);
    }

    //initial value
    data = swvInitVoltageLSB + swvOffsetVoltageLSB;
    ret = DAC_SPI_WriteReg(DAC_CMD_WRITEUPDATE_A, data);
    if (ret) return -1;

    return 0;
}

/////////////////////////////////////////////////////////////////////
// DAC_SPI_Init
/////////////////////////////////////////////////////////////////////
int DAC_SPI_Init(void)
{
    int ret;
    static unsigned short data;

    static unsigned short dacLSBuV      = DAC_REF_VOLTAGE*1000/DAC_RESOLUTION*DAC_GAIN;
    unsigned short dacDefaultVoutALSB   = DAC_DEFAULT_VOUT_A*1000/dacLSBuV;
    unsigned short dacDefaultVoutBLSB   = dacDefaultVoutALSB;

    //open spi
    bspSpiOpen();

    //reset all dac
    ret = DAC_SPI_WriteReg(DAC_CMD_SOFTRESET_AB, DAC_DATA_SOFTRESET_AB_ALL);
    if (ret) return -1;

    //internal reference enable
    ret = DAC_SPI_WriteReg(DAC_CMD_INTERNALREF, DAC_DATA_INTERNALREF_ENBL);
    if (ret) return -1;

    //dac AB write data and update
    data = dacDefaultVoutALSB;
    ret = DAC_SPI_WriteReg(DAC_CMD_WRITEUPDATE_A, data);
    if (ret) return -1;
    data = dacDefaultVoutBLSB;
    ret = DAC_SPI_WriteReg(DAC_CMD_WRITEUPDATE_B, data);
    if (ret) return -1;

    //dac AB power on
    ret = DAC_SPI_WriteReg(DAC_CMD_POWERCNTROL_AB, DAC_DATA_POWERCNTROL_AB_ON);
    if (ret) return -1;

    return 0;
}

/////////////////////////////////////////////////////////////////////
// DAC_SPI_WriteUpdateA
/////////////////////////////////////////////////////////////////////
int DAC_SPI_WriteUpdateA(unsigned short data)
{
    int ret;

    //write and update dac A
    ret = DAC_SPI_WriteReg(DAC_CMD_WRITEUPDATE_A, data);
    if (ret) return -1;

    return 0;
}

/////////////////////////////////////////////////////////////////////
// DAC_SPI_WriteReg
/////////////////////////////////////////////////////////////////////
int DAC_SPI_WriteReg(unsigned char cmd, unsigned short data)
{
    //write reg (cmd 1byte, data 2byte)
    const uint8_t wbuf_cmd[] = { cmd };
    const uint8_t wbuf_data[] = { data>>8, data };

    //chip select enable
    PIN_setOutputValue(daccsPinHandle, Board_SPI0_DAC_CSN, Board_SPI0_DAC_CS_ON);

    //write command
    int ret = bspSpiWrite(wbuf_cmd,sizeof(wbuf_cmd));
    if (ret) {
        PIN_setOutputValue(daccsPinHandle, Board_SPI0_DAC_CSN, Board_SPI0_DAC_CS_OFF);
        return -3;
    }

    //write data
    ret = bspSpiWrite(wbuf_data,sizeof(wbuf_data));
    if (ret) {
        PIN_setOutputValue(daccsPinHandle, Board_SPI0_DAC_CSN, Board_SPI0_DAC_CS_OFF);
        return -2;
    }

    //chip select disable
    PIN_setOutputValue(daccsPinHandle, Board_SPI0_DAC_CSN, Board_SPI0_DAC_CS_OFF);
    return 0;
}

// copy from bsp_spi.c
/*******************************************************************************
 * LOCAL variables
 */
static SPI_Handle spiHandle = NULL;
static SPI_Params spiParams;

static PIN_Handle hSpiPin = NULL;
static PIN_State pinState;

// Table of pins to be "parked" in when no device is selected. Leaving the pin
// floating causes increased power consumption in WinBond W25XC10.
static PIN_Config BoardSpiInitTable[] = {
    Board_SPI0_MOSI | PIN_PULLDOWN,
    Board_SPI0_MISO | PIN_PULLDOWN,
    Board_SPI0_CLK | PIN_PULLDOWN,
    PIN_TERMINATE
};

static uint8_t nUsers = 0;

/*******************************************************************************
 * @fn          bspSpiWrite
 *
 * @brief       Write to an SPI device
 *
 * @param       buf - pointer to data buffer
 * @param       len - number of bytes to write
 *
 * @return      '0' if success, -1 if failed
 */
int bspSpiWrite(const uint8_t *buf, size_t len)
{
  SPI_Transaction masterTransaction;

  masterTransaction.count  = len;
  masterTransaction.txBuf  = (void*)buf;
  masterTransaction.arg    = NULL;
  masterTransaction.rxBuf  = NULL;

  return SPI_transfer(spiHandle, &masterTransaction) ? 0 : -1;
}

/*******************************************************************************
 * @fn          bspSpiRead
 *
 * @brief       Read from an SPI device
 *
 * @param       buf - pointer to data buffer
 * @param       len - number of bytes to write
 *
 * @return      '0' if success, -1 if failed
 */
int bspSpiRead(uint8_t *buf, size_t len)
{
  SPI_Transaction masterTransaction;

  masterTransaction.count  = len;
  masterTransaction.txBuf  = NULL;
  masterTransaction.arg    = NULL;
  masterTransaction.rxBuf  = buf;

  return SPI_transfer(spiHandle, &masterTransaction) ? 0 : -1;
}


/*******************************************************************************
 * @fn          bspSpiWriteRead
 *
 * @brief       Write to and read from an SPI device in the same transaction
 *
 * @param       buf - pointer to data buffer
 * @param       wlen - number of bytes to write
 * @param       rlen - number of bytes to read
 *
 * @return      '0' if success, -1 if failed
 */
int bspSpiWriteRead(uint8_t *buf, uint8_t wlen, uint8_t rlen)
{
  SPI_Transaction masterTransaction;
  bool success;

  masterTransaction.count  = wlen + rlen;
  masterTransaction.txBuf  = buf;
  masterTransaction.arg    = NULL;
  masterTransaction.rxBuf  = buf;

  success = SPI_transfer(spiHandle, &masterTransaction);
  if (success)
  {
    memcpy(buf,buf+wlen,rlen);
  }

  return success ? 0 : -1;
}

/*******************************************************************************
 * @fn          bspSpiOpen
 *
 * @brief       Open the RTOS SPI driver
 *
 * @param       none
 *
 * @return      none
 */
void bspSpiOpen(void)
{
  if (hSpiPin != NULL)
  {
    // Remove IO configuration of SPI lines
    PIN_close(hSpiPin);

    hSpiPin = NULL;
  }

  if (spiHandle == NULL)
  {
    /*  Configure SPI as master, 1 MHz bit rate*/
    SPI_Params_init(&spiParams);
    spiParams.bitRate = 1000000;
    spiParams.mode         = SPI_MASTER;
    spiParams.transferMode = SPI_MODE_BLOCKING;
    //t.n.tmp 161107
    spiParams.frameFormat  = SPI_POL0_PHA1;

    /* Attempt to open SPI. */
    spiHandle = SPI_open(Board_SPI0, &spiParams);

    if (spiHandle == NULL)
    {
      Task_exit();
    }
  }

  nUsers++;
}

/*******************************************************************************
 * @fn          bspSpiClose
 *
 * @brief       Close the RTOS SPI driver
 *
 * @return      none
 */
void bspSpiClose(void)
{
  nUsers--;

  if (nUsers > 0)
  {
    // Don't close the driver if still in use
    return;
  }

  if (spiHandle != NULL)
  {
    // Close the RTOS driver
    SPI_close(spiHandle);
    spiHandle = NULL;
  }

  if (hSpiPin == NULL)
  {
    // Configure SPI lines as IO and set them according to BoardSpiInitTable
    hSpiPin = PIN_open(&pinState, BoardSpiInitTable);
  }

  nUsers = 0;
}

/*********************************************************************
*********************************************************************/



///*
// * Copyright (c) 2016, Texas Instruments Incorporated
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions
// * are met:
// *
// * *  Redistributions of source code must retain the above copyright
// *    notice, this list of conditions and the following disclaimer.
// *
// * *  Redistributions in binary form must reproduce the above copyright
// *    notice, this list of conditions and the following disclaimer in the
// *    documentation and/or other materials provided with the distribution.
// *
// * *  Neither the name of Texas Instruments Incorporated nor the names of
// *    its contributors may be used to endorse or promote products derived
// *    from this software without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
// * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
///*********************************************************************
// * INCLUDES
// */
//#include <string.h>
//
////#define xdc_runtime_Log_DISABLE_ALL 1  // Add to disable logs from this file
//
//#include <ti/sysbios/knl/Task.h>
//#include <ti/sysbios/knl/Semaphore.h>
//#include <ti/sysbios/knl/Queue.h>
//
//#include <ti/drivers/PIN.h>
//#include <ti/mw/display/Display.h>
//
//#include <xdc/runtime/Log.h>
//#include <xdc/runtime/Diags.h>
//
//// Stack headers
//#include <hci_tl.h>
//#include <gap.h>
//#include <gatt.h>
//#include <gapgattserver.h>
//#include <gattservapp.h>
//#include <osal_snv.h>
//#include <gapbondmgr.h>
//#include <peripheral.h>
//#include <icall_apimsg.h>
//
//#include <devinfoservice.h>
//
//#include "util.h"
//
//#include "Board.h"
//#include "project_zero.h"
//
//// Bluetooth Developer Studio services
//#include "led_service.h"
//#include "button_service.h"
//#include "data_service.h"
//
//
///*********************************************************************
// * CONSTANTS
// */
//// Advertising interval when device is discoverable (units of 625us, 160=100ms)
//#define DEFAULT_ADVERTISING_INTERVAL          160
//
//// Limited discoverable mode advertises for 30.72s, and then stops
//// General discoverable mode advertises indefinitely
//#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
//
//// Default pass-code used for pairing.
//#define DEFAULT_PASSCODE                      000000
//
//// Task configuration
//#define PRZ_TASK_PRIORITY                     1
//
//#ifndef PRZ_TASK_STACK_SIZE
//#define PRZ_TASK_STACK_SIZE                   800
//#endif
//
//// Internal Events for RTOS application
//#define PRZ_STATE_CHANGE_EVT                  0x0001
//#define PRZ_CHAR_CHANGE_EVT                   0x0002
//#define PRZ_PERIODIC_EVT                      0x0004
//#define PRZ_CONN_EVT_END_EVT                  0x0008
//
///*********************************************************************
// * TYPEDEFS
// */
//// Types of messages that can be sent to the user application task from other
//// tasks or interrupts. Note: Messages from BLE Stack are sent differently.
//typedef enum
//{
//  APP_MSG_SERVICE_WRITE = 0,   /* A characteristic value has been written     */
//  APP_MSG_SERVICE_CFG,         /* A characteristic configuration has changed  */
//  APP_MSG_UPDATE_CHARVAL,      /* Request from ourselves to update a value    */
//  APP_MSG_GAP_STATE_CHANGE,    /* The GAP / connection state has changed      */
//  APP_MSG_BUTTON_DEBOUNCED,    /* A button has been debounced with new value  */
//  APP_MSG_SEND_PASSCODE,       /* A pass-code/PIN is requested during pairing */
//} app_msg_types_t;
//
//// Struct for messages sent to the application task
//typedef struct
//{
//  Queue_Elem       _elem;
//  app_msg_types_t  type;
//  uint8_t          pdu[];
//} app_msg_t;
//
//// Struct for messages about characteristic data
//typedef struct
//{
//  uint16_t svcUUID; // UUID of the service
//  uint16_t dataLen; //
//  uint8_t  paramID; // Index of the characteristic
//  uint8_t  data[];  // Flexible array member, extended to malloc - sizeof(.)
//} char_data_t;
//
//// Struct for message about sending/requesting passcode from peer.
//typedef struct
//{
//  uint16_t connHandle;
//  uint8_t  uiInputs;
//  uint8_t  uiOutputs;
//  uint32   numComparison;
//} passcode_req_t;
//
//// Struct for message about button state
//typedef struct
//{
//  PIN_Id   pinId;
//  uint8_t  state;
//} button_state_t;
//
///*********************************************************************
// * LOCAL VARIABLES
// */
//
//// Entity ID globally used to check for source and/or destination of messages
//static ICall_EntityID selfEntity;
//
//// Semaphore globally used to post events to the application thread
//static ICall_Semaphore sem;
//
//// Queue object used for application messages.
//static Queue_Struct applicationMsgQ;
//static Queue_Handle hApplicationMsgQ;
//
//// Task configuration
//Task_Struct przTask;
//Char przTaskStack[PRZ_TASK_STACK_SIZE];
//
//
//// GAP - SCAN RSP data (max size = 31 bytes)
//static uint8_t scanRspData[] =
//{
//  // No scan response data provided.
//  0x00 // Placeholder to keep the compiler happy.
//};
//
//// GAP - Advertisement data (max size = 31 bytes, though this is
//// best kept short to conserve power while advertisting)
//static uint8_t advertData[] =
//{
//  // Flags; this sets the device to use limited discoverable
//  // mode (advertises for 30 seconds at a time) or general
//  // discoverable mode (advertises indefinitely), depending
//  // on the DEFAULT_DISCOVERY_MODE define.
//  0x02,   // length of this data
//  GAP_ADTYPE_FLAGS,
//  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
//
//  // complete name
//  13,
//  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
//  'P', 'r', 'o', 'j', 'e', 'c', 't', ' ', 'Z', 'e', 'r', 'o',
//
//};
//
//// GAP GATT Attributes
//static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Project Zero";
//
//// Globals used for ATT Response retransmission
//static gattMsgEvent_t *pAttRsp = NULL;
//static uint8_t rspTxRetry = 0;
//
//
///* Pin driver handles */
//static PIN_Handle buttonPinHandle;
//static PIN_Handle ledPinHandle;
//
///* Global memory storage for a PIN_Config table */
//static PIN_State buttonPinState;
//static PIN_State ledPinState;
//
///*
// * Initial LED pin configuration table
// *   - LEDs Board_LED0 & Board_LED1 are off.
// */
//PIN_Config ledPinTable[] = {
//  Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//  Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//  PIN_TERMINATE
//};
//
///*
// * Application button pin configuration table:
// *   - Buttons interrupts are configured to trigger on falling edge.
// */
//PIN_Config buttonPinTable[] = {
//    Board_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
//    Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
//    PIN_TERMINATE
//};
//
//// Clock objects for debouncing the buttons
//static Clock_Struct button0DebounceClock;
//static Clock_Struct button1DebounceClock;
//
//// State of the buttons
//static uint8_t button0State = 0;
//static uint8_t button1State = 0;
//
//// Global display handle
//Display_Handle dispHandle;
//
///*********************************************************************
// * LOCAL FUNCTIONS
// */
//
//static void ProjectZero_init( void );
//static void ProjectZero_taskFxn(UArg a0, UArg a1);
//
//static void user_processApplicationMessage(app_msg_t *pMsg);
//static uint8_t ProjectZero_processStackMsg(ICall_Hdr *pMsg);
//static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg);
//
//static void ProjectZero_sendAttRsp(void);
//static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg);
//static void ProjectZero_freeAttRsp(uint8_t status);
//
//static void user_processGapStateChangeEvt(gaprole_States_t newState);
//static void user_gapStateChangeCB(gaprole_States_t newState);
//static void user_gapBondMgr_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
//                                       uint8_t uiInputs, uint8_t uiOutputs, uint32 numComparison);
//static void user_gapBondMgr_pairStateCB(uint16_t connHandle, uint8_t state,
//                                        uint8_t status);
//
//static void buttonDebounceSwiFxn(UArg buttonId);
//static void user_handleButtonPress(button_state_t *pState);
//
//// Generic callback handlers for value changes in services.
//static void user_service_ValueChangeCB( uint16_t connHandle, uint16_t svcUuid, uint8_t paramID, uint8_t *pValue, uint16_t len );
//static void user_service_CfgChangeCB( uint16_t connHandle, uint16_t svcUuid, uint8_t paramID, uint8_t *pValue, uint16_t len );
//
//// Task context handlers for generated services.
//static void user_LedService_ValueChangeHandler(char_data_t *pCharData);
//static void user_ButtonService_CfgChangeHandler(char_data_t *pCharData);
//static void user_DataService_ValueChangeHandler(char_data_t *pCharData);
//static void user_DataService_CfgChangeHandler(char_data_t *pCharData);
//
//// Task handler for sending notifications.
//static void user_updateCharVal(char_data_t *pCharData);
//
//// Utility functions
//static void user_enqueueRawAppMsg(app_msg_types_t appMsgType, uint8_t *pData, uint16_t len );
//static void user_enqueueCharDataMsg(app_msg_types_t appMsgType, uint16_t connHandle,
//                                    uint16_t serviceUUID, uint8_t paramID,
//                                    uint8_t *pValue, uint16_t len);
//static void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId);
//
//static char *Util_convertArrayToHexString(uint8_t const *src, uint8_t src_len,
//                                          uint8_t *dst, uint8_t dst_len);
//static char *Util_getLocalNameStr(const uint8_t *data);
//
///*********************************************************************
// * PROFILE CALLBACKS
// */
//
//// GAP Role Callbacks
//static gapRolesCBs_t user_gapRoleCBs =
//{
//  user_gapStateChangeCB     // Profile State Change Callbacks
//};
//
//// GAP Bond Manager Callbacks
//static gapBondCBs_t user_bondMgrCBs =
//{
//  user_gapBondMgr_passcodeCB, // Passcode callback
//  user_gapBondMgr_pairStateCB // Pairing / Bonding state Callback
//};
//
///*
// * Callbacks in the user application for events originating from BLE services.
// */
//// LED Service callback handler.
//// The type LED_ServiceCBs_t is defined in led_service.h
//static LedServiceCBs_t user_LED_ServiceCBs =
//{
//  .pfnChangeCb    = user_service_ValueChangeCB, // Characteristic value change callback handler
//  .pfnCfgChangeCb = NULL, // No notification-/indication enabled chars in LED Service
//};
//
//// Button Service callback handler.
//// The type Button_ServiceCBs_t is defined in button_service.h
//static ButtonServiceCBs_t user_Button_ServiceCBs =
//{
//  .pfnChangeCb    = NULL, // No writable chars in Button Service, so no change handler.
//  .pfnCfgChangeCb = user_service_CfgChangeCB, // Noti/ind configuration callback handler
//};
//
//// Data Service callback handler.
//// The type Data_ServiceCBs_t is defined in data_service.h
//static DataServiceCBs_t user_Data_ServiceCBs =
//{
//  .pfnChangeCb    = user_service_ValueChangeCB, // Characteristic value change callback handler
//  .pfnCfgChangeCb = user_service_CfgChangeCB, // Noti/ind configuration callback handler
//};
//
//
///*********************************************************************
// * PUBLIC FUNCTIONS
// */
//
///*
// * @brief   Task creation function for the user task.
// *
// * @param   None.
// *
// * @return  None.
// */
//void ProjectZero_createTask(void)
//{
//  Task_Params taskParams;
//
//  // Configure task
//  Task_Params_init(&taskParams);
//  taskParams.stack = przTaskStack;
//  taskParams.stackSize = PRZ_TASK_STACK_SIZE;
//  taskParams.priority = PRZ_TASK_PRIORITY;
//
//  Task_construct(&przTask, ProjectZero_taskFxn, &taskParams, NULL);
//}
//
///*
// * @brief   Called before the task loop and contains application-specific
// *          initialization of the BLE stack, hardware setup, power-state
// *          notification if used, and BLE profile/service initialization.
// *
// * @param   None.
// *
// * @return  None.
// */
//static void ProjectZero_init(void)
//{
//  // ******************************************************************
//  // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
//  // ******************************************************************
//  // Register the current thread as an ICall dispatcher application
//  // so that the application can send and receive messages via ICall to Stack.
//  ICall_registerApp(&selfEntity, &sem);
//
//  Log_info0("Initializing the user task, hardware, BLE stack and services.");
//
//  // Open display. By default this is disabled via the predefined symbol Display_DISABLE_ALL.
//  dispHandle = Display_open(Display_Type_LCD, NULL);
//
//  // Initialize queue for application messages.
//  // Note: Used to transfer control to application thread from e.g. interrupts.
//  Queue_construct(&applicationMsgQ, NULL);
//  hApplicationMsgQ = Queue_handle(&applicationMsgQ);
//
//  // ******************************************************************
//  // Hardware initialization
//  // ******************************************************************
//
//  // Open LED pins
//  ledPinHandle = PIN_open(&ledPinState, ledPinTable);
//  if(!ledPinHandle) {
//    Log_error0("Error initializing board LED pins");
//    Task_exit();
//  }
//
//  buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
//  if(!buttonPinHandle) {
//    Log_error0("Error initializing button pins");
//    Task_exit();
//  }
//
//  // Setup callback for button pins
//  if (PIN_registerIntCb(buttonPinHandle, &buttonCallbackFxn) != 0) {
//    Log_error0("Error registering button callback function");
//    Task_exit();
//  }
//
//  // Create the debounce clock objects for Button 0 and Button 1
//  Clock_Params clockParams;
//  Clock_Params_init(&clockParams);
//
//  // Both clock objects use the same callback, so differentiate on argument
//  // given to the callback in Swi context
//  clockParams.arg = Board_BUTTON0;
//
//  // Initialize to 50 ms timeout when Clock_start is called.
//  // Timeout argument is in ticks, so convert from ms to ticks via tickPeriod.
//  Clock_construct(&button0DebounceClock, buttonDebounceSwiFxn,
//                  50 * (1000/Clock_tickPeriod),
//                  &clockParams);
//
//  // Second button
//  clockParams.arg = Board_BUTTON1;
//  Clock_construct(&button1DebounceClock, buttonDebounceSwiFxn,
//                  50 * (1000/Clock_tickPeriod),
//                  &clockParams);
//
//  // ******************************************************************
//  // BLE Stack initialization
//  // ******************************************************************
//
//  // Setup the GAP Peripheral Role Profile
//  uint8_t initialAdvertEnable = TRUE;  // Advertise on power-up
//
//  // By setting this to zero, the device will go into the waiting state after
//  // being discoverable. Otherwise wait this long [ms] before advertising again.
//  uint16_t advertOffTime = 0; // miliseconds
//
//  // Set advertisement enabled.
//  GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
//                       &initialAdvertEnable);
//
//  // Configure the wait-time before restarting advertisement automatically
//  GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
//                       &advertOffTime);
//
//  // Initialize Scan Response data
//  GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
//
//  // Initialize Advertisement data
//  GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
//
//  Log_info1("Name in advertData array: \x1b[33m%s\x1b[0m",
//            (IArg)Util_getLocalNameStr(advertData));
//
//  // Set advertising interval
//  uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;
//
//  GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
//  GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
//  GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
//  GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
//
//  // Set duration of advertisement before stopping in Limited adv mode.
//  GAP_SetParamValue(TGAP_LIM_ADV_TIMEOUT, 30); // Seconds
//
//  // ******************************************************************
//  // BLE Bond Manager initialization
//  // ******************************************************************
//  uint32_t passkey = 0; // passkey "000000"
//  uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
//  uint8_t mitm = TRUE;
//  uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
//  uint8_t bonding = TRUE;
//
//  GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
//                          &passkey);
//  GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
//  GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
//  GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
//  GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
//
//  // ******************************************************************
//  // BLE Service initialization
//  // ******************************************************************
//
//  // Add services to GATT server
//  GGS_AddService(GATT_ALL_SERVICES);           // GAP
//  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
//  DevInfo_AddService();                        // Device Information Service
//
//  // Set the device name characteristic in the GAP Profile
//  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);
//
//  // Add services to GATT server and give ID of this task for Indication acks.
//  LedService_AddService( selfEntity );
//  ButtonService_AddService( selfEntity );
//  DataService_AddService( selfEntity );
//
//  // Register callbacks with the generated services that
//  // can generate events (writes received) to the application
//  LedService_RegisterAppCBs( &user_LED_ServiceCBs );
//  ButtonService_RegisterAppCBs( &user_Button_ServiceCBs );
//  DataService_RegisterAppCBs( &user_Data_ServiceCBs );
//
//  // Placeholder variable for characteristic intialization
//  uint8_t initVal[40] = {0};
//  uint8_t initString[] = "This is a pretty long string, isn't it!";
//
//  // Initalization of characteristics in LED_Service that can provide data.
//  LedService_SetParameter(LS_LED0_ID, LS_LED0_LEN, initVal);
//  LedService_SetParameter(LS_LED1_ID, LS_LED1_LEN, initVal);
//
//  // Initalization of characteristics in Button_Service that can provide data.
//  ButtonService_SetParameter(BS_BUTTON0_ID, BS_BUTTON0_LEN, initVal);
//  ButtonService_SetParameter(BS_BUTTON1_ID, BS_BUTTON1_LEN, initVal);
//
//  // Initalization of characteristics in Data_Service that can provide data.
//  DataService_SetParameter(DS_STRING_ID, sizeof(initString), initString);
//  DataService_SetParameter(DS_STREAM_ID, DS_STREAM_LEN, initVal);
//
//  // Start the stack in Peripheral mode.
//  VOID GAPRole_StartDevice(&user_gapRoleCBs);
//
//  // Start Bond Manager
//  VOID GAPBondMgr_Register(&user_bondMgrCBs);
//
//  // Register with GAP for HCI/Host messages
//  GAP_RegisterForMsgs(selfEntity);
//
//  // Register for GATT local events and ATT Responses pending for transmission
//  GATT_RegisterForMsgs(selfEntity);
//}
//
//
///*
// * @brief   Application task entry point.
// *
// *          Invoked by TI-RTOS when BIOS_start is called. Calls an init function
// *          and enters an infinite loop waiting for messages.
// *
// *          Messages can be either directly from the BLE stack or from user code
// *          like Hardware Interrupt (Hwi) or a callback function.
// *
// *          The reason for sending messages to this task from e.g. Hwi's is that
// *          some RTOS and Stack APIs are not available in callbacks and so the
// *          actions that may need to be taken is dispatched to this Task.
// *
// * @param   a0, a1 - not used.
// *
// * @return  None.
// */
//static void ProjectZero_taskFxn(UArg a0, UArg a1)
//{
//  // Initialize application
//  ProjectZero_init();
//
//  // Application main loop
//  for (;;)
//  {
//    // Waits for a signal to the semaphore associated with the calling thread.
//    // Note that the semaphore associated with a thread is signaled when a
//    // message is queued to the message receive queue of the thread or when
//    // ICall_signal() function is called onto the semaphore.
//    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);
//
//    if (errno == ICALL_ERRNO_SUCCESS)
//    {
//      ICall_EntityID dest;
//      ICall_ServiceEnum src;
//      ICall_HciExtEvt *pMsg = NULL;
//
//      // Check if we got a signal because of a stack message
//      if (ICall_fetchServiceMsg(&src, &dest,
//                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
//      {
//        uint8 safeToDealloc = TRUE;
//
//        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
//        {
//          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;
//
//          // Check for event flags received (event signature 0xffff)
//          if (pEvt->signature == 0xffff)
//          {
//            // Event received when a connection event is completed
//            if (pEvt->event_flag & PRZ_CONN_EVT_END_EVT)
//            {
//              // Try to retransmit pending ATT Response (if any)
//              ProjectZero_sendAttRsp();
//            }
//          }
//          else // It's a message from the stack and not an event.
//          {
//            // Process inter-task message
//            safeToDealloc = ProjectZero_processStackMsg((ICall_Hdr *)pMsg);
//          }
//        }
//
//        if (pMsg && safeToDealloc)
//        {
//          ICall_freeMsg(pMsg);
//        }
//      }
//
//      // Process messages sent from another task or another context.
//      while (!Queue_empty(hApplicationMsgQ))
//      {
//        app_msg_t *pMsg = Queue_dequeue(hApplicationMsgQ);
//
//        // Process application-layer message probably sent from ourselves.
//        user_processApplicationMessage(pMsg);
//
//        // Free the received message.
//        ICall_free(pMsg);
//      }
//    }
//  }
//}
//
//
///*
// * @brief   Handle application messages
// *
// *          These are messages not from the BLE stack, but from the
// *          application itself.
// *
// *          For example, in a Software Interrupt (Swi) it is not possible to
// *          call any BLE APIs, so instead the Swi function must send a message
// *          to the application Task for processing in Task context.
// *
// * @param   pMsg  Pointer to the message of type app_msg_t.
// *
// * @return  None.
// */
//static void user_processApplicationMessage(app_msg_t *pMsg)
//{
//  char_data_t *pCharData = (char_data_t *)pMsg->pdu;
//
//  switch (pMsg->type)
//  {
//    case APP_MSG_SERVICE_WRITE: /* Message about received value write */
//      /* Call different handler per service */
//      switch(pCharData->svcUUID) {
//        case LED_SERVICE_SERV_UUID:
//          user_LedService_ValueChangeHandler(pCharData);
//          break;
//        case DATA_SERVICE_SERV_UUID:
//          user_DataService_ValueChangeHandler(pCharData);
//          break;
//
//      }
//      break;
//
//    case APP_MSG_SERVICE_CFG: /* Message about received CCCD write */
//      /* Call different handler per service */
//      switch(pCharData->svcUUID) {
//        case BUTTON_SERVICE_SERV_UUID:
//          user_ButtonService_CfgChangeHandler(pCharData);
//          break;
//        case DATA_SERVICE_SERV_UUID:
//          user_DataService_CfgChangeHandler(pCharData);
//          break;
//      }
//      break;
//
//    case APP_MSG_UPDATE_CHARVAL: /* Message from ourselves to send  */
//      user_updateCharVal(pCharData);
//      break;
//
//    case APP_MSG_GAP_STATE_CHANGE: /* Message that GAP state changed  */
//      user_processGapStateChangeEvt( *(gaprole_States_t *)pMsg->pdu );
//      break;
//
//    case APP_MSG_SEND_PASSCODE: /* Message about pairing PIN request */
//      {
//        passcode_req_t *pReq = (passcode_req_t *)pMsg->pdu;
//        Log_info2("BondMgr Requested passcode. We are %s passcode %06d",
//                  (IArg)(pReq->uiInputs?"Sending":"Displaying"),
//                  DEFAULT_PASSCODE);
//        // Send passcode response.
//        GAPBondMgr_PasscodeRsp(pReq->connHandle, SUCCESS, DEFAULT_PASSCODE);
//      }
//      break;
//
//    case APP_MSG_BUTTON_DEBOUNCED: /* Message from swi about pin change */
//      {
//        button_state_t *pButtonState = (button_state_t *)pMsg->pdu;
//        user_handleButtonPress(pButtonState);
//      }
//      break;
//  }
//}
//
//
///******************************************************************************
// *****************************************************************************
// *
// *  Handlers of system/application events deferred to the user Task context.
// *  Invoked from the application Task function above.
// *
// *  Further down you can find the callback handler section containing the
// *  functions that defer their actions via messages to the application task.
// *
// ****************************************************************************
// *****************************************************************************/
//
//
///*
// * @brief   Process a pending GAP Role state change event.
// *
// * @param   newState - new state
// *
// * @return  None.
// */
//static void user_processGapStateChangeEvt(gaprole_States_t newState)
//{
//  switch ( newState )
//  {
//    case GAPROLE_STARTED:
//      {
//        uint8_t ownAddress[B_ADDR_LEN];
//        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];
//
//        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
//
//        // use 6 bytes of device address for 8 bytes of system ID value
//        systemId[0] = ownAddress[0];
//        systemId[1] = ownAddress[1];
//        systemId[2] = ownAddress[2];
//
//        // set middle bytes to zero
//        systemId[4] = 0x00;
//        systemId[3] = 0x00;
//
//        // shift three bytes up
//        systemId[7] = ownAddress[5];
//        systemId[6] = ownAddress[4];
//        systemId[5] = ownAddress[3];
//
//        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
//
//        // Display device address
//        char *cstr_ownAddress = Util_convertBdAddr2Str(ownAddress);
//        Log_info1("GAP is started. Our address: \x1b[32m%s\x1b[0m", (IArg)cstr_ownAddress);
//      }
//      break;
//
//    case GAPROLE_ADVERTISING:
//      Log_info0("Advertising");
//      break;
//
//    case GAPROLE_CONNECTED:
//      {
//        uint8_t peerAddress[B_ADDR_LEN];
//
//        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);
//
//        char *cstr_peerAddress = Util_convertBdAddr2Str(peerAddress);
//        Log_info1("Connected. Peer address: \x1b[32m%s\x1b[0m", (IArg)cstr_peerAddress);
//       }
//      break;
//
//    case GAPROLE_CONNECTED_ADV:
//      Log_info0("Connected and advertising");
//      break;
//
//    case GAPROLE_WAITING:
//      Log_info0("Disconnected / Idle");
//      break;
//
//    case GAPROLE_WAITING_AFTER_TIMEOUT:
//      Log_info0("Connection timed out");
//      break;
//
//    case GAPROLE_ERROR:
//      Log_info0("Error");
//      break;
//
//    default:
//      break;
//  }
//}
//
//
///*
// * @brief   Handle a debounced button press or release in Task context.
// *          Invoked by the taskFxn based on a message received from a callback.
// *
// * @see     buttonDebounceSwiFxn
// * @see     buttonCallbackFxn
// *
// * @param   pState  pointer to button_state_t message sent from debounce Swi.
// *
// * @return  None.
// */
//static void user_handleButtonPress(button_state_t *pState)
//{
//  Log_info2("%s %s",
//    (IArg)(pState->pinId == Board_BUTTON0?"Button 0":"Button 1"),
//    (IArg)(pState->state?"\x1b[32mpressed\x1b[0m":
//                         "\x1b[33mreleased\x1b[0m"));
//
//  // Update the service with the new value.
//  // Will automatically send notification/indication if enabled.
//  switch (pState->pinId)
//  {
//    case Board_BUTTON0:
//      ButtonService_SetParameter(BS_BUTTON0_ID,
//                                 sizeof(pState->state),
//                                 &pState->state);
//      break;
//    case Board_BUTTON1:
//      ButtonService_SetParameter(BS_BUTTON1_ID,
//                                 sizeof(pState->state),
//                                 &pState->state);
//      break;
//  }
//}
//
///*
// * @brief   Handle a write request sent from a peer device.
// *
// *          Invoked by the Task based on a message received from a callback.
// *
// *          When we get here, the request has already been accepted by the
// *          service and is valid from a BLE protocol perspective as well as
// *          having the correct length as defined in the service implementation.
// *
// * @param   pCharData  pointer to malloc'd char write data
// *
// * @return  None.
// */
//void user_LedService_ValueChangeHandler(char_data_t *pCharData)
//{
//  static uint8_t pretty_data_holder[16]; // 5 bytes as hex string "AA:BB:CC:DD:EE"
//  Util_convertArrayToHexString(pCharData->data, pCharData->dataLen,
//                               pretty_data_holder, sizeof(pretty_data_holder));
//
//  switch (pCharData->paramID)
//  {
//    case LS_LED0_ID:
//      Log_info3("Value Change msg: %s %s: %s",
//                (IArg)"LED Service",
//                (IArg)"LED0",
//                (IArg)pretty_data_holder);
//
//      // Do something useful with pCharData->data here
//      // -------------------------
//      // Set the output value equal to the received value. 0 is off, not 0 is on
//      PIN_setOutputValue(ledPinHandle, Board_LED0, pCharData->data[0]);
//      Log_info2("Turning %s %s",
//                (IArg)"\x1b[31mLED0\x1b[0m",
//                (IArg)(pCharData->data[0]?"on":"off"));
//      break;
//
//    case LS_LED1_ID:
//      Log_info3("Value Change msg: %s %s: %s",
//                (IArg)"LED Service",
//                (IArg)"LED1",
//                (IArg)pretty_data_holder);
//
//      // Do something useful with pCharData->data here
//      // -------------------------
//      // Set the output value equal to the received value. 0 is off, not 0 is on
//      PIN_setOutputValue(ledPinHandle, Board_LED1, pCharData->data[0]);
//      Log_info2("Turning %s %s",
//                (IArg)"\x1b[32mLED1\x1b[0m",
//                (IArg)(pCharData->data[0]?"on":"off"));
//      break;
//
//  default:
//    return;
//  }
//}
//
//
///*
// * @brief   Handle a CCCD (configuration change) write received from a peer
// *          device. This tells us whether the peer device wants us to send
// *          Notifications or Indications.
// *
// * @param   pCharData  pointer to malloc'd char write data
// *
// * @return  None.
// */
//void user_ButtonService_CfgChangeHandler(char_data_t *pCharData)
//{
//  // Cast received data to uint16, as that's the format for CCCD writes.
//  uint16_t configValue = *(uint16_t *)pCharData->data;
//  char *configValString;
//
//  // Determine what to tell the user
//  switch(configValue)
//  {
//  case GATT_CFG_NO_OPERATION:
//    configValString = "Noti/Ind disabled";
//    break;
//  case GATT_CLIENT_CFG_NOTIFY:
//    configValString = "Notifications enabled";
//    break;
//  case GATT_CLIENT_CFG_INDICATE:
//    configValString = "Indications enabled";
//    break;
//  }
//
//  switch (pCharData->paramID)
//  {
//    case BS_BUTTON0_ID:
//      Log_info3("CCCD Change msg: %s %s: %s",
//                (IArg)"Button Service",
//                (IArg)"BUTTON0",
//                (IArg)configValString);
//      // -------------------------
//      // Do something useful with configValue here. It tells you whether someone
//      // wants to know the state of this characteristic.
//      // ...
//      break;
//
//    case BS_BUTTON1_ID:
//      Log_info3("CCCD Change msg: %s %s: %s",
//                (IArg)"Button Service",
//                (IArg)"BUTTON1",
//                (IArg)configValString);
//      // -------------------------
//      // Do something useful with configValue here. It tells you whether someone
//      // wants to know the state of this characteristic.
//      // ...
//      break;
//  }
//}
//
///*
// * @brief   Handle a write request sent from a peer device.
// *
// *          Invoked by the Task based on a message received from a callback.
// *
// *          When we get here, the request has already been accepted by the
// *          service and is valid from a BLE protocol perspective as well as
// *          having the correct length as defined in the service implementation.
// *
// * @param   pCharData  pointer to malloc'd char write data
// *
// * @return  None.
// */
//void user_DataService_ValueChangeHandler(char_data_t *pCharData)
//{
//  // Value to hold the received string for printing via Log, as Log printouts
//  // happen in the Idle task, and so need to refer to a global/static variable.
//  static uint8_t received_string[DS_STRING_LEN] = {0};
//
//  switch (pCharData->paramID)
//  {
//    case DS_STRING_ID:
//      // Do something useful with pCharData->data here
//      // -------------------------
//      // Copy received data to holder array, ensuring NULL termination.
//      memset(received_string, 0, DS_STRING_LEN);
//      memcpy(received_string, pCharData->data, DS_STRING_LEN-1);
//      // Needed to copy before log statement, as the holder array remains after
//      // the pCharData message has been freed and reused for something else.
//      Log_info3("Value Change msg: %s %s: %s",
//                (IArg)"Data Service",
//                (IArg)"String",
//                (IArg)received_string);
//      break;
//
//    case DS_STREAM_ID:
//      Log_info3("Value Change msg: Data Service Stream: %02x:%02x:%02x...",
//                (IArg)pCharData->data[0],
//                (IArg)pCharData->data[1],
//                (IArg)pCharData->data[2]);
//      // -------------------------
//      // Do something useful with pCharData->data here
//      break;
//
//  default:
//    return;
//  }
//}
//
///*
// * @brief   Handle a CCCD (configuration change) write received from a peer
// *          device. This tells us whether the peer device wants us to send
// *          Notifications or Indications.
// *
// * @param   pCharData  pointer to malloc'd char write data
// *
// * @return  None.
// */
//void user_DataService_CfgChangeHandler(char_data_t *pCharData)
//{
//  // Cast received data to uint16, as that's the format for CCCD writes.
//  uint16_t configValue = *(uint16_t *)pCharData->data;
//  char *configValString;
//
//  // Determine what to tell the user
//  switch(configValue)
//  {
//  case GATT_CFG_NO_OPERATION:
//    configValString = "Noti/Ind disabled";
//    break;
//  case GATT_CLIENT_CFG_NOTIFY:
//    configValString = "Notifications enabled";
//    break;
//  case GATT_CLIENT_CFG_INDICATE:
//    configValString = "Indications enabled";
//    break;
//  }
//
//  switch (pCharData->paramID)
//  {
//    case DS_STREAM_ID:
//      Log_info3("CCCD Change msg: %s %s: %s",
//                (IArg)"Data Service",
//                (IArg)"Stream",
//                (IArg)configValString);
//      // -------------------------
//      // Do something useful with configValue here. It tells you whether someone
//      // wants to know the state of this characteristic.
//      // ...
//      break;
//  }
//}
//
//
///*
// * @brief   Process an incoming BLE stack message.
// *
// *          This could be a GATT message from a peer device like acknowledgement
// *          of an Indication we sent, or it could be a response from the stack
// *          to an HCI message that the user application sent.
// *
// * @param   pMsg - message to process
// *
// * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
// */
//static uint8_t ProjectZero_processStackMsg(ICall_Hdr *pMsg)
//{
//  uint8_t safeToDealloc = TRUE;
//
//  switch (pMsg->event)
//  {
//    case GATT_MSG_EVENT:
//      // Process GATT message
//      safeToDealloc = ProjectZero_processGATTMsg((gattMsgEvent_t *)pMsg);
//      break;
//
//    case HCI_GAP_EVENT_EVENT:
//      {
//        // Process HCI message
//        switch(pMsg->status)
//        {
//          case HCI_COMMAND_COMPLETE_EVENT_CODE:
//            // Process HCI Command Complete Event
//            Log_info0("HCI Command Complete Event received");
//            break;
//
//          default:
//            break;
//        }
//      }
//      break;
//
//    default:
//      // do nothing
//      break;
//  }
//
//  return (safeToDealloc);
//}
//
//
///*
// * @brief   Process GATT messages and events.
// *
// * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
// */
//static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg)
//{
//  // See if GATT server was unable to transmit an ATT response
//  if (pMsg->hdr.status == blePending)
//  {
//    Log_warning1("Outgoing RF FIFO full. Re-schedule transmission of msg with opcode 0x%02x",
//      pMsg->method);
//
//    // No HCI buffer was available. Let's try to retransmit the response
//    // on the next connection event.
//    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
//                                   PRZ_CONN_EVT_END_EVT) == SUCCESS)
//    {
//      // First free any pending response
//      ProjectZero_freeAttRsp(FAILURE);
//
//      // Hold on to the response message for retransmission
//      pAttRsp = pMsg;
//
//      // Don't free the response message yet
//      return (FALSE);
//    }
//  }
//  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
//  {
//    // ATT request-response or indication-confirmation flow control is
//    // violated. All subsequent ATT requests or indications will be dropped.
//    // The app is informed in case it wants to drop the connection.
//
//    // Log the opcode of the message that caused the violation.
//    Log_error1("Flow control violated. Opcode of offending ATT msg: 0x%02x",
//      pMsg->msg.flowCtrlEvt.opcode);
//  }
//  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
//  {
//    // MTU size updated
//    Log_info1("MTU Size change: %d bytes", pMsg->msg.mtuEvt.MTU);
//  }
//  else
//  {
//    // Got an expected GATT message from a peer.
//    Log_info1("Recevied GATT Message. Opcode: 0x%02x", pMsg->method);
//  }
//
//  // Free message payload. Needed only for ATT Protocol messages
//  GATT_bm_free(&pMsg->msg, pMsg->method);
//
//  // It's safe to free the incoming message
//  return (TRUE);
//}
//
//
//
//
///*
// *  Application error handling functions
// *****************************************************************************/
//
///*
// * @brief   Send a pending ATT response message.
// *
// *          The message is one that the stack was trying to send based on a
// *          peer request, but the response couldn't be sent because the
// *          user application had filled the TX queue with other data.
// *
// * @param   none
// *
// * @return  none
// */
//static void ProjectZero_sendAttRsp(void)
//{
//  // See if there's a pending ATT Response to be transmitted
//  if (pAttRsp != NULL)
//  {
//    uint8_t status;
//
//    // Increment retransmission count
//    rspTxRetry++;
//
//    // Try to retransmit ATT response till either we're successful or
//    // the ATT Client times out (after 30s) and drops the connection.
//    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
//    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
//    {
//      // Disable connection event end notice
//      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);
//
//      // We're done with the response message
//      ProjectZero_freeAttRsp(status);
//    }
//    else
//    {
//      // Continue retrying
//      Log_warning2("Retrying message with opcode 0x%02x. Attempt %d",
//        pAttRsp->method, rspTxRetry);
//    }
//  }
//}
//
///*
// * @brief   Free ATT response message.
// *
// * @param   status - response transmit status
// *
// * @return  none
// */
//static void ProjectZero_freeAttRsp(uint8_t status)
//{
//  // See if there's a pending ATT response message
//  if (pAttRsp != NULL)
//  {
//    // See if the response was sent out successfully
//    if (status == SUCCESS)
//    {
//      Log_info2("Sent message with opcode 0x%02x. Attempt %d",
//        pAttRsp->method, rspTxRetry);
//    }
//    else
//    {
//      Log_error2("Gave up message with opcode 0x%02x. Status: %d",
//        pAttRsp->method, status);
//
//      // Free response payload
//      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
//    }
//
//    // Free response message
//    ICall_freeMsg(pAttRsp);
//
//    // Reset our globals
//    pAttRsp = NULL;
//    rspTxRetry = 0;
//  }
//}
//
//
///******************************************************************************
// *****************************************************************************
// *
// *  Handlers of direct system callbacks.
// *
// *  Typically enqueue the information or request as a message for the
// *  application Task for handling.
// *
// ****************************************************************************
// *****************************************************************************/
//
//
///*
// *  Callbacks from the Stack Task context (GAP or Service changes)
// *****************************************************************************/
//
///**
// * Callback from GAP Role indicating a role state change.
// */
//static void user_gapStateChangeCB(gaprole_States_t newState)
//{
//  Log_info1("(CB) GAP State change: %d, Sending msg to app.", (IArg)newState);
//  user_enqueueRawAppMsg( APP_MSG_GAP_STATE_CHANGE, (uint8_t *)&newState, sizeof(newState) );
//}
//
///*
// * @brief   Passcode callback.
// *
// * @param   connHandle - connection handle
// * @param   uiInputs   - input passcode?
// * @param   uiOutputs  - display passcode?
// * @param   numComparison - numeric comparison value
// *
// * @return  none
// */
//static void user_gapBondMgr_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
//                                       uint8_t uiInputs, uint8_t uiOutputs, uint32 numComparison)
//{
//  passcode_req_t req =
//  {
//    .connHandle = connHandle,
//    .uiInputs = uiInputs,
//    .uiOutputs = uiOutputs,
//    .numComparison = numComparison
//  };
//
//  // Defer handling of the passcode request to the application, in case
//  // user input is required, and because a BLE API must be used from Task.
//  user_enqueueRawAppMsg(APP_MSG_SEND_PASSCODE, (uint8_t *)&req, sizeof(req));
//}
//
///*
// * @brief   Pairing state callback.
// *
// * @param   connHandle - connection handle
// * @param   state      - pairing state
// * @param   status     - pairing status
// *
// * @return  none
// */
//static void user_gapBondMgr_pairStateCB(uint16_t connHandle, uint8_t state,
//                                        uint8_t status)
//{
//  if (state == GAPBOND_PAIRING_STATE_STARTED)
//  {
//    Log_info0("Pairing started");
//  }
//  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
//  {
//    if (status == SUCCESS)
//    {
//      Log_info0("Pairing completed successfully.");
//    }
//    else
//    {
//      Log_error1("Pairing failed. Error: %02x", status);
//    }
//  }
//  else if (state == GAPBOND_PAIRING_STATE_BONDED)
//  {
//    if (status == SUCCESS)
//    {
//     Log_info0("Re-established pairing from stored bond info.");
//    }
//  }
//}
//
///**
// * Callback handler for characteristic value changes in services.
// */
//static void user_service_ValueChangeCB( uint16_t connHandle, uint16_t svcUuid,
//                                        uint8_t paramID, uint8_t *pValue,
//                                        uint16_t len )
//{
//  // See the service header file to compare paramID with characteristic.
//  Log_info2("(CB) Characteristic value change: svc(0x%04x) paramID(%d). "
//            "Sending msg to app.", (IArg)svcUuid, (IArg)paramID);
//  user_enqueueCharDataMsg(APP_MSG_SERVICE_WRITE, connHandle, svcUuid, paramID,
//                          pValue, len);
//}
//
///**
// * Callback handler for characteristic configuration changes in services.
// */
//static void user_service_CfgChangeCB( uint16_t connHandle, uint16_t svcUuid,
//                                      uint8_t paramID, uint8_t *pValue,
//                                      uint16_t len )
//{
//  Log_info2("(CB) Char config change: svc(0x%04x) paramID(%d). "
//            "Sending msg to app.", (IArg)svcUuid, (IArg)paramID);
//  user_enqueueCharDataMsg(APP_MSG_SERVICE_CFG, connHandle, svcUuid,
//                          paramID, pValue, len);
//}
//
///*
// *  Callbacks from Swi-context
// *****************************************************************************/
//
///*
// * @brief  Callback from Clock module on timeout
// *
// *         Determines new state after debouncing
// *
// * @param  buttonId    The pin being debounced
// */
//static void buttonDebounceSwiFxn(UArg buttonId)
//{
//  // Used to send message to app
//  button_state_t buttonMsg = { .pinId = buttonId };
//  uint8_t        sendMsg   = FALSE;
//
//  // Get current value of the button pin after the clock timeout
//  uint8_t buttonPinVal = PIN_getInputValue(buttonId);
//
//  // Set interrupt direction to opposite of debounced state
//  // If button is now released (button is active low, so release is high)
//  if (buttonPinVal)
//  {
//    // Enable negative edge interrupts to wait for press
//    PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, buttonId | PIN_IRQ_NEGEDGE);
//  }
//  else
//  {
//    // Enable positive edge interrupts to wait for relesae
//    PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, buttonId | PIN_IRQ_POSEDGE);
//  }
//
//  switch(buttonId)
//  {
//    case Board_BUTTON0:
//      // If button is now released (buttonPinVal is active low, so release is 1)
//      // and button state was pressed (buttonstate is active high so press is 1)
//      if (buttonPinVal && button0State)
//      {
//        // Button was released
//        buttonMsg.state = button0State = 0;
//        sendMsg = TRUE;
//      }
//      else if (!buttonPinVal && !button0State)
//      {
//        // Button was pressed
//        buttonMsg.state = button0State = 1;
//        sendMsg = TRUE;
//      }
//      break;
//
//    case Board_BUTTON1:
//      // If button is now released (buttonPinVal is active low, so release is 1)
//      // and button state was pressed (buttonstate is active high so press is 1)
//      if (buttonPinVal && button1State)
//      {
//        // Button was released
//        buttonMsg.state = button1State = 0;
//        sendMsg = TRUE;
//      }
//      else if (!buttonPinVal && !button1State)
//      {
//        // Button was pressed
//        buttonMsg.state = button1State = 1;
//        sendMsg = TRUE;
//      }
//      break;
//  }
//
//  if (sendMsg == TRUE)
//  {
//    user_enqueueRawAppMsg(APP_MSG_BUTTON_DEBOUNCED,
//                      (uint8_t *)&buttonMsg, sizeof(buttonMsg));
//  }
//}
//
///*
// *  Callbacks from Hwi-context
// *****************************************************************************/
//
///*
// * @brief  Callback from PIN driver on interrupt
// *
// *         Sets in motion the debouncing.
// *
// * @param  handle    The PIN_Handle instance this is about
// * @param  pinId     The pin that generated the interrupt
// */
//static void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId)
//{
//  Log_info1("Button interrupt: %s",
//            (IArg)((pinId == Board_BUTTON0)?"Button 0":"Button 1"));
//
//  // Disable interrupt on that pin for now. Re-enabled after debounce.
//  PIN_setConfig(handle, PIN_BM_IRQ, pinId | PIN_IRQ_DIS);
//
//  // Start debounce timer
//  switch (pinId)
//  {
//    case Board_BUTTON0:
//      Clock_start(Clock_handle(&button0DebounceClock));
//      break;
//    case Board_BUTTON1:
//      Clock_start(Clock_handle(&button1DebounceClock));
//      break;
//  }
//}
//
//
///******************************************************************************
// *****************************************************************************
// *
// *  Utility functions
// *
// ****************************************************************************
// *****************************************************************************/
//
///*
// * @brief  Generic message constructor for characteristic data.
// *
// *         Sends a message to the application for handling in Task context where
// *         the message payload is a char_data_t struct.
// *
// *         From service callbacks the appMsgType is APP_MSG_SERVICE_WRITE or
// *         APP_MSG_SERVICE_CFG, and functions running in another context than
// *         the Task itself, can set the type to APP_MSG_UPDATE_CHARVAL to
// *         make the user Task loop invoke user_updateCharVal function for them.
// *
// * @param  appMsgType    Enumerated type of message being sent.
// * @param  connHandle    GAP Connection handle of the relevant connection
// * @param  serviceUUID   16-bit part of the relevant service UUID
// * @param  paramID       Index of the characteristic in the service
// * @oaram  *pValue       Pointer to characteristic value
// * @param  len           Length of characteristic data
// */
//static void user_enqueueCharDataMsg( app_msg_types_t appMsgType,
//                                     uint16_t connHandle,
//                                     uint16_t serviceUUID, uint8_t paramID,
//                                     uint8_t *pValue, uint16_t len )
//{
//  // Called in Stack's Task context, so can't do processing here.
//  // Send message to application message queue about received data.
//  uint16_t readLen = len; // How much data was written to the attribute
//
//  // Allocate memory for the message.
//  // Note: The pCharData message doesn't have to contain the data itself, as
//  //       that's stored in a variable in the service implementation.
//  //
//  //       However, to prevent data loss if a new value is received before the
//  //       service's container is read out via the GetParameter API is called,
//  //       we copy the characteristic's data now.
//  app_msg_t *pMsg = ICall_malloc( sizeof(app_msg_t) + sizeof(char_data_t) +
//                                  readLen );
//
//  if (pMsg != NULL)
//  {
//    pMsg->type = appMsgType;
//
//    char_data_t *pCharData = (char_data_t *)pMsg->pdu;
//    pCharData->svcUUID = serviceUUID; // Use 16-bit part of UUID.
//    pCharData->paramID = paramID;
//    // Copy data from service now.
//    memcpy(pCharData->data, pValue, readLen);
//    // Update pCharData with how much data we received.
//    pCharData->dataLen = readLen;
//    // Enqueue the message using pointer to queue node element.
//    Queue_enqueue(hApplicationMsgQ, &pMsg->_elem);
//    // Let application know there's a message.
//    Semaphore_post(sem);
//  }
//}
//
///*
// * @brief  Generic message constructor for application messages.
// *
// *         Sends a message to the application for handling in Task context.
// *
// * @param  appMsgType    Enumerated type of message being sent.
// * @oaram  *pValue       Pointer to characteristic value
// * @param  len           Length of characteristic data
// */
//static void user_enqueueRawAppMsg(app_msg_types_t appMsgType, uint8_t *pData,
//                                  uint16_t len)
//{
//  // Allocate memory for the message.
//  app_msg_t *pMsg = ICall_malloc( sizeof(app_msg_t) + len );
//
//  if (pMsg != NULL)
//  {
//    pMsg->type = appMsgType;
//
//    // Copy data into message
//    memcpy(pMsg->pdu, pData, len);
//
//    // Enqueue the message using pointer to queue node element.
//    Queue_enqueue(hApplicationMsgQ, &pMsg->_elem);
//    // Let application know there's a message.
//    Semaphore_post(sem);
//  }
//}
//
//
///*
// * @brief  Convenience function for updating characteristic data via char_data_t
// *         structured message.
// *
// * @note   Must run in Task context in case BLE Stack APIs are invoked.
// *
// * @param  *pCharData  Pointer to struct with value to update.
// */
//static void user_updateCharVal(char_data_t *pCharData)
//{
//  switch(pCharData->svcUUID) {
//    case LED_SERVICE_SERV_UUID:
//      LedService_SetParameter(pCharData->paramID, pCharData->dataLen,
//                              pCharData->data);
//    break;
//
//    case BUTTON_SERVICE_SERV_UUID:
//      ButtonService_SetParameter(pCharData->paramID, pCharData->dataLen,
//                                 pCharData->data);
//    break;
//
//  }
//}
//
///*
// * @brief   Convert {0x01, 0x02} to "01:02"
// *
// * @param   src - source byte-array
// * @param   src_len - length of array
// * @param   dst - destination string-array
// * @param   dst_len - length of array
// *
// * @return  array as string
// */
//static char *Util_convertArrayToHexString(uint8_t const *src, uint8_t src_len,
//                                          uint8_t *dst, uint8_t dst_len)
//{
//  char        hex[] = "0123456789ABCDEF";
//  uint8_t     *pStr = dst;
//  uint8_t     avail = dst_len-1;
//
//  memset(dst, 0, avail);
//
//  while (src_len && avail > 3)
//  {
//    if (avail < dst_len-1) { *pStr++ = ':'; avail -= 1; };
//    *pStr++ = hex[*src >> 4];
//    *pStr++ = hex[*src++ & 0x0F];
//    avail -= 2;
//    src_len--;
//  }
//
//  if (src_len && avail)
//    *pStr++ = ':'; // Indicate not all data fit on line.
//
//  return (char *)dst;
//}
//
///*
// * @brief   Extract the LOCALNAME from Scan/AdvData
// *
// * @param   data - Pointer to the advertisement or scan response data
// *
// * @return  Pointer to null-terminated string with the adv local name.
// */
//static char *Util_getLocalNameStr(const uint8_t *data) {
//  uint8_t nuggetLen = 0;
//  uint8_t nuggetType = 0;
//  uint8_t advIdx = 0;
//
//  static char localNameStr[32] = { 0 };
//  memset(localNameStr, 0, sizeof(localNameStr));
//
//  for (advIdx = 0; advIdx < 32;) {
//    nuggetLen = data[advIdx++];
//    nuggetType = data[advIdx];
//    if ( (nuggetType == GAP_ADTYPE_LOCAL_NAME_COMPLETE ||
//          nuggetType == GAP_ADTYPE_LOCAL_NAME_SHORT) && nuggetLen < 31) {
//      memcpy(localNameStr, &data[advIdx + 1], nuggetLen - 1);
//      break;
//    } else {
//      advIdx += nuggetLen;
//    }
//  }
//
//  return localNameStr;
//}
//
///*********************************************************************
//*********************************************************************/

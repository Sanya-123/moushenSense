// This callback file is created for your convenience. You may add application
// code to this file. If you regenerate this file over a previous version, the
// previous version will be overwritten and any code you have added will be
// lost.

#include "app/framework/include/af.h"
#include "hal/hal.h"
#include EMBER_AF_API_NETWORK_STEERING
#include "app/framework/include/af.h"
#include "app/util/ezsp/ezsp-enum.h"

#include "em_device.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"

#include "bsp.h"


int8u endpointServer = 0xFF, endpointClient = 0xFF;

void emberAfMainInitCallback(void)
{

  CMU_ClockEnable(cmuClock_GPIO, true);

  // Configure VCOM transmit pin to board controller as an output
  GPIO_PinModeSet(gpioPortF, 3, gpioModePushPull, 1);

  // Configure VCOM reeive pin from board controller as an input
  GPIO_PinModeSet(gpioPortF, 4, gpioModeInput, 0);

//  // Enable VCOM connection to board controller
//  GPIO_PinModeSet(BSP_BCC_ENABLE_PORT, BSP_BCC_ENABLE_PIN, gpioModePushPull, 1);

  CMU_ClockEnable(cmuClock_USART1, true);

  // Default asynchronous initializer (115.2 Kbps, 8N1, no flow control)
  USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;

  // Configure and enable USART1
  USART_InitAsync(USART1, &init);

  // Enable NVIC USART sources
  NVIC_ClearPendingIRQ(USART1_RX_IRQn);
  NVIC_EnableIRQ(USART1_RX_IRQn);
//  NVIC_ClearPendingIRQ(USART1_TX_IRQn);
//  NVIC_EnableIRQ(USART1_TX_IRQn);

  // Enable RX and TX for USART1 VCOM connection
  USART1->ROUTELOC0 = USART_ROUTELOC0_RXLOC_LOC27 | USART_ROUTELOC0_TXLOC_LOC27;
  USART1->ROUTEPEN |= USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_TXPEN;

  //Initializae USART Interrupts
  USART_IntEnable(USART1, USART_IEN_RXDATAV);


  //gpio
  GPIO_PinModeSet(gpioPortJ, 14, gpioModePushPull, 1);//EN

  GPIO_PinModeSet(gpioPortD, 11, gpioModePushPull, 0);//R
  GPIO_PinModeSet(gpioPortD, 12, gpioModePushPull, 0);//G
  GPIO_PinModeSet(gpioPortD, 13, gpioModePushPull, 1);//B

  GPIO_PinModeSet(gpioPortI, 0, gpioModePushPull, 0);//led0
  GPIO_PinModeSet(gpioPortI, 1, gpioModePushPull, 0);//led1
  GPIO_PinModeSet(gpioPortI, 2, gpioModePushPull, 0);//led2
  GPIO_PinModeSet(gpioPortI, 3, gpioModePushPull, 0);//led3


  GPIO_PinOutSet(gpioPortI, 3);
  GPIO_PinOutSet(gpioPortI, 2);




  printTextLine("Init done!");
}

/** @brief Called when Button 0 is pressed long
 *
 * @param timePressedMs Amount of time button 0 was pressed.
 * @param pressedAtReset Was the button pressed at startup.
 */
void emberAfPluginButtonInterfaceButton0PressedLongCallback(uint16_t timePressedMs,
                                                            bool pressedAtReset)
{
  if(emberAfNetworkState() == EMBER_JOINED_NETWORK)
  {
      emberAfPluginFindAndBindTargetStart(1);
//      GPIO_PinOutSet(gpioPortI, 1);
//      emberAfFillCommandGlobalClientToServerReadAttributes(0xFC25, 0x0000, 1);
//      emberAfFillCommandGlobalClientToServerReadAttributes

//      emberAfReadOrUpdateAttribute();
  }
  else
  {
      GPIO_PinOutSet(gpioPortI, 0);
      emberAfPluginNetworkSteeringStart();
  }
}

void emberAfPluginButtonInterfaceButton1PressedLongCallback(uint16_t timePressedMs,
                                                            bool pressedAtReset)
{
  emberAfPluginButtonInterfaceButton0PressedLongCallback(timePressedMs, pressedAtReset);
}

/** @brief Called when Button 1 is pressed short
 *
 * @param timePressedMs Time (in ms) button 1 was pressed short
 */
void emberAfPluginButtonInterfaceButton0PressedShortCallback(uint16_t timePressedMs)
{//test
  static uint32_t tmp = 0;
  tmp += 100;
  uint32_t bufferIndex = 0;
  emberAfWriteServerAttribute(1, 0xFC25, 0x0000, (uint8_t*)(&tmp), ZCL_INT32U_ATTRIBUTE_TYPE);

//  if(emberAfAppendAttributeReportFields(1, 0xFC25, 0x0000, CLUSTER_MASK_SERVER, (uint8_t*)(&tmp), 4, &bufferIndex) != EMBER_ZCL_STATUS_SUCCESS)
//    {
//      printTextLine("Error Report");
//    }

//  uint16_t time = 15;
//  emberAfWriteServerAttribute(endpointServer, 0x0003, 0x0000, (uint8_t*)(&time), ZCL_INT16U_ATTRIBUTE_TYPE);
//  emberAfWriteClientAttribute(endpointClient, 0x0003, 0x0000, (uint8_t*)(&time), ZCL_INT16U_ATTRIBUTE_TYPE);
  GPIO_PinOutToggle(gpioPortI, 1);
  printTextLine("GPIO1 clucked \n");
}

void emberAfPluginButtonInterfaceButton1PressedShortCallback(uint16_t timePressedMs)
{
  emberAfPluginButtonInterfaceButton0PressedShortCallback(timePressedMs);
}

/** @brief Cluster Init
 *
 * This function is called when a specific cluster is initialized. It gives the
 * application an opportunity to take care of cluster initialization procedures.
 * It is called exactly once for each endpoint where cluster is present.
 *
 * @param endpoint   Ver.: always
 * @param clusterId   Ver.: always
 */
void emberAfClusterInitCallback(int8u endpoint,
                                EmberAfClusterId clusterId)
{
  char str[256];
  sprintf(str, "Init cluser %d cluster:%04X", endpoint, clusterId);
  printTextLine(str);
}

//// Note: change this to change the number of seconds to delay the pulse going high
//#define NUM_SEC_DELAY 1
//
//// Note: change this to change the timer prescale
//#define TIMER_PRESCALE timerPrescale1024
//
//// Note: change this to change the pulse width (in units of ms)
//#define PULSE_WIDTH 1
//
//// Compare values for outputting the rising and falling edge
//static uint32_t compareValue1;
//static uint32_t compareValue2;
//
//void initTimer(void)
//{
//  // Enable clock for TIMER0 module
//  CMU_ClockEnable(cmuClock_TIMER0, true);
//
//  // Configure TIMER0 Compare/Capture for output compare
//  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
//  timerCCInit.mode = timerCCModeCompare;
//  timerCCInit.cmoa = timerOutputActionToggle; // Toggle output on compare match
//  TIMER_InitCC(TIMER0, 0, &timerCCInit);
//
//  // Set route to Location 3 and enable
//  // TIM0_CC0 #3 is PD1
//  TIMER0->ROUTE |= (TIMER_ROUTE_CC0PEN | TIMER_ROUTE_LOCATION_LOC3);
//
//  // Initialize and start timer with highest prescale
//  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
//  timerInit.enable = false;
//  timerInit.prescale = TIMER_PRESCALE;
//  timerInit.oneShot = true; // Generate only one pulse
//  TIMER_Init(TIMER0, &timerInit);
//
//  // Set the first compare value
//  compareValue1 = CMU_ClockFreqGet(cmuClock_TIMER0)
//                    * NUM_SEC_DELAY
//                    / (1 << TIMER_PRESCALE);
//  TIMER_CompareSet(TIMER0, 0, compareValue1);
//
//  // Set the second compare value (don't actually use it, just set the global so
//  // that it can be used by the handler later)
//  compareValue2 = (CMU_ClockFreqGet(cmuClock_TIMER0)
//                    * PULSE_WIDTH
//                    / 1000
//                    / (1 << TIMER_PRESCALE))
//                    + compareValue1;
//
//  // Enable TIMER0 interrupts
//  TIMER_IntEnable(TIMER0, TIMER_IEN_CC0);
//  NVIC_EnableIRQ(TIMER0_IRQn);
//
//  // Enable the TIMER
//  TIMER_Enable(TIMER0, true);
//}

#define BUFFER_SIZE 80
volatile uint32_t rx_data_ready = 0;
volatile char rx_buffer[BUFFER_SIZE];
uint32_t masterRommSize;

#define CMD_SET_MASTER          0x93
#define CMD_INCREMENT_SLAVE     0x94
#define CMD_DINCREMENT_SLAVE    0x95

void USART1_RX_IRQHandler(void)
{
  static uint32_t i = 0;
  static uint32_t flags;
  static uint8_t whaitsData = 0;
  flags = (USART_IntGet(USART1) & (~USART_IF_RXDATAV));
  USART_IntClear(USART1, flags);

  GPIO_PinOutToggle(gpioPortI, 1);

  /* Store incoming data into rx_buffer, set rx_data_ready when a full
  * line has been received
  */

  uint8_t data = USART_Rx(USART1);

  if(whaitsData == 0)
  {

    if(data == CMD_INCREMENT_SLAVE)
    {//TODO not checked
        //send increment
        emberAfFillExternalBuffer((ZCL_CLUSTER_SPECIFIC_COMMAND | ZCL_FRAME_CONTROL_CLIENT_TO_SERVER), 0xFC25, 0x00, "");
    }
    else if(data == CMD_DINCREMENT_SLAVE)
    {//TODO not checked
        //send dincrement
        emberAfFillExternalBuffer((ZCL_CLUSTER_SPECIFIC_COMMAND | ZCL_FRAME_CONTROL_CLIENT_TO_SERVER), 0xFC25, 0x01, "");
    }
    else if(data == CMD_SET_MASTER)
    {//++
        whaitsData = 4;
    }
  }
  else
  {
      //++
    rx_buffer[4 - whaitsData] = data;
    whaitsData--;
    if(whaitsData == 0)
    {
        masterRommSize = rx_buffer[0] | (rx_buffer[1] << 8) | (rx_buffer[2] << 16) | (rx_buffer[3] << 24);
        emberAfWriteServerAttribute(1, 0xFC25, 0x0000, (uint8_t*)(&masterRommSize), ZCL_INT32U_ATTRIBUTE_TYPE);
//        GPIO_PinOutToggle(gpioPortI, 3);
        GPIO_PinOutToggle(gpioPortD, 12);
        //send master room size
    }
  }
}

void sendMasterRoomToESP()
{
  USART_Tx(USART1, CMD_SET_MASTER);
  USART_Tx(USART1, (masterRommSize >> 0 ) & 0xFF);
  USART_Tx(USART1, (masterRommSize >> 8 ) & 0xFF);
  USART_Tx(USART1, (masterRommSize >> 16) & 0xFF);
  USART_Tx(USART1, (masterRommSize >> 24) & 0xFF);
}

boolean emberAfSampleMfgSfecificCiammerSetPoepleZeroCallback(void)
{//TODO not checked
  masterRommSize = 0;
  sendMasterRoomToESP();
  GPIO_PinOutToggle(gpioPortI, 1);
  return true;
}

boolean emberAfSampleMfgSfecificCiammerSetPoepleIncrementCallback(void)
{//TODO not checked
  masterRommSize++;
  sendMasterRoomToESP();
  GPIO_PinOutToggle(gpioPortI, 2);
  return true;
}

boolean emberAfSampleMfgSfecificCiammerSetPoepleDincrementCallback(void)
{//TODO not checked
  if(masterRommSize > 0)
  {
      masterRommSize--;
      sendMasterRoomToESP();
  }
//  GPIO_PinOutToggle(gpioPortI, 3);
  return true;
}


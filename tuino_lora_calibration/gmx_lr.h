/* _____  _____  __  __             _____  _____ 
  / ____||_   _||  \/  |    /\     / ____||_   _|
 | |  __   | |  | \  / |   /  \   | (___    | |  
 | | |_ |  | |  | |\/| |  / /\ \   \___ \   | |  
 | |__| | _| |_ | |  | | / ____ \  ____) | _| |_ 
  \_____||_____||_|  |_|/_/    \_\|_____/ |_____|
  (c) 2017 GIMASI SA                                               

 * gmx_lr.cpp
 *
 *  Created on: March 11, 2017
 *      Author: Massimo Santoli
 *      Brief: Tuino 1 interface for GMX-LR module family
 *      Version: 1.0
 *
 *      License: it's free - do whatever you want! ( provided you leave the credits)
 *
 */


#define GMXLR_KO                      -1 
#define GMXLR_OK                      0 

#define GMXLR_NOT_PRESENT             10
#define GMXLR_NO_SERIAL               20

#define GMXLR_AT_ERROR                30
#define GMXLR_AT_PARAM_ERROR          40
#define GMXLR_AT_BUSY_ERROR           50
#define GMXLR_AT_NOT_JOINED_ERROR     60
#define GMXLR_AT_RX_ERROR             70
#define GMXLR_AT_GENERIC_ERROR        99


#define GMXLR_LED_ON                  1
#define GMXLR_LED_OFF                 0

#define GMXLR_UART_INTERFACE          1
#define GMXLR_SPI_INTERFACE           2

#define GMX_UART_SPEED                9600
#define GMX_UART_TIMEOUT              3000

#define GMX_BOOT0                     1
#define GMX_BOOT_DELAY                2000


#define LORA_OTAA                     1
#define LORA_ABP                      0

#define LORA_ADR_ON                   1
#define LORA_ADR_OFF                  0

#define LORA_CLASS_A                 "A"
#define LORA_CLASS_C                 "C"

#define LORA_DCS_ON                   1
#define LORA_DCS_OFF                  0

#define LORA_CFM_ON                   1
#define LORA_CFM_OFF                  0

#define LORA_MESSAGE_CONFIRMED        1
#define LORA_MESSAGE_NOT_CONFIRMED    0

#define LORA_NETWORK_JOINED           1
#define LORA_NETWORK_NOT_JOINED       0

#define LORA_SF7                      5
#define LORA_SF8                      4
#define LORA_SF9                      3
#define LORA_SF10                     2
#define LORA_SF11                     1
#define LORA_SF12                     0


//#define ENABLE_SET_DEVEUI
 
/*
 * GMX-LR1 Init function
 *  the only parameter is the function for the RxData callback
 */
 
byte gmxLR_init(void (*function)());
byte gmxLR_getVersion(String& version);

byte gmxLR_getRegion(String &region); 
byte gmxLR_setRegion(String region);


/*
 *  OTAA LoRaWAN Parameters 
 */
 
byte gmxLR_getDevEui(String& devEui);

#ifdef ENABLE_SET_DEVEUI
byte gmxLR_setDevEui(String devEui);
#endif

byte gmxLR_getAppEui(String& appEui);
byte gmxLR_setAppEui(String appEui);

byte gmxLR_getAppKey(String& appKey);
byte gmxLR_setAppKey(String appKey);

byte gmxLR_getClass(String& lrclass);
byte gmxLR_setClass(String lrclass);

/*
 * Network Join and Confimration
 */

byte gmxLR_Join(void);
byte gmxLR_isNetworkJoined(void);

byte gmxLR_setJoinMode(byte mode);
byte gmxLR_getJoinMode(String& mode);

byte gmxLR_getConfirmationMode(void);
byte gmxLR_setConfirmationMode(String cfm);
byte gmxLR_getMessageConfirmation(void);

/*
 * Lower level LoRaWAN Functions
 */ 

byte gmxLR_getADR();
byte gmxLR_setADR(String on_off);

byte gmxLR_getDutyCycle();
byte gmxLR_setDutyCycle(String dutycyle);

byte gmxLR_getRSSI(String& rssi);
byte gmxLR_getSNR(String& snr);

byte gmxLR_getTXPower(String& power);
byte gmxLR_setTXPower(String power);

byte gmxLR_getJoinRX1Delay(String& joinrx1);
byte gmxLR_setJoinRX1Delay(String joinrx1);
byte gmxLR_getJoinRX2Delay(String& joinrx2);
byte gmxLR_setJoinRX1Delay(String joinrx2);
byte gmxLR_getRX1Delay(String& rx1);
byte gmxLR_setRX1Delay(String rx1);
byte gmxLR_getRX2Delay(String& rx2);
byte gmxLR_setRX2Delay(String rx2);
byte gmxLR_getRX2DataRate(String& rx2dr);
byte gmxLR_setRX2DataRate(String rx2dr);
byte gmxLR_getDataRate(String& dr);


/*
 * TX & RX Data
 */ 
byte gmxLR_TXData(String data);
byte gmxLR_TXData(String data, int port);
byte gmxLR_RXData(String& data, int *port);


/*
 *  ABP
 */
 
byte gmxLR_getDevAddr(String& devAddr);
byte gmxLR_setDevAddr(String devAddr);
byte gmxLR_getNetworkID(String& netId);
byte gmxLR_setNetworkID(String netId);
byte gmxLR_setNetworkSessionKey(String nwsk);
byte gmxLR_getNetworkSessionKey(String& nwsk);
byte gmxLR_setApplicationSessionKey(String appsk);
byte gmxLR_getApplicationSessionKey(String& appsk);


/*
 * Utilities
 */

byte gmxLR_Led(byte led_state);
void gmxLR_Reset(void);
void gmxLR_StringToHex(String string, char *data, int *len );




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
#include "Arduino.h"
#include "gmx_lr.h"
#include "Regexp.h"

//#define DEBUG 0

String gmxSerialString;

// RegExp Engine
MatchState ms;
char buf[512]; 

//
String dummyResponse;

byte gmxLR_interface = GMXLR_UART_INTERFACE; 

void(*_loraRX)();

//
// ISR 
//
ISR(PCINT1_vect) {     
  if (digitalRead(GMX_INT)==0) {
    if (_loraRX)
      _loraRX();
  }
}

void _resetGMX(byte state){

  switch(state){
    case GMX_BOOT0:
      // for compatibility with preproduction TUINO1 boards - where GPIO6 and GPIO5 where inverted
      // in the production board REV E onwards the correct pin is GPIO6
      pinMode(GMX_GPIO5,OUTPUT);
      digitalWrite(GMX_GPIO5,0);
      pinMode(GMX_GPIO6,OUTPUT);
      digitalWrite(GMX_GPIO6,0);
      break;
  }

  pinMode(GMX_RESET,OUTPUT);
  // Reset 
  digitalWrite(GMX_RESET,HIGH);
  delay(100);
  digitalWrite(GMX_RESET,LOW);
  delay(100);
  digitalWrite(GMX_RESET,HIGH);
}


void _log(String data )
{
  #ifdef DEBUG
    Serial.println(data);
  #endif
}

void _sendCmd(String in) {
  int len = in.length();
  long int start_timeout;

  //  send data
  for (int i=0; i<len; i++) {
    if ( gmxLR_interface == GMXLR_UART_INTERFACE )
    {
      Serial1.write(in.charAt(i));
    }
    delay(1);
  }
  
  delay(100);

  
  if ( gmxLR_interface == GMXLR_UART_INTERFACE )
  {
     start_timeout = millis();
     while(Serial1.available()==0){
      if (( millis() - start_timeout ) > GMX_UART_TIMEOUT )
      {
        Serial.println("TIMEOUT on :"+in);
        break;
      }
     }
  }
}

byte _parseResponse(String& response) {
  
  char cmd[128];
  
  gmxSerialString="";
  
  // Read Serial
  while (Serial1.available()>0) 
  {
      while (Serial1.available()>0) 
      {
        char c = Serial1.read(); 
        gmxSerialString += c;
      }
      
      delay(10);
  }

  gmxSerialString.toCharArray(cmd,gmxSerialString.length());
     
  // Parse Response
  ms.Target(cmd);
  char result = ms.Match ("(.*)\r\nOK", 0);
  if ( result == REGEXP_MATCHED )
  { 
    ms.GetCapture (buf, 0);
  
    response = String(buf);
     
    // remove second \r\n => Not very elegant to optimize
    response.toCharArray(cmd,response.length());
    response = String(cmd);
     
    return (GMXLR_OK);
  }

  // Check for Errors
  result = ms.Match ("(.*)\r\nAT_ERRROR", 0);
  if ( result == REGEXP_MATCHED )
  {
    return(GMXLR_AT_ERROR);
  }

  result = ms.Match ("(.*)\r\nAT_PARAM_ERROR", 0);
  if ( result == REGEXP_MATCHED )
  {
    return(GMXLR_AT_PARAM_ERROR);
  }

  result = ms.Match ("(.*)\r\nAT_BUSY_ERROR", 0);
  if ( result == REGEXP_MATCHED )
  {
    return(GMXLR_AT_BUSY_ERROR);
  }

  result = ms.Match ("(.*)\r\nAT_NO_NETWORK_JOINED", 0);
  if ( result == REGEXP_MATCHED )
  {
    return(GMXLR_AT_NOT_JOINED_ERROR);
  }

  result = ms.Match ("(.*)\r\nAT_RX_ERROR", 0);
  if ( result == REGEXP_MATCHED )
  {
      return(GMXLR_AT_RX_ERROR);
  }
    
  return(GMXLR_AT_GENERIC_ERROR);
}


/* GMXLR Commands Interface */

byte gmxLR_init(void( *callback)()) 
{
    String response;
    byte init_status = GMXLR_KO;
    
    _log("GMXLR Init");

    _resetGMX(GMX_BOOT0);

    // Init Interface
    if ( gmxLR_interface == GMXLR_UART_INTERFACE )
    {
        // Setup Serial
        if(Serial1) {
            Serial1.begin(GMX_UART_SPEED);
           _log("GMXLR Serial Interface");
            init_status = GMXLR_OK;
        } 
        else 
        {
          return(GMXLR_NO_SERIAL);
        }
    }

    // Setup Interrupt PIN for Rx
    *digitalPinToPCICR(GMX_INT) |= (1<<digitalPinToPCICRbit(GMX_INT));
    *digitalPinToPCMSK(GMX_INT) |= (1 << digitalPinToPCMSKbit(GMX_INT));

    // set RX callback
    _loraRX = callback;

    // delay to wait BootUp of GMX-LR
    delay(GMX_BOOT_DELAY);
    
    _sendCmd( "AT\r" );
    return(_parseResponse(response));
    
}

/* DevEui */
byte gmxLR_getDevEui(String& devEui)
{
   _sendCmd( "AT+DEUI=?\r" );
   return( _parseResponse(devEui) );
}


#ifdef ENABLE_SET_DEVEUI
byte gmxLR_setDevEui(String devEui)
{
   _sendCmd( "AT+DEUI="+devEui+"\r" );
   return( _parseResponse(dummyResponse) );
}
#endif

/* AppEui */
byte gmxLR_getAppEui(String& appEui)
{
   _sendCmd( "AT+APPEUI=?\r" );
   return( _parseResponse(appEui) );
}


byte gmxLR_setAppEui(String appEui)
{
   _sendCmd( "AT+APPEUI="+appEui+"\r" );
   return( _parseResponse(dummyResponse) );
}

/* AppKey */
byte gmxLR_getAppKey(String& appKey)
{
   _sendCmd( "AT+APPKEY=?\r" );
   return( _parseResponse(appKey) );
}


byte gmxLR_setAppKey(String appKey)
{
   _sendCmd( "AT+APPKEY="+appKey+"\r" );
   return( _parseResponse(dummyResponse) );
}

/* Join */

byte gmxLR_Join(void)
{
   _sendCmd( "AT+JOIN\r" );
   return( _parseResponse(dummyResponse) );
}

byte gmxLR_setJoinMode(byte mode)
{
  if ( mode == LORA_OTAA )
    _sendCmd( "AT+NJM=1\r" );
  else if (mode == LORA_ABP )
    _sendCmd("AT+NJM=0\r");
  else
    return( GMXLR_AT_PARAM_ERROR );
      
  return( _parseResponse(dummyResponse) );
}

byte gmxLR_getJoinMode(String& mode)
{
  _sendCmd( "AT+NJM=?\r" );
  return( _parseResponse(mode) );
}

// returns true or false
byte gmxLR_isNetworkJoined(void)
{
  byte status;
  char cmd[16];
  
  _sendCmd( "AT+NJS=?\r" );
  
  if ( (status = _parseResponse(dummyResponse)) == GMXLR_OK ) 
  {
    dummyResponse.toCharArray(cmd,dummyResponse.length());
    if (cmd[0] == '0'){
      return LORA_NETWORK_NOT_JOINED;
    }
    if (cmd[0] == '1'){
      return LORA_NETWORK_JOINED;
    }
  }   

  return(status);    
}

// Lora Network

//modified by Micha Burger 09.03.2018
byte gmxLR_setSF(String sf, String& ret){
  
  _sendCmd( "AT+DR="+sf+"\r" );

  return(_parseResponse(ret));    

}

// returns true or false
byte gmxLR_getADR(void)
{
  byte status;
  char cmd[16];
  
  _sendCmd( "AT+ADR=?\r" );
  
  if ( (status = _parseResponse(dummyResponse)) == GMXLR_OK ) 
  {
   dummyResponse.toCharArray(cmd,dummyResponse.length());
    if (cmd[0] == '0'){
      return LORA_ADR_OFF;
    }
    if (cmd[0] == '1'){
      return LORA_ADR_ON;
    }
  }   

  return(status);    
}

byte gmxLR_setADR(String on_off)
{  
  _sendCmd( "AT+ADR="+on_off+"\r" );
   return( _parseResponse(dummyResponse) );
}

byte gmxLR_getDutyCycle(void)
{
  byte status;
  char cmd[16];
  
  _sendCmd( "AT+DCS=?\r" );
  
  if ( (status = _parseResponse(dummyResponse)) == GMXLR_OK ) 
  {
   dummyResponse.toCharArray(cmd,dummyResponse.length());
    if (cmd[0] == '0'){
      return LORA_DCS_OFF;
    }
    if (cmd[0] == '1'){
      return LORA_DCS_ON;
    }
  }   

  return(status);    
}

byte gmxLR_setDutyCycle(String dcs)
{  
  _sendCmd( "AT+DCS="+dcs+"\r" );
   return( _parseResponse(dummyResponse) );
}

byte gmxLR_getClass(String& lr_class)
{  
  _sendCmd( "AT+CLASS=?\r" );
   return( _parseResponse(lr_class) );
}

byte gmxLR_setClass(String lr_class)
{  
  _sendCmd( "AT+CLASS="+lr_class+"\r" );
   return( _parseResponse(dummyResponse) );
}



// Info Calls
byte gmxLR_getRSSI(String& rssi)
{
  _sendCmd( "AT+RSSI=?\r" );
  return( _parseResponse(rssi) );
}


byte gmxLR_getSNR(String& snr)
{
  _sendCmd( "AT+SNR=?\r" );
  return( _parseResponse(snr) );
}

// Transmit Power
byte gmxLR_getTXPower(String& power)
{
  _sendCmd( "AT+TXP=?\r" );
  return( _parseResponse(power) );
}

byte gmxLR_setTXPower(String power)
{
  _sendCmd( "AT+TXP="+power+"\r" );
  return( _parseResponse(dummyResponse) );
}

// Low Level Params
// JoinRX1Dl
// JoinRX2Dl
// RX1
// RX2
// RX1DR
// RX2DR
// RX2FQ

byte gmxLR_getJoinRX1Delay(String& joinrx1)
{
  _sendCmd( "AT+JN1DL=?\r" );
  return( _parseResponse(joinrx1) );
}

byte gmxLR_setJoinRX1Delay(String joinrx1)
{
  _sendCmd( "AT+JN1DL="+joinrx1+"\r" );
  return( _parseResponse(dummyResponse) );
}


byte gmxLR_getJoinRX2Delay(String& joinrx2)
{
  _sendCmd( "AT+JN2DL=?\r" );
  return( _parseResponse(joinrx2) );
}

byte gmxLR_setJoinRX2Delay(String joinrx2)
{
  _sendCmd( "AT+JN2DL="+joinrx2+"\r" );
  return( _parseResponse(dummyResponse) );
}

byte gmxLR_getRX1Delay(String& rx1)
{
  _sendCmd( "AT+RX1DL=?\r" );
  return( _parseResponse(rx1) );
}

byte gmxLR_setRX1Delay(String rx1)
{
  _sendCmd( "AT+RX1DL="+rx1+"\r" );
  return( _parseResponse(dummyResponse) );
}

byte gmxLR_getRX2Delay(String& rx2)
{
  _sendCmd( "AT+RX2DL=?\r" );
  return( _parseResponse(rx2) );
}

byte gmxLR_setRX2Delay(String rx2)
{
  _sendCmd( "AT+RX2DL="+rx2+"\r" );
  return( _parseResponse(dummyResponse) );
}


byte gmxLR_getRX2DataRate(String& rx2dr)
{
  _sendCmd( "AT+RX2DR=?\r" );
  return( _parseResponse(rx2dr) );
}

byte gmxLR_setRX2DataRate(String rx2dr)
{
  _sendCmd( "AT+RX2DR="+rx2dr+"\r" );
  return( _parseResponse(dummyResponse) );
}


// TX & RX Data

byte _TXData(String data, int port) {

    String _port = String(port);
    
   _sendCmd( "AT+SENDB="+_port+":"+data+"\r" );
    return( _parseResponse(dummyResponse) );
}

byte gmxLR_TXData(String data) {
  return( _TXData(data,1));
}

byte gmxLR_TXData(String data, int port) {
  return( _TXData(data,port ));  
}


byte gmxLR_RXData(String& data, int *port)
{
  byte status;
  String str1,str2;
  
    // need a delay because the interrupt arrives too fast
    delay(100);
   _sendCmd( "AT+RECVB=?\r" );
   status = _parseResponse(dummyResponse);

   if (status == GMXLR_OK )
   {
      byte index = dummyResponse.indexOf(":");
      if ( index != -1 )
      {
        str1 = dummyResponse.substring(0,index);
        str2 = dummyResponse.substring(index+1,dummyResponse.length());
        data = String(str2);
        *port = str1.toInt();
      }   
   }
}

/*
 *  ABP Part
 */

byte gmxLR_getDevAddr(String& devAddr)
{
   _sendCmd( "AT+DADDR=?\r" );
   return( _parseResponse(devAddr) );
}


byte gmxLR_setDevAddr(String devAddr)
{
   _sendCmd( "AT+DADDR="+devAddr+"\r" );
   return( _parseResponse(dummyResponse) );
}

/*
 * 
 */

byte gmxLR_getNetworkID(String& netId)
{
   _sendCmd( "AT+NWKID=?\r" );
   return( _parseResponse(netId) );
}

byte gmxLR_setNetworkID(String netId)
{
   _sendCmd( "AT+NWKID="+netId+"\r" );
   return( _parseResponse(dummyResponse) );
}

/*
 * 
 */

byte gmxLR_setNetworkSessionKey(String nwsk)
{
   _sendCmd( "AT+NWKSKEY="+nwsk+"\r" );
   return( _parseResponse(dummyResponse) );
}

byte gmxLR_getNetworkSessionKey(String& nwsk)
{
   _sendCmd( "AT+NWKSKEY=?\r" );
   return( _parseResponse(nwsk) );
}

/*
 * 
 */
 
byte gmxLR_setApplicationSessionKey(String appsk)
{
   _sendCmd( "AT+APPSKEY="+appsk+"\r" );
   return( _parseResponse(dummyResponse) );
}

byte gmxLR_getApplicationSessionKey(String& appsk)
{
   _sendCmd( "AT+APPSKEY=?\r" );
   return( _parseResponse(appsk) );
}

/*
 * 
 */
 
byte gmxLR_getConfirmationMode(void)
{
  byte status;
  char cmd[16];
  
  _sendCmd( "AT+CFM=?\r" );
  
  if ( (status = _parseResponse(dummyResponse)) == GMXLR_OK ) 
  {
   dummyResponse.toCharArray(cmd,dummyResponse.length());
    if (cmd[0] == '0'){
      return LORA_CFM_OFF;
    }
    if (cmd[0] == '1'){
      return LORA_CFM_ON;
    }
  }   

  return(status);    
}

byte gmxLR_setConfirmationMode(String cfm)
{
   _sendCmd( "AT+CFM="+cfm+"\r" );
   return( _parseResponse(dummyResponse) );
}

/*
 * 
 */
 
byte gmxLR_getMessageConfirmation(void)
{
  byte status;
  char cmd[16];
  
  _sendCmd( "AT+CFS=?\r" );
  
  if ( (status = _parseResponse(dummyResponse)) == GMXLR_OK ) 
  {
   dummyResponse.toCharArray(cmd,dummyResponse.length());
    if (cmd[0] == '0'){
      return LORA_MESSAGE_NOT_CONFIRMED;
    }
    if (cmd[0] == '1'){
      return LORA_MESSAGE_CONFIRMED;
    }
  }   

  return(status);    
}

/*
 * Utilities
 */

byte gmxLR_Led(byte led_state){
  String led_num = String(led_state);
  
  _sendCmd( "AT+LED=4:"+led_num+"\r" );
  return( _parseResponse(dummyResponse));
   
}

void gmxLR_Reset(void){
  String command = String("ATZ\r");
  int len = command.length();
  
  //  send data
  for (int i=0; i<len; i++) {
    if ( gmxLR_interface == GMXLR_UART_INTERFACE )
    {
      Serial1.write(command.charAt(i));
    }
    delay(1);
  }
  
  delay(GMX_BOOT_DELAY);
 
}




void gmxLR_StringToHex(String string, char *data, int *len )
{
  char tmp[255];
  char temp[2];
  int j=0;
  
  string.toCharArray(tmp,255);

  for (int i = 0; i < strlen(tmp); i += 2)
  {
    strncpy(temp, &tmp[i], 2);

    temp[0] = toupper(temp[0]);    // Convert to upper case
    temp[1] = toupper(temp[1]);
    
    // Convert hex string to numeric:
    data[j] = (temp[0] <= '9') ? (temp[0] - '0') : (temp[0] - 'A' + 10);
    data[j] *= 16;
    data[j] += (temp[1] <= '9') ? (temp[1] - '0') : (temp[1] - 'A' +10);

    j++;
  }

    *len = j;
}


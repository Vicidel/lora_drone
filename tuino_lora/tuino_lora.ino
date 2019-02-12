/* _____  _____  __  __             _____  _____ 
  / ____||_   _||  \/  |    /\     / ____||_   _|
 | |  __   | |  | \  / |   /  \   | (___    | |  
 | | |_ |  | |  | |\/| |  / /\ \   \___ \   | |  
 | |__| | _| |_ | |  | | / ____ \  ____) | _| |_ 
  \_____||_____||_|  |_|/_/    \_\|_____/ |_____|
  (c) 2017 GIMASI SA                                               

 * tuino_lora.ino
 *
 * Based on: October 11, 2017
 *      Author: Massimo Santoli
 *      Brief: Example Sketch to use LoRa GMX-LR1 Module
 *      Version: 1.2
 *      
 *  Created on: February 7, 2019
 *      Author: Victor Delafontaine
 *
 *      License: it's free - do whatever you want! ( provided you leave the credits)
 *
 */
 
#include "gmx_lr.h"

long int timer_period_to_tx = 20000;    // time between transmissions
long int timer_millis_lora_tx = 0;      // time of transmission (?)
int ledState = 0;

// Declare the interrupt callback function for RX
bool data_received=false;
void loraRx(){
  data_received = true;
}


void setup() {
  // put your setup code here, to run once:
  String DevEui;
  String AppEui, NewAppEui, _AppEui;
  String AppKey, NewAppKey, _AppKey;
  String loraClass,LoRaWANClass;
  String version;

  char string[64];

  String adr,dcs,dxrate;

  byte join_status;
  int join_wait;
  
  Serial.begin(9600);
  Serial.println("Starting");

  pinMode(LED_BUILTIN, OUTPUT);

  // GMX-LR init pass callback function
  gmxLR_init(&loraRx);

  gmxLR_getVersion(version);
  Serial.println("GMXLR Version:"+version);

  // Set AppEui and AppKey, uncomment these if you want to change the default keys
  // NewAppEui = "00:00:00:00:00:00:00:00";
  // NewAppKey = "6d:41:46:39:67:4e:30:56:46:4a:62:4c:67:30:58:33";

  LoRaWANClass = "A";

  Serial.println("Joining...");
  join_wait = 0; 
  while((join_status = gmxLR_isNetworkJoined()) != LORA_NETWORK_JOINED) {

  
    if ( join_wait == 0 )
    {

     gmxLR_getAppEui(AppEui);
     if (NewAppEui.length() > 0 )
     {
        AppEui.trim();
        
        Serial.println("**** UPDATING AppEUI ****");
        if ( !AppEui.equals(NewAppEui) )
        {
          Serial.println("Setting AppEui:"+NewAppEui);
          gmxLR_setAppEui(NewAppEui);
        }
        else
        {
          Serial.println("AppEui is already:"+AppEui);
        }
    }

    gmxLR_getAppKey(AppKey);
    if (NewAppKey.length() > 0 )
    {
      AppKey.trim();
      
      Serial.println("**** UPDATING AppKey ****");
      if ( !AppKey.equals(NewAppKey) )
      {
          Serial.println("Setting AppKey:"+NewAppKey);
          gmxLR_setAppKey(NewAppKey);
      }
      else
      {
          Serial.println("AppKey is already:"+AppKey);
      }
    }  
    
      // Disable Duty Cycle  ONLY FOR DEBUG!
      gmxLR_setDutyCycle("0");

      // Set LoRaWAN Class
      gmxLR_setClass(LoRaWANClass);

      Serial.println("LoRaWAN Params:");
      gmxLR_getDevEui(DevEui);
      Serial.println("DevEui:"+DevEui);
      gmxLR_getAppEui(_AppEui);
      Serial.println("AppEui:"+_AppEui);
      gmxLR_getAppKey(_AppKey);
      Serial.println("AppKey:"+_AppKey);
      gmxLR_getClass(loraClass);
      Serial.println("Class:"+loraClass);
      adr = String( gmxLR_getADR() );
      Serial.println("ADR:"+adr);
      dcs = String( gmxLR_getDutyCycle() );
      Serial.println("DCS:"+dcs);
      gmxLR_getRX2DataRate(dxrate);
      Serial.println("RX2 DataRate:"+dxrate);
      
      gmxLR_Join();
    }
    Serial.print("Join:");
    Serial.println(join_wait);
  
    join_wait++;

    // Not really necessary - but we reset everything after a timeout
    if (!( join_wait % 100 )) {
      gmxLR_Reset();
      join_wait = 0;
    }

    delay(5000);

  };
  
  Serial.println("Network Joined!");
 
}

void loop() {
  // put your main code here, to run repeatedly:

  long int delta_lora_tx;
  String RSSI;
  String SNR;
  String rx_data;
  char rx_buf[128];
  int rx_buf_len;
  int port;
  int _snr;

   delta_lora_tx = millis() - timer_millis_lora_tx;
   
   // Transmit Period 
   // check delta TX Timeout
   if ( delta_lora_tx > timer_period_to_tx) {
    Serial.println("TX DATA");

    // Transmit Data - as HEX String
    gmxLR_TXData("0A");
    // possible data: 0A=10m, 14=20m, B2=50m, 64=100m, 96=150m, C8=200m, FF=other
    
    // get RSSI and SNR of last received packet
    gmxLR_getRSSI(RSSI);
    Serial.println("RSSI: "+RSSI);
    gmxLR_getSNR(SNR);
    _snr = SNR.toInt();
    Serial.print("SNR: ");
    Serial.println(_snr);
    

     timer_millis_lora_tx = millis();
   }

   if (data_received)
   {
      // retrive RX data from GMX-LR module. 
      // rx_data is a HEX String
      // port is also returned
      gmxLR_RXData(rx_data,&port);

      //convert HEX String to byte array for simpler packet decoding
      gmxLR_StringToHex(rx_data, rx_buf, rx_buf_len );

      Serial.println("LORA RX DATA:"+rx_data);  
      Serial.print("LORA RX PORT:");
      Serial.println(port);

      if ( rx_buf[0] == 0x01 )
        digitalWrite(LED_BUILTIN,HIGH);
      else
        digitalWrite(LED_BUILTIN,LOW);
        
      // clear flag
      data_received = false;
   }
  
}

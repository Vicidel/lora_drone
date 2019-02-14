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
 * Inspired by: March 9, 2018
 *      Author: Micha Burger michaburger92@gmail.com
 *      Brief: LoRaWAN GPS mapper for signal strength
 *      Version: 1.0
 *      
 *  Created on: February 7, 2019
 *      Author: Victor Delafontaine
 *      Brief: LoRaWAN localization using drone-mounted gateway
 *      Version: 1.1
 *
 *      License: it's free - do whatever you want! ( provided you leave the credits)
 *
 */

// included librairies
#include "gmx_lr.h"
#include "Regexp.h"
#include "SeeedOLED.h"
#include "display_utils.h"
#include <Wire.h>

// LoRa transmission parameters
#define DISABLE_ADR 0   // ADR has to be enabled to set custom SF
#define SF LORA_SF7     // only SF=7
long int timer_period_to_tx = 5000;     // time between transmissions
long int timer_millis_lora_tx = 0;      // time of last transmission

// user set LED
int ledState = 0;

// where button is plugged
int buttonPin = D4;

// string to display
String oled_string;

// Declare the interrupt callback function for RX
bool data_received=false;
void loraRx(){
  data_received = true;
}

// FSM state
int current_state = 0;  // from 0 to 7, 8 states
int message_sent = 0;


void setup() {
    // put your setup code here, to run once:
    String DevEui;
    String AppEui, NewAppEui, _AppEui;
    String AppKey, NewAppKey, _AppKey;
    String loraClass,LoRaWANClass;
    String version;
    
    char send_string[64];
    
    String adr,dcs,dxrate;
    
    byte join_status;
    int join_wait;
    
    Wire.begin();
    Serial.begin(9600);
    Serial.println("Starting");
    
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(buttonPin, INPUT);
    
    // OLED init
    SeeedOled.init();                  //initialze SEEED OLED display
    SeeedOled.clearDisplay();          //clear the screen and set start position to top left corner
    SeeedOled.setNormalDisplay();      //Set display to normal mode (i.e non-inverse mode)
    SeeedOled.setHorizontalMode();     //Set addressing mode to Page Mode
    splashScreen();
    
    // GMX-LR init pass callback function
    gmxLR_init(&loraRx);
    gmxLR_getVersion(version);
    Serial.println("GMXLR Version:"+version);

    // set LoRa class
    LoRaWANClass = "A";

    // print on display and serial
    SeeedOled.clearDisplay();
    oledPut(0, "Joining...");
    Serial.println("Joining...");
    
    join_wait = 0; 
    while((join_status = gmxLR_isNetworkJoined()) != LORA_NETWORK_JOINED) {

        // first start only
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
                    Serial.println("AppEui is already:"+AppEui);
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
                    Serial.println("AppKey is already:"+AppKey);
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

        // print number of attempts
        Serial.print("Join:");
        Serial.println(join_wait);
        SeeedOled.setTextXY(1, 0);
        sprintf(send_string, "Attempt: %d", join_wait);
        SeeedOled.putString(send_string);
        
        join_wait++;
      
        // Not really necessary - but we reset everything after a timeout
        if (!( join_wait % 100 )) {
            gmxLR_Reset();
            join_wait = 0;
        }

        // one second between each trial
        delay(1000);
    };

    // print network joined
    SeeedOled.setTextXY(2, 0);
    SeeedOled.putString("Joined!");
    Serial.println("Network Joined!");

    
    /*               ::::   block to disable ADR and force set SF   ::::
    // disable ADR ?
    if(DISABLE_ADR) gmxLR_setADR("0");
    Serial.print("ADR: ");
    Serial.println(gmxLR_getADR());
    SeeedOled.setTextXY(4, 0);
    if (gmxLR_getADR()==0)
        SeeedOled.putString("ADR inactive");
    else
        SeeedOled.putString("ADR active");
    
    // set spreading factor
    String sf = "";
    gmxLR_setSF(String(SF), sf);
    Serial.print("Data rate (SF): ");
    Serial.println(sf);
    SeeedOled.setTextXY(5, 0);
    sprintf(send_string,"SF: %s", SF);
    SeeedOled.putString(send_string);

    // get TX power
    Serial.print("TXpow: ");
    String answ = "";
    gmxLR_getTXPower(answ);
    Serial.println(answ);
    */
    
    delay(1000);
    SeeedOled.clearDisplay();
}


// put new information
void oledPut(int line, char *str){
    SeeedOled.setTextXY(line,0);
    SeeedOled.putString(str);
    delay(30);
}

// put current state
void oledPutState(int line, int current_state){
    char send_string[64];
    SeeedOled.setTextXY(line, 0);
    sprintf(send_string,"State: %d", current_state);
    SeeedOled.putString(send_string);
}

void oledPutNumMessage(int line){
    char send_string[64];
    SeeedOled.setTextXY(line, 0);
    sprintf(send_string,"Messages: %d", message_sent);
    SeeedOled.putString(send_string);
}

// if time has passed, send a message
void sendLora(char *str){

    // LoRa parameters
    String RSSI;
    String SNR;
    String rx_data;
    char rx_buf[128];
    int rx_buf_len;
    int port;
    int _snr;

    // time from last emission
    long int delta_lora_tx = millis() - timer_millis_lora_tx;

    // check delta TX Timeout
    if ( delta_lora_tx > timer_period_to_tx) {
        
        // set last emitting time as now
        timer_millis_lora_tx = millis();

        // transmit the string passed in argument
        gmxLR_TXData(str);    // Transmit data as HEX String
        
        // get RSSI and SNR of last received packet
        gmxLR_getRSSI(RSSI);
        Serial.println("RSSI: "+RSSI);
        gmxLR_getSNR(SNR);
        _snr = SNR.toInt();
        Serial.print("SNR: ");
        Serial.println(_snr);

        // increase message sent
        message_sent++; 
    }
}

void loop() {
  
    // when button cliqued
    if (digitalRead(buttonPin) == HIGH)
    {
        // as state changes, screen resets
        SeeedOled.clearDisplay();

        // state change
        current_state++;
        if (current_state == 12)
            current_state = 0;

        // time for realeasing button
        delay(200);
    }    

    // print 
    oledPut(0, "Vic LoRa drone");
    oledPutState(2, current_state);
    
    switch(current_state)
    {
        // current_state even: not sending, move in position
        case 0:
            oledPut(3, "Not sending");
            oledPut(4, "  go to 10m");
            message_sent = 0;
            break;
        case 2:
            oledPut(3, "Not sending");
            oledPut(4, "  go to 20m");
            message_sent = 0;
            break;
        case 4:
            oledPut(3, "Not sending");
            oledPut(4, "  go to 50m");
            message_sent = 0;
            break;
        case 6:
            oledPut(3, "Not sending");
            oledPut(4, "  go to 100m");
            message_sent = 0;
            break;
        case 8:
            oledPut(3, "Not sending");
            oledPut(4, "  go to 150m");
            message_sent = 0;
            break;
        case 10:
            oledPut(3, "Not sending");
            oledPut(4, "  go to 200m");
            message_sent = 0;
            break;

        // current_state odd: sending data to server, don't move it
        case 1:
            oledPut(3, "Sending: 0A=10m");
            oledPutNumMessage(4);
            sendLora("0A");
            break;
        case 3:
            oledPut(3, "Sending: 14=20m");
            oledPutNumMessage(4);
            sendLora("14");
            break;
        case 5:
            oledPut(3, "Sending: B2=50m");
            oledPutNumMessage(4);
            sendLora("B2");
            break;
        case 7:
            oledPut(3, "Sending: 64=100m");
            oledPutNumMessage(4);
            sendLora("64");
            break;
        case 9:
            oledPut(3, "Sending: 96=150m");
            oledPutNumMessage(4);
            sendLora("96");
            break;
        case 11:
            oledPut(3, "Sending: C8=200m");
            oledPutNumMessage(4);
            sendLora("C8"); 
            break;
    }
    
}

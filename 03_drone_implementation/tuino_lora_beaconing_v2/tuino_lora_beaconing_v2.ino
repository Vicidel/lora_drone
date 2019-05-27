/*
        Tuino GPS mapper state machine
        
        Created on: March 9, 2018
        
                Author: Micha Burger michaburger92@gmail.com
                Brief: LoRaWAN GPS mapper for signal strength
                Version: 1.0

                Based on: Tuino LoRa example by Gimasi

                License: it's free - do whatever you want! ( provided you leave the credits)

*/

#include "gmx_lr.h"
#include "Regexp.h"
#include "SeeedOLED.h"
#include "display_utils.h"
#include "TinyGPS++.h"

#include <Wire.h>

//DHT
#include "DHT.h"
#define DHTPIN 5
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

//State machine definitions
enum {NO_GPS, NO_FIX, CHECK_PRECISION, GPS_IMPROVE, SEND, ERR};
#define NB_STATES 7
#define STATE_STR_LEN 20
char states[NB_STATES][STATE_STR_LEN] = {"NO_GPS          ", "NO_FIX          ", "CHECK_PRECISION ", 
                                                                                 "GPS_IMPROVE     ", "SEND            ", "ERROR           "};
#define NB_TRANSMISSIONS 12

//minimum hdop to accept GPS fix
#define MIN_HDOP 500
//minimum hdop to send point with lorawan
#define HDOP_SEND 500

#define SF LORA_SF7
//Tested minimum delay times between two transmissions when changing txpower, for each SF
int min_delay_txpow[6] = {2500,2000,1500,1000,1000,1000};

unsigned int fsm_state = NO_GPS;
unsigned int fsm_pck_count = 0;
unsigned long five_minutes = 300000;
unsigned long fsm_flag;
unsigned long state_entry;
unsigned long eval_wait_flag;
unsigned int pck_counter = 0;

enum {NONE, SEND_WITH_GPS, SEND_WITHOUT_GPS};
int current_state = NONE;

#define DISABLE_ADR 0 //ADR has to be enabled to set custom SF

#define SECOND 1000
int t_check_fix = 5*SECOND;
int t_improve = SECOND;
int t_update = 200;
//the time to wait with registering after GPS fix
int t_precision = 15*SECOND; //ATTENTION: Overflow at 32'000
int t_lora_tx = SECOND;
int t_lora_val = 20*SECOND;
int t_eval = 3*SECOND;
int t_trilat = 3*SECOND;

static const uint32_t GPSBaud = 4800;

char oled_string[17];

int buttonPin = D4;
int humidityPin = D5;

// The TinyGPS++ object
TinyGPSPlus gps;

// LoRa RX interrupt
bool data_received = false;

void loraRx() {
    data_received = true;
}

void setup() {
    // put your setup code here, to run once:

    //state machine setup
    fsm_flag = 0;

    //button
    pinMode(buttonPin,INPUT);
 
    String DevEui,NewDevEui;
    String AppEui,NewAppEui;
    String AppKey,NewAppKey;
    String loraClass,LoRaWANClass;

    String adr, dcs, dxrate, dr;

    byte join_status;
    int join_wait;

    Wire.begin();
    Serial.begin(GPSBaud);
    Serial.println("Starting");

    //humidity and temperature sensor
    dht.begin();

    // Init Oled
    SeeedOled.init();  //initialze SEEED OLED display

    SeeedOled.clearDisplay();         //clear the screen and set start position to top left corner
    SeeedOled.setNormalDisplay();     //Set display to normal mode
    SeeedOled.setHorizontalMode();    //Set addressing mode to Page Mode
    SeeedOled.setRotation(true);      //Modified SeeedOled library


    // GMX-LR init pass callback function
    gmxLR_init(&loraRx);

 
    // Set AppEui and AppKey
    // Uncomment these if you want to change the default keys
    
    // NewAppEui = "00:00:00:00:00:00:00:00";
    // NewAppKey = "6d:41:46:39:67:4e:30:56:46:4a:62:4c:67:30:58:33";

    // Region is available in GMX Firmware v2.0
    // available regions: EU868,US915,IN865,AS923,AU915,CN779,KR920
    // NewLoraRegion = "AS923";

    //Default class: A
    LoRaWANClass = "A";


    #ifdef MULTIREGION
        gmxLR_getRegion(Region);
        if (NewLoraRegion.length() > 0 )
        {
            Region.trim();
            
            //Serial.println("**** UPDATING Region ****");
        
            if ( !Region.equals(NewLoraRegion) )
            {
                 Serial.println("Setting Region:"+NewLoraRegion);
                 gmxLR_setRegion(NewLoraRegion);
                 // reboot GMX_LR1
                 gmxLR_Reset();
            }
            else
            {
                 Serial.println("Region is already:"+Region);
            }
        }
    #endif

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

    // Show LoRaWAN Params on OLED
    gmxLR_getDevEui(DevEui);
    gmxLR_getAppKey(AppKey);
    gmxLR_getAppEui(AppEui);
    displayLoraWanParams(DevEui, AppEui, AppKey);

    delay(2*SECOND);
    
    SeeedOled.clearDisplay();
    SeeedOled.setTextXY(0, 0);
    SeeedOled.putString("Joining...");

    Serial.println("Joining...");
    join_wait = 0;
    while ((join_status = gmxLR_isNetworkJoined()) != LORA_NETWORK_JOINED) {


        if ( join_wait == 0 )
        {
            Serial.println("LoRaWAN Params:");
            gmxLR_getDevEui(DevEui);
            Serial.println("DevEui:" + DevEui);
            gmxLR_getAppEui(AppEui);
            Serial.println("AppEui:" + AppEui);
            gmxLR_getAppKey(AppKey);
            Serial.println("AppKey:" + AppKey);
            gmxLR_getClass(loraClass);
            Serial.println("Class:" + loraClass);
            adr = String( gmxLR_getADR() );
            Serial.println("ADR:" + adr);
            dcs = String( gmxLR_getDutyCycle() );
            Serial.println("DCS:" + dcs);
            gmxLR_getRX2DataRate(dxrate);
            Serial.println("RX2 DataRate:" + dxrate);

            gmxLR_Join();
        }

        SeeedOled.setTextXY(1, 0);
        sprintf(oled_string, "Attempt: %d", join_wait);
        SeeedOled.putString(oled_string);

        join_wait++;

        if (!( join_wait % 100 )) {
            gmxLR_Reset();
            join_wait = 0;
        }

        delay(3*SECOND);

    };

    SeeedOled.setTextXY(2, 0);
    SeeedOled.putString("Joined!");

    //Disable ADR
    if(DISABLE_ADR) gmxLR_setADR("0");
    Serial.print("ADR: ");
    Serial.println(gmxLR_getADR());

    //Set spreading factor
    String sf = "";
    gmxLR_setSF(String(SF),sf);
    Serial.print("Data rate (SF): ");
    Serial.println(sf);
    
    Serial.print("TXpow: ");
    String answ = "";
    gmxLR_getTXPower(answ);
    Serial.println(answ);

    delay(2*SECOND);
    SeeedOled.clearDisplay();

    oledPutInfo();
    oledPut(3,"IDLE");
}

//Send packet with LoRa
void sendLoraTX(int txpow){

    char lora_data[128];
    byte tx_buf[128];
    String tx_data;

    int temperature_int;
    int hum_int;
    long gps_lat = 0.0 * 10000000;
    long gps_lon = 0.0 * 10000000;
    
    //Read temperature and humidity
    temperature_int = dht.readTemperature(false) * 100; //false for celsius
    hum_int = dht.readHumidity() * 100;
                     
    //get GPS data
    gps_lat = gps.location.lat()*10000000;
    gps_lon = gps.location.lng()*10000000;
    
    //speed between 0 and 255 to fit into 1 byte. To be divided by 2 by the server.
    int gps_speed = (int) 2*gps.speed.kmph();
    if(gps_speed > 255) gps_speed = 255;
    else if (gps_speed <0) gps_speed = 0;

    //course is between 0 and 360° --> map between 0 an 180, to be multiplied by 2 by the server
    int gps_course = (int) 0.5*gps.course.deg();
    if(gps_course > 255) gps_course = 255;
    else if(gps_course <0) gps_course = 0;
    
    int nbSat = gps.satellites.value();
    if(nbSat>255) nbSat=255;
    int gpsHDOP = gps.hdop.value();
                     
    //create package
    tx_buf[0] = 0x02; // packet header - multiple data
    tx_buf[1] = (temperature_int & 0xff00 ) >> 8;
    tx_buf[2] = temperature_int & 0x00ff;
    tx_buf[3] = (hum_int & 0xff00 ) >> 8;
    tx_buf[4] = hum_int & 0x00ff;
    tx_buf[5] = (gps_lat & 0xff000000) >> 24;
    tx_buf[6] = (gps_lat & 0x00ff0000) >> 16;
    tx_buf[7] = (gps_lat & 0x0000ff00) >> 8;
    tx_buf[8] = gps_lat & 0x000000ff;
    tx_buf[9] = (gps_lon & 0xff000000) >> 24;
    tx_buf[10] = (gps_lon & 0x00ff0000) >> 16;
    tx_buf[11] = (gps_lon & 0x0000ff00) >> 8;
    tx_buf[12] = gps_lon & 0x000000ff;
    tx_buf[13] = nbSat & 0x00ff;
    tx_buf[14] = (gpsHDOP & 0xff00) >> 8;
    tx_buf[15] = gpsHDOP & 0x00ff;
    tx_buf[16] = gps_speed & 0x00ff;
    tx_buf[17] = gps_course & 0x00ff;
    tx_buf[18] = current_state & 0x00ff;
    tx_buf[19] = txpow & 0x00ff;
                     
    sprintf(lora_data, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X", tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3], tx_buf[4], tx_buf[5], tx_buf[6], tx_buf[7], tx_buf[8], tx_buf[9], tx_buf[10], tx_buf[11], tx_buf[12], tx_buf[13], tx_buf[14], tx_buf[15], tx_buf[16], tx_buf[17], tx_buf[18], tx_buf[19] );
                     
     //displayLoraTX(true); 
     tx_data = String(lora_data);

     String answ = "";

     //multiple trials to set TX power
     for(int i=0; i<5; i++){
         gmxLR_setTXPower(String(txpow));
         delay(50);
         gmxLR_getTXPower(answ);
         if(int(answ[0])-48 == txpow){
             //When ADR is enabled, SF change has to be enforced before every TX
             gmxLR_setSF(String(SF),answ);
             gmxLR_TXData(tx_data);
             /*
             Serial.print("TXpow sent: ");
             Serial.println(tx_pow);
             Serial.print("TXpow chip: ");
     
             gmxLR_getTXPower(answ);
             Serial.println(answ);
             */
             i=5;
         }
     }

     //displayLoraTX(false);
}

void state_change_reset(){
    fsm_pck_count = 0;
    state_entry = millis();
    fsm_flag = millis();
}

//put current state to OLED
void oledPutState(){
    SeeedOled.setTextXY(4, 0);
    SeeedOled.putString(states[fsm_state]);
    oledPutInfo();
}

void oledPutInfo(){
    SeeedOled.setTextXY(0, 0);
    SeeedOled.putString("LoRa on drone");
    SeeedOled.setTextXY(1, 0);
    SeeedOled.putString("Swisscom 2019");
}

//Delete line and put new information
void oledPut(int line, char *str){
    SeeedOled.setTextXY(line, 0);
    //clean line
    SeeedOled.putString("                ");
    delay(30);
    SeeedOled.setTextXY(line, 0);
    SeeedOled.putString(str);
    delay(30);
}

void loop() {
    
    /*// print in serial
    sprintf(oled_string, "State: %d", current_state);
    Serial.println(oled_string);
    sprintf(oled_string, "FSM: %d", fsm_state);
    Serial.println(oled_string);*/
  
    //change suspend mode or track number
    if(digitalRead(buttonPin)){
        switch(current_state)
        {
            case NONE:
                current_state = SEND_WITH_GPS;
                SeeedOled.clearDisplay();
                state_change_reset();
                oledPut(3,"SEND W. GPS");
                oledPutInfo();
                delay(SECOND);
                break;
            case SEND_WITH_GPS:
                current_state = SEND_WITHOUT_GPS;
                SeeedOled.clearDisplay();
                state_change_reset();
                oledPut(3,"SEND WO. GPS");
                oledPutInfo();
                delay(SECOND);
                break;
            case SEND_WITHOUT_GPS:
                current_state = NONE;
                SeeedOled.clearDisplay();
                state_change_reset();
                oledPut(3,"IDLE");
                oledPutInfo();
                delay(SECOND);
                break;
        }
    }


    if(current_state == SEND_WITH_GPS) {
        //check if GPS is still connected
        while (Serial.available() > 0 && !digitalRead(buttonPin)){

            // info coming from serial (GPS)
            if (gps.encode(Serial.read())){
                
                // put state on screen
                oledPutState();
                
                //check GPS fix
                if(fsm_state!=NO_GPS && (!gps.location.isValid() || !gps.time.isValid() || gps.satellites.value()<1)){
                    fsm_state = NO_FIX;
                }
                    
                switch (fsm_state)
                {
                    case NO_GPS: 
                        if (gps.charsProcessed() > 10){
                            fsm_state = NO_FIX;
                        }
                        delay(t_check_fix);
                        break;
            
                    case NO_FIX:
                        if(gps.location.isValid() && gps.time.isValid() && gps.satellites.value()>0){
                            fsm_state = SEND;
                        }
                        delay(t_check_fix);
                        break;

            /*
                    case CHECK_PRECISION:
                        if(gps.hdop.value()<MIN_HDOP && gps.hdop.value()>0){
                            //store time when the fix was accepted
                            fsm_flag = millis();
                            fsm_state = GPS_IMPROVE;
                        }
                        sprintf(oled_string, "HDOP: %d", gps.hdop.value());
                        oledPut(5, oled_string);
                        sprintf(oled_string, "Satellites: %d",gps.satellites.value());
                        oledPut(6, oled_string);
                        delay(t_check_fix);
                        break;
            
                    case GPS_IMPROVE:
                        if(millis()-fsm_flag > t_precision) {
                            fsm_flag = millis();
                            eval_wait_flag = millis();
                            oledPut(5, "Done, start mapping!");
                            if(current_state==SEND_WITH_GPS){
                                fsm_state = SEND;
                            }
                        }
                        else{
                            sprintf(oled_string, "Wait: %d", (int)(0.001 * (t_precision - (millis()-fsm_flag))));
                            oledPut(5, oled_string);
                        }
                        //continuously check fix and precision
                        if(gps.hdop.value()>MIN_HDOP){
                            //store time when the fix was accepted
                            fsm_state = CHECK_PRECISION;
                            oledPutState();
                            oledPut(5, "GPS precision lost");
                        }
                        delay(t_improve);
                        break;*/
            
                    case SEND:
                        /*
                        sprintf(oled_string, "Since: %d s", (int) (0.001 * (millis()-fsm_flag)));
                        oledPut(5, oled_string);
                        sprintf(oled_string, "HDOP: %d", gps.hdop.value());
                        oledPut(6, oled_string);
                        sprintf(oled_string, "Satellites: %d",gps.satellites.value());
                        oledPut(7, oled_string); 
                        
                        if(gps.location.isUpdated()){
                            //only send acceptable HDOP
                            if(gps.hdop.value() > 0 && gps.hdop.value() < HDOP_SEND) {
                                sendLoraTX(0);
                                delay(t_lora_tx);
                            }
                        }
                        else {
                            oledPut(6,"No update, wait.");
                            delay(t_update);
                        }*/
                        // send every 3 seconds
                        if(millis()-fsm_flag>3*SECOND){
                            sendLoraTX(0);
                            delay(min_delay_txpow[SF]);
                            fsm_pck_count++;
                            sprintf(oled_string, "Packets: %d", fsm_pck_count);
                            oledPut(5, oled_string);
                            sprintf(oled_string, "HDOP: %d", gps.hdop.value());
                            oledPut(6, oled_string);
                            sprintf(oled_string, "Satellites: %d",gps.satellites.value());
                            oledPut(7, oled_string); 
                            fsm_flag = millis();
                        }
                        break;
                }
            }
        }
    }

    else if(current_state == SEND_WITHOUT_GPS){
        // send every 3 seconds
        if(millis()-fsm_flag>3*SECOND){
            sendLoraTX(0);
            delay(min_delay_txpow[SF]);
            fsm_pck_count++;
            sprintf(oled_string, "Packets: %d", fsm_pck_count);
            oledPut(4, oled_string);
            fsm_flag = millis();
        }
    }
}

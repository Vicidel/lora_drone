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
enum {NO_GPS, NO_FIX, CHECK_PRECISION, GPS_IMPROVE, MAPPING, EVAL, ERR};
#define NB_STATES 7
#define STATE_STR_LEN 20
char states[NB_STATES][STATE_STR_LEN] = {"NO_GPS          ", "NO_FIX          ", "CHECK_PRECISION ", 
                                         "GPS_IMPROVE     ", "MAPPING         ", "EVAL            ", "ERROR           "};
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

enum {TRK_SUSPEND,TRK_TEST, TRK_WEATHER, TRK_TRILAT3, TRK_TRILAT4, TRK_TRILAT5, TRK_TRILAT6, TRK_TRILAT7, 
      TRK_TRILAT8, TRK_TRILAT9, TRK_TRILAT10, TRK_TRILAT11, TRK_MAP_EPFL = 20, TRK_MAP_LAUSANNE, TRK_DISTANCE = 30, 
      TRK_DEVCOMPARE = 40, TRK_EVAL_EPFL = 50, TRK_EVAL_LAUSANNE, DISCARD = 99};


//Defines where on the server the points go, and 0 suspends the mapping.
//0 suspend mapping
//1 for mapping & data collection test
//2 for static hum&temp measures
//3-12 for static trilateration
//20 for data collection mapping EPFL
//21 for data collection mapping Lausanne
//30 for distance vs rssi measure
//40 for comparing multiple devices
//50 for model validation EPFL
//51 for model validation Lausanne
//99 test to be discarded
int track_number = TRK_MAP_EPFL;

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

  oledPutState();

}

//put current state to OLED
void oledPutState(){
  SeeedOled.setTextXY(0, 0);
  SeeedOled.putString("                ");
  SeeedOled.putString(states[fsm_state]);
  oledPutInfo();
}

void oledPutInfo(){
  SeeedOled.setTextXY(5, 0);
  SeeedOled.putString("Micha Burger    ");
  SeeedOled.setTextXY(6, 0);
  sprintf(oled_string, "v1 03/2018   t%d",track_number);
  SeeedOled.putString(oled_string);
}

//Delete line and put new information
void oledPut(int line, char *str){
  SeeedOled.setTextXY(line,0);
  //clean line
  SeeedOled.putString("                ");
  delay(30);
  SeeedOled.setTextXY(line,0);
  SeeedOled.putString(str);
  delay(30);
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
  tx_buf[18] = track_number & 0x00ff;
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


void loop() {
    //IMPROVE: Get rid of delay() statements in state machine!!!
    //--> Do with timer & millis()

    /*
    int rand_txpow = random(0,5);
    //for testing SF: only when track number is 99!!
    track_number = DISCARD;
    sendLoraTX(rand_txpow);
    delay(2000);

*/

    //change suspend mode or track number
    if(digitalRead(buttonPin)){
      switch (track_number)
      {
        case TRK_MAP_EPFL:
          track_number = TRK_EVAL_EPFL;
          SeeedOled.clearDisplay();
          state_change_reset();
          oledPut(1,"EVAL EPFL");
          oledPutInfo();
          delay(SECOND);
          break;
        case TRK_EVAL_EPFL:
          track_number = TRK_SUSPEND;
          SeeedOled.clearDisplay();
          state_change_reset();
          oledPut(1,"SUSPEND");
          oledPutInfo();
          delay(SECOND);
          break;
        case TRK_SUSPEND:
          track_number = TRK_MAP_LAUSANNE;
          SeeedOled.clearDisplay();
          state_change_reset();
          oledPut(1,"MAP LAUSANNE");
          oledPutInfo();
          delay(SECOND);
          break;
        case TRK_MAP_LAUSANNE:
          track_number = TRK_EVAL_LAUSANNE;
          SeeedOled.clearDisplay();
          state_change_reset();
          oledPut(1,"EVAL LAUSANNE");
          oledPutInfo();
          delay(SECOND);
          break;
        case TRK_EVAL_LAUSANNE:
          track_number = TRK_MAP_EPFL;
          SeeedOled.clearDisplay();
          state_change_reset();
          oledPut(1,"MAP EPFL");
          oledPutInfo();
          delay(SECOND);
          break;

          /*
        case TRK_TRILAT3:
          track_number = TRK_TRILAT4;
          SeeedOled.clearDisplay();
          state_change_reset();
          oledPut(1,"STONE CIRCLE");
          oledPutInfo();
          delay(SECOND);
          break;
        case TRK_TRILAT4:
          track_number = TRK_TRILAT5;
          SeeedOled.clearDisplay();
          state_change_reset();
          oledPut(1,"ROLEX SOUTH");
          oledPutInfo();
          delay(SECOND);
          break;
        case TRK_TRILAT5:
          track_number = TRK_TRILAT6;
          SeeedOled.clearDisplay();
          state_change_reset();
          oledPut(1,"INNOVATION PARK");
          oledPutInfo();
          delay(SECOND);
          break;
        case TRK_TRILAT6:
          track_number = TRK_TRILAT7;
          SeeedOled.clearDisplay();
          state_change_reset();
          oledPut(1,"TIR FEDERAL");
          oledPutInfo();
          delay(SECOND);
          break;
        case TRK_TRILAT7:
          track_number = TRK_TRILAT8;
          SeeedOled.clearDisplay();
          state_change_reset();
          oledPut(1,"ARGAND");
          oledPutInfo();
          delay(SECOND);
          break;
        case TRK_TRILAT8:
          track_number = TRK_TRILAT9;
          SeeedOled.clearDisplay();
          state_change_reset();
          oledPut(1,"COSANDEY");
          oledPutInfo();
          delay(SECOND);
          break;
        case TRK_TRILAT9:
          track_number = TRK_TRILAT10;
          SeeedOled.clearDisplay();
          state_change_reset();
          oledPut(1,"INM");
          oledPutInfo();
          delay(SECOND);
          break;
        case TRK_TRILAT10:
          track_number = TRK_TRILAT11;
          SeeedOled.clearDisplay();
          state_change_reset();
          oledPut(1,"POLYDOME");
          oledPutInfo();
          delay(SECOND);
          break;
        case TRK_TRILAT11:
          track_number = TRK_MAPPING;
          SeeedOled.clearDisplay();
          state_change_reset();
          oledPutInfo();
          delay(SECOND);
          break;   

          */
      }
    }


    if(track_number == TRK_MAP_EPFL || track_number == TRK_EVAL_EPFL || track_number == TRK_MAP_LAUSANNE || track_number == TRK_EVAL_LAUSANNE) {
      //check if GPS is still connected
      while (Serial.available() > 0 && !digitalRead(buttonPin)) 
        if (gps.encode(Serial.read())){
          
        //check GPS fix
        if(fsm_state!=NO_GPS && (!gps.location.isValid() || !gps.time.isValid() || gps.satellites.value()<1)){
          fsm_state = NO_FIX;
          oledPutState();
        }
          
        
        switch (fsm_state)
        {
          case NO_GPS: 
            if (gps.charsProcessed() > 10){
              fsm_state = NO_FIX;
              oledPutState();
            }
            delay(t_check_fix);
            break;
      
          case NO_FIX:
            if(gps.location.isValid() && gps.time.isValid() && gps.satellites.value()>0){
              fsm_state = CHECK_PRECISION;
              oledPutState();
            }
         
            delay(t_check_fix);
            break;
      
          case CHECK_PRECISION:
            if(gps.hdop.value()<MIN_HDOP && gps.hdop.value()>0){
              //store time when the fix was accepted
              fsm_flag = millis();
              fsm_state = GPS_IMPROVE;
              oledPutState();
            }
            sprintf(oled_string, "HDOP: %d", gps.hdop.value());
            oledPut(2,oled_string);
            sprintf(oled_string, "Satellites: %d",gps.satellites.value());
            oledPut(3,oled_string);
            delay(t_check_fix);
            break;
      
          case GPS_IMPROVE:
            if(millis()-fsm_flag > t_precision) {
              fsm_flag = millis();
              eval_wait_flag = millis();
              if(track_number ==TRK_MAP_EPFL || track_number == TRK_MAP_LAUSANNE) {
                fsm_state = MAPPING;
                oledPutState();
              }
              else if(track_number == TRK_EVAL_EPFL || track_number == TRK_EVAL_LAUSANNE){
                fsm_state = EVAL;
                oledPutState();
              }
              oledPut(2,"Done, start mapping!");
            }
            else{
              sprintf(oled_string, "Wait: %d", (int)(0.001 * (t_precision - (millis()-fsm_flag))));
              oledPut(2,oled_string);
            }
            //continuously check fix and precision
            if(gps.hdop.value()>MIN_HDOP){
              //store time when the fix was accepted
              fsm_state = CHECK_PRECISION;
              oledPutState();
              oledPut(2,"GPS precision lost");
            }
            delay(t_improve);
            break;
      
          case MAPPING:
            //check if in correct track
            if(track_number == TRK_EVAL_EPFL || track_number == TRK_EVAL_LAUSANNE){
              eval_wait_flag = millis();
              fsm_state = EVAL;
              oledPutState();
              break;
            }

            sprintf(oled_string, "Since: %d s", (int) (0.001 * (millis()-fsm_flag)));
            oledPut(2,oled_string);
            sprintf(oled_string, "HDOP: %d", gps.hdop.value());
            oledPut(3,oled_string);
            sprintf(oled_string, "Satellites: %d",gps.satellites.value());
            oledPut(4,oled_string); 
            
            if(gps.location.isUpdated()){
              //only send acceptable HDOP
              if(gps.hdop.value() > 0 && gps.hdop.value() < HDOP_SEND) {
                  sendLoraTX(0);
                  delay(t_lora_tx);
              }
            }
            else {
              oledPut(3,"No update, wait.");
              delay(t_update);
            }
            break;

            case EVAL:
              //check if in correct track
              if(track_number ==TRK_MAP_EPFL || track_number == TRK_MAP_LAUSANNE){
                fsm_state = MAPPING;
                oledPutState();
                break;
              }
             sprintf(oled_string, "Packet: %d", pck_counter);
             oledPut(2,oled_string);
             if(pck_counter == 0){
              oledPut(3,"WAIT / MOVE");
             }
              //wait for the time between two transmissions 
              if(millis() - eval_wait_flag > t_lora_val){
                sendLoraTX(0);
                delay(t_eval);
                pck_counter++;
                //send N+1 transmissions in a row because some of them sometimes fail
                if(pck_counter > (NB_TRANSMISSIONS)){
                  eval_wait_flag = millis();
                  pck_counter = 0;
                }
              }            
              break;
        }  
      }
    }

    else if(track_number == TRK_WEATHER){
      //Send every 5 minutes
      if(millis()-fsm_flag>five_minutes){
        sendLoraTX(0);
        delay(min_delay_txpow[SF]);
        fsm_pck_count++;
        sprintf(oled_string, "Packets: %d",fsm_pck_count);
        oledPut(2,oled_string);
        fsm_flag = millis();
      }
    }

    else if (track_number != TRK_SUSPEND){
      //only start sending after 10 seconds to avoid wrong attribution
      if(millis()-state_entry>10*SECOND)
        if(millis()-fsm_flag>t_trilat){
          sendLoraTX(0);
          delay(min_delay_txpow[SF]);
          fsm_pck_count++;
          sprintf(oled_string, "Packets: %d",fsm_pck_count);
          oledPut(2,oled_string);
          fsm_flag = millis();
        }
          
    }
}

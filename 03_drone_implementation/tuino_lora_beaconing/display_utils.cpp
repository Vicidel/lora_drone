/*
 * 
 */

#include "SeeedOLED.h"

 void centerString(char *string )
{
  String tmpString;
  int len,spaces,i;
  
  len = strlen(string);
Serial.println(string);
Serial.println(len);

  spaces = 0;
  if ( len < 16 ) 
    spaces = ( 16 - len ) / 2;

Serial.println(spaces);

  for (i=0;i<spaces;i++)
    tmpString = tmpString + " ";
    
 tmpString = tmpString + string;
 tmpString.toCharArray(string, 16);
 
}

void splashScreen() {

  char string[16];
  
  SeeedOled.clearDisplay();   

  SeeedOled.setTextXY(0,0);        
  SeeedOled.putString("*** TUINO1 ***"); 
  SeeedOled.setTextXY(2,0);  
  sprintf(string,"LPN Demo");
  centerString(string);
  SeeedOled.putString(string); 

  /*
  SeeedOled.setTextXY(4,0);  
  sprintf(string,"PRESS");
  centerString(string);
  SeeedOled.putString(string);
  
  SeeedOled.setTextXY(5,0);  
  sprintf(string,"BUTTON");
  centerString(string);
  SeeedOled.putString(string);
  
  SeeedOled.setTextXY(6,0); 
  sprintf(string,"TO CONTINUE");
  centerString(string);
  SeeedOled.putString(string);
  */
 }


 void displayTemp(float temp, String text ) {

  char string[256];
  
  SeeedOled.setTextXY(0,0);        
  SeeedOled.putString("Temp: ");
  SeeedOled.putFloat(temp);


  SeeedOled.setTextXY(2,0);        
  text.toCharArray(string, 256);
  SeeedOled.putString(string);

  /*
  SeeedOled.setTextXY(1,0);  
  SeeedOled.putString("Setting: "); 
  SeeedOled.putFloat(thermostat_temperature);  
  SeeedOled.setTextXY(2,0);  
  SeeedOled.putString("Mode: ");
  if ( manual_mode == 1 )
    SeeedOled.putString("MANUAL   ");
  else
    SeeedOled.putString("AUTOMATIC");
  
  SeeedOled.setTextXY(3,0);  
  SeeedOled.putString("Relay: ");
  if ( relay_status == 1 )
    SeeedOled.putString(" ON");
  else
    SeeedOled.putString("OFF");
    */
  
 }


 void displayLoraWanParams(String DevEUI, String AppEUI, String AppKey ) 
 {
  char string[64];

  SeeedOled.clearDisplay();
  SeeedOled.setTextXY(0,0);        
  SeeedOled.putString(" LoRaWAN PARAMS"); 
  SeeedOled.setTextXY(1,0);        
  SeeedOled.putString("DevEUI");
  DevEUI.replace(":","");
  DevEUI.toCharArray(string, 64);
  SeeedOled.setTextXY(2,0);        
  SeeedOled.putString(string);

  SeeedOled.setTextXY(3,0);        
  SeeedOled.putString("AppEUI"); 
  AppEUI.replace(":","");
  AppEUI.toCharArray(string, 64);
  SeeedOled.setTextXY(4,0);        
  SeeedOled.putString(string);
  
  SeeedOled.setTextXY(5,0);        
  SeeedOled.putString("AppKey"); 
  AppKey.replace(":","");
  AppKey.toCharArray(string, 64);
  SeeedOled.setTextXY(6,0);        
  SeeedOled.putString(string);
     

 }

 void displayTime2TX(long int time_ms) {

  char buf[32];
  int time_sec;

  time_sec = time_ms / 1000;

  sprintf(buf,"Time to TX: %3d",time_sec);
  
  SeeedOled.setTextXY(7,0);    
  SeeedOled.putString(buf); 
  
 }
 void displayLoraTX(bool status) {
  SeeedOled.setTextXY(7,0);    
  if ( status )    
    SeeedOled.putString("  ***LORA TX*** "); 
  else
    SeeedOled.putString("                "); 
 }


 void displayLoraRX(bool status) {
  SeeedOled.setTextXY(7,0);    
  if ( status )    
    SeeedOled.putString("  ***LORA RX*** "); 
  else
    SeeedOled.putString("                "); 
 }
 
 void displayLora( ) {

  SeeedOled.clearDisplay();   

 }


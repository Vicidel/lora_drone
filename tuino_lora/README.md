# TUINO LORA
This is an example sketch that shows the use of the GMX-LR1 library to join, trasmit and receive data via LoRa.</bR>
You can use it as a blueprint for your LoRa projects and remember to install or copy in the same folder the GMX-LR libraries.<br/>
<br/>
Every GMX-LR1 module come with their unique DevEUI that cannot be changed (we can provide a custom firmware if you need to change the DevEUI).<br/> AppEUI and AppKey, for OTA provisioning, come preprogrammed, but can be changed with the respective gmxLR_ function calls. The preprogrammed values follow this pattern:<br/>
* AppEUI = DevEUI
* AppKey = two times the DevEUI
so for example:<br/>
DevEUI = 78AF580301020304<br/>
AppEUI = 78AF580301020304<br/>
AppKEY = 78AF58030102030478AF580301020304
<br/>
When you change the AppKey and AppEUI they will stored in the EEPROM of the GMX-LR module and will remain set also after power cycle.<br/>
Here a quick explanation of the code:<br/>

## Init Section

```c
// Declare the interrupt callback function for RX

bool data_received=false;

void loraRx(){
  data_received = true;
}
...
...
...

// GMX-LR init pass callback function
  gmxLR_init(&loraRx);

...
...

// We now Join the NetWork
// while loop checking the Join State
while((join_status = gmxLR_isNetworkJoined()) != LORA_NETWORK_JOINED) {

  
    if ( join_wait == 0 )
    {
     // If AppKey and/or AppEUI are specified set them
     if (AppEui.length() > 0 )
      gmxLR_setAppEui(AppEui);
     if (AppKey.length() > 0 )
      gmxLR_setAppKey(AppKey);

      // Disable Duty Cycle  ONLY FOR DEBUG!
      gmxLR_setDutyCycle("0");
      gmxLR_setClass("C");

	  ...
	  ...	
	  ...

	  // Here we trigger the Join Request
      gmxLR_Join();
    }

    
    join_wait++;

    // Not necessary - but we reset everything after a timeout
    if (!( join_wait % 100 )) {
      gmxLR_Reset();
      join_wait = 0;
    }

    delay(5000);

  };

```

## Main Loop

TX Data

```c

	...
	...
	// check delta TX Timeout
	delta_lora_tx = millis() - timer_millis_lora_tx;
   
   // Transmit Period 
   if ( delta_lora_tx > timer_period_to_tx) {
  
    Serial.println("TX DATA");

  	// Transmit Data - as HEX String
    gmxLR_TXData("010203");

    ...
    ...
  
     timer_millis_lora_tx = millis();
   }
```

RX Data. RX is interrupt driven, when a RX packet arrives the loraRx() callback function is called and the data_received flag is set. We  will simply test the flag status to read the RX buffer from the GMX-LR module.

```c
if (data_received)
   {
   	  // retrive RX data from GMX-LR module. 
   	  // rx_data is a HEX String
   	  // port is also returned
      gmxLR_RXData(rx_data,&port);
 
      // convert HEX String to byte array for simpler packet decoding
      gmxLR_StringToHex(rx_data, rx_buf, rx_buf_len );

      ...
      ...

      // simple protocol to turn on and off on board LED

      if ( rx_buf[0] == 0x01 )
        digitalWrite(LED_BUILTIN,HIGH);
      else
        digitalWrite(LED_BUILTIN,LOW);
        
      
      // clear flag
      data_received = false;
   }
```

The GMX-LR module uses the onboard leds to show it's state and activity, read [here](../) for a complete description
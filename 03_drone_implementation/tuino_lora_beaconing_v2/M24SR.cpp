/* Arduino library for ST M24SR Dynamic NFC/RFID tag IC with EEPROM, NFC Forum Type 4 Tag and I2C interface
   (c) 2014 by ReNa http://regnerischernachmittag.wordpress.com/ 
*/
/*
 * some modification by Gimasi SA for Tuino board http:/www.gimasi.ch
 */

#include "M24SR.h"
#include <Wire.h>

const char AID_NDEF_TAG_APPLICATION2[] PROGMEM = "\xD2\x76\x00\x00\x85\x01\x01";
const char FILE_CC[] PROGMEM = "\xE1\x03";
const char FILE_SYSTEM[]  PROGMEM = "\xE1\x01";
const char DEFAULT_PASSWORD[] PROGMEM  = "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00";

//prog_char SELECT_NDEF_Tag_Application2[] PROGMEM = "\x00\xA4\x04\x00\x07\xD2\x76\x00\x00\x85\x01\x01";
//prog_char SELECT_CC[] PROGMEM = "\x00\xA4\x00\x0C\x02\xE1\x03";                 
//prog_char SELECT_NDEF_file[] PROGMEM = "\x00\xA4\x00\x0C\x02\x00\x01";
//prog_char SELECT_system_file[] PROGMEM = "\x00\xA4\x00\x0C\x02\xE1\x01";
//prog_char UPDATE_BINARY_NDEF_MSG_LEN0[] PROGMEM = "\x00\xD6\x00\x00\x02\x00\x00";
//prog_char READ_BINARY_LENGTH[] PROGMEM = "\x02\x00\xB0\x00\x00\x02";
//prog_char READ_BINARY[] PROGMEM = "\x02\x00\xB0\x00\x00";
//prog_char READ_BINARY_DEF_MSG[] PROGMEM = "\x02\x00\xB0\x00\x02";

// 0x45
const char SAMPLE_NDEF_message1[]  PROGMEM = "\x91\x01\x10\x55\x01\x73\x74\x2E\x63\x6F\x6D\x2F\x6E\x66\x63\x2D\x72\x66\x69\x64\x54\x18\x16\x73\x74\x2E\x63\x6F\x6D\x3A\x6D\x32\x34\x73\x72\x5F\x70\x72\x6F\x70\x72\x69\x65\x74\x61\x72\x73\x4D\x32\x34\x53\x52\x20\x70\x72\x6F\x70\x72\x69\x65\x74\x61\x72\x79\x20\x64\x61\x74\x61";
//char SAMPLE_NDEF_message2[] PROGMEM = "\xd1\x01\x0a\x55\x03\x6e\x6f\x6b\x69\x61\x2e\x63\x6f\x6d";

// 0x0F www.st.com
//char SAMPLE_NDEF_message2[] PROGMEM = "\xD1\x01\x0b\x55\x01\x77\x77\x77\x2E\x73\x74\x2E\x63\x6F\x6D";

// 0x0c www.nfc.com 
//prog_char SAMPLE_NDEF_message2[] PROGMEM = "\xD1\x01\x08\x55\x01\x6E\x66\x63\x2E\x63\x6F\x6D";

// 0x11 phone: +358 9 1234567
const char SAMPLE_NDEF_message2[] PROGMEM = "\xD1\x01\x0D\x55\x05\x2B\x33\x35\x38\x39\x31\x32\x33\x34\x35\x36\x37";

// 0x13 txt: "hello, world"
const char SAMPLE_NDEF_message3[] PROGMEM = "\xD1\x01\x0F\x54\x02\x65\x6e\x68\x65\x6c\x6c\x6f\x2c\x20\x77\x6f\x72\x6c\x64";

// EMPTY Tag
// 00 03    D0 00 00

M24SR::M24SR(void) {
    _verbose = false;
    _cmds = false;
}

M24SR::M24SR(uint8_t gpo) {
    _verbose = false;
    _cmds = false;
    _gpo_pin = gpo;
}

void M24SR::_setup() {
    if (_verbose)
      Serial.println(F("_setup"));
    _lastGPO = 1;
    _sendGetI2cSession = true;
    _deviceaddress = 0x56;
    _blockNo = 0;
    _responseLength = 0x15;
    _response = (byte*)malloc(_responseLength);
    
    Wire.begin(); // join i2c bus (address optional for master)
    if (_gpo_pin != 0) {
       pinMode(_gpo_pin, INPUT);
       writeGPO(0x30);
    }
}

M24SR::~M24SR() {
    if (_responseLength)
    {
        free(_response);
    }
}

void M24SR::writeGPO(uint8_t value) {
   if (_verbose)
      Serial.println(F("\r\nwriteGPO"));
   if (!verifyI2cPassword()) {
      Serial.println(F("\r\nwrong password!!!"));
      return;
   }
   sendApdu_P(0x00, INS_SELECT_FILE, 0x00, 0x0C, 0x02, FILE_SYSTEM);
   receiveResponse(2 + 3);
   
   //write system file at offset 0x0004 GPO
   sendApdu(0x00, INS_UPDATE_BINARY, 0x00, 0x04, 0x01, &value);
   receiveResponse(2 + 3);
   sendDESELECT();
}

void M24SR::selectFile_NDEF_App() {
   if (_verbose)
      Serial.println(F("\r\nselectFile_NDEF_App"));
   _sendGetI2cSession = true;
   sendApdu_P(0x00, INS_SELECT_FILE, 0x04, 0x00, 0x07, AID_NDEF_TAG_APPLICATION2);
   receiveResponse(2 + 3);
}

boolean M24SR::verifyI2cPassword() {
  if (_verbose)
      Serial.println(F("\r\nverifyI2cPassword"));
  selectFile_NDEF_App();
  sendApdu_P(0x00, INS_VERIFY, 0x00, 0x03, 0x10, DEFAULT_PASSWORD);
  receiveResponse(2 + 3);
  return ((_response[0] == 0x90) && (_response[1] == 0));
}

boolean M24SR::checkGPOTrigger()
{
  uint8_t newval = digitalRead(_gpo_pin);
  if (newval == 1 && _lastGPO == 0) {
     _lastGPO = newval;
     return true;
  }
  _lastGPO = newval;
  return false;
}

void M24SR::updateBinaryLen(int len) {
    if (_verbose)
        Serial.println(F("\r\nupdateBinaryLen"));
    uint8_t len_bytes[] = "\x00\x00";
    len_bytes[0] = (len >> 8) & 0xff; 
    len_bytes[1] = (len & 0xff);
    sendApdu(0x00, INS_UPDATE_BINARY, 0x00, 0x00, 0x02, len_bytes);
}

void M24SR::updateBinary(char* Data, uint8_t len) {
    if (_verbose)
        Serial.println(F("\r\nupdateBinary"));
    dumpHex((uint8_t*)Data, len);
    uint8_t pos = 0;
    while(pos < len) {
       uint8_t chunk_len = (len - pos);
       if (chunk_len > (BUFFER_LENGTH - 8))
          chunk_len = BUFFER_LENGTH - 8;
       Serial.print(F("\r\nchunk_len:"));
       Serial.print(chunk_len, DEC);
       Serial.print(F(", pos:"));
       Serial.print(pos, DEC);
       sendApdu(0x00, INS_UPDATE_BINARY, ((pos+2) >> 8) & 0xff, (pos+2) & 0xff, chunk_len, (uint8_t*)&Data[pos]);
       pos += chunk_len;
       if (pos < len)
          receiveResponse(2 + 3);
    }
}

void M24SR::updateBinary(unsigned int offset, char* data, uint8_t len) {
    if (_verbose)
        Serial.println(F("\r\nupdateBinary offset"));
    sendApdu(0x00, INS_UPDATE_BINARY, (offset >> 8) & 0xff, (offset & 0xff), len, (uint8_t*)data);
}

void M24SR::displaySystemFile() {
   selectFile_NDEF_App();
   sendApdu_P(0x00, INS_SELECT_FILE, 0x00, 0x0C, 0x02, FILE_SYSTEM);
   receiveResponse(2 + 3);
   
   sendApdu(0x00, INS_READ_BINARY, 0x00, 0x00, 0x02);
   receiveResponse(2 + 2 + 3);
  
   if (_verbose) {
     Serial.print(F("\r\nresponse[1]: "));
     Serial.print(_response[1] & 0xff, HEX);
   }

   sendApdu(0x00, INS_READ_BINARY, 0x00, 0x00, _response[1]);
   receiveResponse((_response[1] & 0xff) + 2 + 3);
   
   //display settings
   Serial.print(F("\r\nUID: "));
   dumpHex(&_response[8], 7);
   Serial.print(F("\r\nMemory Size: 0x"));
   if ((_response[0xf] & 0xff) < 0x10)
       Serial.print("0");
   Serial.print((_response[0xf] & 0xff), HEX);
   if ((_response[0x10] & 0xff) < 0x10)
       Serial.print("0");
   Serial.print((_response[0x10] & 0xff), HEX);
   Serial.print(F("\r\nProduct Code: 0x"));
   if ((_response[0x11] & 0xff) < 0x10)
       Serial.print("0");
   Serial.print((_response[0x11] & 0xff), HEX);
   
   sendDESELECT();
}

/*
end of a I2c Session:
5.4 S-Block format
 0xC2: for S(DES) when the DID field is not present
*/
void M24SR::sendDESELECT() {
  if (_verbose)
        Serial.print(F("\r\nsend DESELECT"));
  sendSBLOCK(0xC2);//PCB field
}

void M24SR::sendSBLOCK(byte sblock) {
  _data[0] = sblock;
  sendCommand(/*_data,*/ 1, false);
  receiveResponse(0 + 3) ;
}

void M24SR::sendApdu(uint8_t CLA, uint8_t INS, uint8_t P1, uint8_t P2, uint8_t Lc, uint8_t* Data) {
  _data[1] = CLA;
  _data[2] = INS;
  _data[3] = P1;
  _data[4] = P2;
  _data[5] = Lc;
  memcpy(&_data[6], Data, Lc);
  sendCommand(/*_data,*/ 1+5+Lc, true);
}

void M24SR::sendApdu_P(uint8_t CLA, uint8_t INS, uint8_t P1, uint8_t P2, uint8_t Lc, const char* Data) {
  _data[1] = CLA;
  _data[2] = INS;
  _data[3] = P1;
  _data[4] = P2;
  _data[5] = Lc;
  memcpy_P(&_data[6], Data, Lc);
  sendCommand(/*_data,*/ 1+5+Lc, true);
}

//APDU case 2
void M24SR::sendApdu(uint8_t CLA, uint8_t INS, uint8_t P1, uint8_t P2, uint8_t Le) {
  _data[1] = CLA;
  _data[2] = INS;
  _data[3] = P1;
  _data[4] = P2;
  _data[5] = Le;
  sendCommand(/*_data, */1+5, true);
}

void M24SR::sendCommand(/*char* data,*/ int len) {
  sendCommand(/*data,*/ len, true);
}

// len (incl PCB byte)
void M24SR::sendCommand(/*char* data, */int len, boolean setPCB) {
  uint8_t v;
  if (setPCB) {
     if (_blockNo == 0) {
        _data[0] = 0x02;
        _blockNo = 1;
     } else {
        _data[0] = 0x03;
        _blockNo = 0;
     }
  }
  if (_sendGetI2cSession) {
    Wire.beginTransmission(_deviceaddress); // transmit to device 0x2D
    Wire.write(byte(CMD_GETI2CSESSION)); // GetI2Csession
    _err = Wire.endTransmission();     // stop transmitting
    if (_verbose) {
      Serial.print(F("\r\nGetI2Csession: "));
      Serial.print(_err, HEX);
    }
    else
      delay(1);
  }
  
  if (_cmds)
      Serial.print(F("\r\n=> "));
  else
      delay(1);
  
  Wire.beginTransmission(_deviceaddress);
  
  for(int i=0; i < len; i++) {
    v = (_data[i] & 0xff);
    if (_cmds) {
       if (v < 0x10)
          Serial.print(F("0"));
       Serial.print(v, HEX);
    } else
      delay(5);
    Wire.write(byte(v & 0xff));
    if (_cmds)
       Serial.print(F(" "));
    else
       delay(1);
  }
  
  //5.5 CRC of the I2C and RF frame ISO/IEC 13239. The initial register content shall be 0x6363
  int chksum =  crcsum((unsigned char*) _data, len, 0x6363 );
 // int chksum =  crcsum((unsigned char*) _data, len);
  
  v = chksum & 0xff;
  if (_cmds) {
     if (v < 0x10) {
       Serial.print(F("0"));
     }
     Serial.print(v, HEX);
  } else
     delay(1);

  Wire.write(byte(v & 0xff));
  
  //EOD field
  v = (chksum >> 8) & 0xff;
  if (_cmds) {
     if (v < 0x10) {
        Serial.print(F("0"));
     }
     Serial.print(v, HEX);
  } else
     delay(1);
  Wire.write(byte(v & 0xff));
  
  _err = Wire.endTransmission();
  if (_cmds) {
      Serial.print(F("\r\n"));
  }
  else {
      delay(1);
  }
  //TODO does this really work?
  if (_err != 0) {
    Serial.print(F("write err: "));
    Serial.print(_err, HEX);
  }
}

void M24SR::selectFile_NDEF_file() {
  if (_verbose)
        Serial.print(F("\r\nselectFile_NDEF_file"));
   uint8_t ndef_file[] = "\x00\x01";
   sendApdu(0x00, INS_SELECT_FILE, 0x00, 0x0C, 0x02, ndef_file);
   receiveResponse(2 + 3);
}

void M24SR::updateBinary_NdefMsgLen0() {
  if (_verbose)
        Serial.print(F("\r\nupdateBinary_NdefMsgLen0"));
   uint8_t len0[] = "\x00\x00";
   sendApdu(0x00, INS_UPDATE_BINARY, 0x00, 0x00, 0x02, len0);
   receiveResponse(2 + 3);
}

void M24SR::writeSampleMsg(uint8_t msgNo) {
  if (_verbose) {
        Serial.print(F("\r\nwriteSampleMsg "));
        Serial.print(msgNo, DEC);
        Serial.println("");
   }
   byte len = 0;
   selectFile_NDEF_App();
   selectFile_NDEF_file();
   updateBinary_NdefMsgLen0();
   
   switch(msgNo) {
   case 0:
     len = 0x45;
     memcpy_P(&_data[0], &SAMPLE_NDEF_message1[0], len);
     break;
   case 1:
     len = 0x11;
     memcpy_P(&_data[0], &SAMPLE_NDEF_message2[0], len);
     break;
   case 2:
     len = 0x13;
     memcpy_P(&_data[0], &SAMPLE_NDEF_message3[0], len);
     break;
   default:
     break;
   }
   updateBinary(_data, len);
   receiveResponse(2 + 3);
   
   updateBinaryLen(len);
   receiveResponse(2 + 3);
   sendDESELECT();
}

NdefMessage* M24SR::getNdefMessage()
{
  unsigned int len = 0;
  
  selectFile_NDEF_App();
  selectFile_NDEF_file();
  //Read NDEF message length 00 B0 00 00 02
  uint16_t ndef_len = getNdefMessageLength();
  if (_verbose) {
     Serial.print(F("\r\nndef_len: "));
     Serial.println(ndef_len, DEC);
   }
  
   if (ndef_len < 255)
       sendApdu(0x00, INS_READ_BINARY, 0x00, 0x02, ndef_len & 0xff);
   else
   {
      Serial.println(F("TODO: ndef_len > 255"));
      return (NdefMessage*)NULL;
   }
   //uint8_t ndeflen[2];
   //ndeflen[0] = ndef_len >> 8;
   //ndeflen[1] = ndef_len & 0xff;
   //sendApdu(0x00, INS_READ_BINARY, 0x00, 0x00, ndeflen);
   receiveResponse((_response[1] & 0xff) + 2 + 3);
   NdefMessage* pNdefMsg = new NdefMessage((byte *)&_response[0], ndef_len);
   
   //delete pNdefMsg;
   sendDESELECT();
   return pNdefMsg;
}


int M24SR::receiveResponse(unsigned int len) {
  int index=0;
  boolean WTX = false;
  boolean loop = false;
  if (_verbose) {
    Serial.print(F("\r\nreceiveResponse, len="));
    Serial.print(len, DEC);
    Serial.println();
  }
  if (_responseLength < len) {
    free(_response);
    if (_verbose) {
        Serial.print(F("\r\n_responseLength="));
        Serial.print(len, DEC);
    }
    _responseLength = len;
    _response = (byte*)malloc(_responseLength);
  }
  else
     delay(1);
  do {
    WTX = false;
    loop = false;
    Wire.requestFrom(_deviceaddress, len);
    if (_cmds)
        Serial.print(F("<= "));
    else
        delay(1);
    while ((Wire.available() && index < len && !WTX) ||
           (WTX && index < len-1)) {
      int c  = (Wire.read() & 0xff);
      if (_cmds) {
          if (c < 0x10)
              Serial.print(F("0"));
          Serial.print(c, HEX);
          Serial.print(F(" "));
      }
      else
         delay(1);
      if (c == 0xF2 && index == 0) {
         WTX = true;
      }
      if (index >= 1) {
          _response[index-1] = c;
      }
      index++;
    }
    if (WTX) {
       Serial.print(F("\r\nWTX"));
       delay(200 * _response[0]);
       //send WTX response
       //sendSBLOCK(0xF2);
       _data[0] = 0xF2;//WTX
       _data[1] = _response[0];
       sendCommand(/*_data,*/ 2, false);
       loop = true;
       index = 0;
    }
  }while(loop);
  return index;
}

void M24SR::writeNdefMessage(NdefMessage* pNDefMsg)
{
    if (pNDefMsg != NULL) {
      // pNDefMsg->print();
       NdefRecord rec = pNDefMsg->getRecord(0);
       //Serial.print(F("NDefRecord: "));
       //Serial.println(rec.toString());

       selectFile_NDEF_App();
       selectFile_NDEF_file();
       updateBinary_NdefMsgLen0();
       uint8_t len = pNDefMsg->getEncodedSize();
       uint8_t* mem = (uint8_t*)malloc(len);
       //TODO memcpy(&_data[0], &SAMPLE_NDEF_message1[0], len);
       //uint8_t encoded[pNDefMsg->getEncodedSize()];
       pNDefMsg->encode((uint8_t*)mem);

       updateBinary((char*)mem, len);
       receiveResponse(2 + 3);
       free(mem);
   
       updateBinaryLen(len);
       receiveResponse(2 + 3);
       sendDESELECT();
    }
}

unsigned int M24SR::getNdefMessageLength() 
{
  sendApdu(0x00, INS_READ_BINARY, 0x00, 0x00, 0x02);
  receiveResponse(2 + 2 + 3);
  return ((_response[0] & 0xff) << 8) | (_response[1] & 0xff);
}

void M24SR::print()
{
    Serial.print(F("\nM24SR GPO:"));
    Serial.println(_gpo_pin);
}

char text[4];

void M24SR::dumpHex(uint8_t* buffer, uint8_t len)
{
  for(byte i=0; i < len; i++) {
     sprintf(text, "%02X \x00", (uint8_t)(*(buffer + i)));
     Serial.print(text);
     if ((i % 16) == 15) {
        Serial.println(""); 
     }
  }
}


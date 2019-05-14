/* Arduino library for ST M24SR Dynamic NFC/RFID tag IC with EEPROM, NFC Forum Type 4 Tag and I2C interface
   (c) 2014 by ReNa http://regnerischernachmittag.wordpress.com/ 
*/

/*
 * some modification by Gimasi SA for Tuino board http://www.gimasi.ch
 */
 
#ifndef M24SR_h
#define M24SR_h

#include <Arduino.h>
#include "crc_utilities.h"
#include "NdefMessage.h"

#define CMD_GETI2CSESSION 0x26
#define CMD_KILLRFSESSION 0x52

/*

TODO
----
- clean-up code and add TODOs
- test: > 1 NDef record in NDef message
- read/write data (without NDef classes)
- what to do with writeSampleMsg?
- ndef_len > 255
- password handling
- dynamic data buffer
- if (len > BUFFER_LENGTH - 8) update for-loop


INFO
----

NOTE: The Arduino Wire library only has a 32 character buffer, so that is the maximun we can send using Arduino. This buffer includes the two address bytes which limits our data payload to 30 bytes

AN4433 Storing data into the NDEF memory of M24SR http://www.st.com/web/en/resource/technical/document/application_note/DM00105043.pdf
Datasheet http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00097458.pdf

*/

#define INS_SELECT_FILE 0xA4
#define INS_UPDATE_BINARY 0xD6
#define INS_READ_BINARY 0xB0
#define INS_VERIFY 0x20

class M24SR
{
    public:
        M24SR();
        M24SR(uint8_t gpo);
        ~M24SR();
        void print();
        boolean checkGPOTrigger();
        void updateBinary_NdefMsgLen0();
        void writeGPO(uint8_t data);
        //getUID()
        void sendDESELECT();
        NdefMessage* getNdefMessage(); 
        unsigned int getNdefMessageLength();
        void writeNdefMessage(NdefMessage* message);
        void selectFile_NDEF_file();
        void sendSBLOCK(uint8_t sblock);
        void selectFile_NDEF_App();
        void updateBinary(char* data, uint8_t len);
        void updateBinary(unsigned int offset, char* data, uint8_t len);
        void updateBinaryLen(int len);
        boolean verifyI2cPassword();
        //TODO boolean verifyI2cPassword(uint8_t* pwd);
        //TODO boolean setI2cPassword(uint8_t* old_password, uint8_t* new_password);
        void checkCRC(char* data, int len);
        void sendApdu(uint8_t CLA, uint8_t INS, uint8_t P1, uint8_t P2, uint8_t Lc, uint8_t* Data);
        void sendApdu(uint8_t CLA, uint8_t INS, uint8_t P1, uint8_t P2, uint8_t Le);
        void sendApdu_P(uint8_t CLA, uint8_t INS, uint8_t P1, uint8_t P2, uint8_t Lc, const char* Data);
        void sendCommand(/*char* data,*/ int len);
        void sendCommand(/*char* data,*/ int len, boolean setPCB);
        void selfTest();
        
        void writeSampleMsg(uint8_t msgNo);
        void displaySystemFile();
        int receiveResponse(unsigned int len);
        void _setup();

        void dumpHex(uint8_t* buffer, uint8_t len);
        boolean _verbose;
        boolean _cmds;
        char _data[100]; //TODO dynamic buffer
    private:

	uint8_t _gpo_pin;
        uint8_t _lastGPO;
        uint8_t _deviceaddress;
        boolean _sendGetI2cSession;
        uint8_t _err;
        uint8_t _blockNo;
        
        uint8_t _responseLength;
        uint8_t* _response;
};

#endif

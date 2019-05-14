/*
 * CRC Utilities
 */

#ifndef _CRC_UTILITIES_h
#define _CRC_UTILITIES_h


static uint16_t updateCrc(uint8_t ch, uint16_t *lpwCrc)
{

	ch = (ch^(uint8_t)((*lpwCrc) & 0x00FF));

	ch = (ch^(ch<<4));

	*lpwCrc = (*lpwCrc >> 8)^((uint16_t)ch << 8)^((uint16_t)ch<<3)^((uint16_t)ch>>4);

	return(*lpwCrc);

}

static uint16_t crcsum(uint8_t *Data, uint8_t Length, uint16_t wCrc)
{
	uint8_t chBlock;
//	uint16_t wCrc;
//	wCrc = 0x6363;
	
	do {

		chBlock = *Data++;
		updateCrc(chBlock, &wCrc);

	} while (--Length);

	return wCrc ;
}


#endif


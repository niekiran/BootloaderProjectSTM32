
/* This file implements some of the utility functions like conversions, CRC calculations , etc
 * This file is common across win/linux/mac
 */

#include "main.h"

//Converts uint32t_t value to series of bytes.
//if "lowerfirst" is 1 , then LSB is returned first (not used)
uint8_t word_to_byte(uint32_t addr, uint8_t index, uint8_t lowerfirst)
{
      uint8_t value = (addr >> ( 8 * ( index -1)) & 0x000000FF );
      return value;
}


//This function computes the 4 byte CRC(CRC32) using polynomial method
//Please refer these links for more details
//https://community.st.com/thread/18626
//http://www.st.com/content/ccc/resource/technical/document/application_note/39/89/da/89/9e/d7/49/b1/DM00068118.pdf/files/DM00068118.pdf/jcr:content/translations/en.DM00068118.pdf
//http://www.hackersdelight.org/hdcodetxt/crc.c.txt
//http://www.zlib.net/crc_v3.txt
uint32_t get_crc(uint8_t *buff, uint32_t len)
{
    uint32_t i;

    uint32_t Crc = 0XFFFFFFFF;

    for(uint32_t n = 0 ; n < len ; n++ )
    {
        uint32_t data = buff[n];
        Crc = Crc ^ data;
        for(i=0; i<32; i++)
        {

        if (Crc & 0x80000000)
            Crc = (Crc << 1) ^ 0x04C11DB7; // Polynomial used in STM32
        else
            Crc = (Crc << 1);
        }

    }

  return(Crc);
}

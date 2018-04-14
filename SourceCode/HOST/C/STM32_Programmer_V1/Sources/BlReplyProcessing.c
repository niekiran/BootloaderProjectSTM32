

/* This file implements the logic to read and process the replies from the Bootloader .
 * This file is common across win/linux/mac
 */

#include "main.h"



//Reads and processes the reply sent from the MCU bootloader
int read_bootloader_reply(uint8_t command_code)
{
    uint8_t ack[2]={0}; //MCU sends ack + len field
    uint32_t len_to_follow=0;
    int ret = -2;

    //The MCU bootloader always sends ack/nack first . read that !!
    read_serial_port(ack,2);
    if(ack[0] == 0xA5)
    {
        //CRC of last command was good .. received ACK and "len to follow"
        len_to_follow=ack[1];
        printf("\n\n   CRC : SUCCESS Len : %d\n",len_to_follow);

        switch(0x50 | command_code)
        {
        case COMMAND_BL_GET_VER:
             process_COMMAND_BL_GET_VER(len_to_follow);
            break;
        case COMMAND_BL_GET_HELP:
            process_COMMAND_BL_GET_HELP(len_to_follow);
            break;
        case COMMAND_BL_GET_CID:
            process_COMMAND_BL_GET_CID(len_to_follow);
            break;
        case COMMAND_BL_GET_RDP_STATUS:
            process_COMMAND_BL_GET_RDP_STATUS(len_to_follow);
            break;
        case COMMAND_BL_GO_TO_ADDR:
            process_COMMAND_BL_GO_TO_ADDR(len_to_follow);
            break;
        case COMMAND_BL_FLASH_ERASE:
            process_COMMAND_BL_FLASH_ERASE(len_to_follow);
            break;
        case COMMAND_BL_MEM_WRITE:
            process_COMMAND_BL_MEM_WRITE(len_to_follow);
            break;
        case COMMAND_BL_READ_SECTOR_P_STATUS:
            process_COMMAND_BL_READ_SECTOR_STATUS(len_to_follow);
            break;
        case COMMAND_BL_EN_R_W_PROTECT:
            process_COMMAND_BL_EN_R_W_PROTECT(len_to_follow);
            break;
        case COMMAND_BL_DIS_R_W_PROTECT:
            process_COMMAND_BL_DIS_R_W_PROTECT(len_to_follow);
            break;
        case COMMAND_BL_MY_NEW_COMMAND:
            process_COMMAND_BL_MY_NEW_COMMAND(len_to_follow);
        default:
            printf("\n  Invalid command code\n");

        }

          ret = 0;
    }
    else if( ack[0] == 0x7F)
    {
        //CRC of last command was bad .. received NACK
        printf("\n   CRC: FAIL \n");
        ret= -1;
    }

    return ret;
}

void process_COMMAND_BL_MY_NEW_COMMAND(uint32_t len)
{


}

void process_COMMAND_BL_GET_VER(uint32_t len)
{
    uint8_t ver;
    read_serial_port(&ver,len);
    printf("\n   Bootloader Ver. : 0x%x\n",ver);
}


void process_COMMAND_BL_GET_HELP(uint32_t len)
{
    uint8_t reply[15];
    read_serial_port(reply,len);
    printf("\n   Supported Commands :");
    for(uint32_t i =0 ; i < len ; i++)
    printf("0x%x  ",reply[i]);
    printf("\n");
}


void process_COMMAND_BL_GET_CID(uint32_t len)
{
    uint8_t cid[2];
    uint16_t ci=0;
    read_serial_port(cid,len);
    ci = (uint16_t)(cid[1] << 8 )+ cid[0];
    printf("\n   Chip Id. : 0x%x\n",ci);
}



void process_COMMAND_BL_GET_RDP_STATUS(uint32_t len)
{
    uint8_t rdp=0;
    read_serial_port(&rdp,len);
    printf("\n   RDP Status : 0x%X\n",rdp);
}


void process_COMMAND_BL_GO_TO_ADDR(uint32_t len)
{
    uint8_t addr_status=0;
    read_serial_port(&addr_status,len);
    printf("\n   Address Status : 0x%x\n",addr_status);
}


void process_COMMAND_BL_FLASH_ERASE(uint32_t len)
{
    uint8_t erase_status=0;
    read_serial_port(&erase_status,len);
    if(erase_status == Flash_HAL_OK)
    {
        printf("\n  Erase Status: Success  Code: Flash_HAL_OK\n");
    }
    else if(erase_status == Flash_HAL_ERROR)
    {
        printf("\n  Erase Status: Fail  Code: Flash_HAL_ERROR\n");

    }
    else if(erase_status == Flash_HAL_BUSY)
    {
        printf("\n  Erase Status: Fail  Code: Flash_HAL_BUSY\n");
    }
    else if(erase_status == Flash_HAL_TIMEOUT)
    {
        printf("\n  Erase Status: Fail  Code: Flash_HAL_TIMEOUT\n");
    }
     else if(erase_status == Flash_HAL_INV_ADDR)
    {
        printf("\n  Erase Status: Fail  Code: Flash_HAL_INV_SECTOR\n");
    }
    else
    {
        printf("\n  Erase Status: Fail  Code: UNKNOWN_ERROR_CODE\n");
    }
   // printf("     Erase Status : 0x%x\n",erase_status);
}

void process_COMMAND_BL_FLASH_MASS_ERASE(uint32_t len_to_follow)
{


}


void process_COMMAND_BL_MEM_WRITE(uint32_t len)
{
    uint8_t write_status=0;
    read_serial_port(&write_status,len);
    if(write_status == Flash_HAL_OK)
    {
        printf("\n   Write_status: Flash_HAL_OK\n");
    }
    else if(write_status == Flash_HAL_ERROR)
    {
        printf("\n   Write_status: Flash_HAL_ERROR\n");
    }
    else if(write_status == Flash_HAL_BUSY)
    {
        printf("\n   Write_status: Flash_HAL_BUSY\n");
    }
    else if(write_status == Flash_HAL_TIMEOUT)
    {
        printf("\n   Write_status: Flash_HAL_TIMEOUT\n");
    }
     else if(write_status == Flash_HAL_INV_ADDR)
    {
        printf("\n   Write_status: Flash_HAL_INV_ADDR\n");
    }
    else
    {
        printf("\n   Write_status: UNKNOWN_ERROR\n");
    }
   // printf("     Erase Status : 0x%x\n",erase_status);
}



char *mode[3]= { "Write Protection", "Read/Write Protection","No protection"};
char *protection_type(t_sector_status *pStatus, uint32_t n)
{

    if(pStatus->flash_sector_status & (1 << 15))
    { //PCROP is active
       // printf("\n  Flash protection mode : Read/Write Protection(PCROP)");

        if(pStatus->flash_sector_status & (1 << n))
        {
             return mode[1];
        }else
        {
            return mode[2];
        }

    }else
    {
       // printf("\n  Flash protection mode : Write Protection\n");
        if(pStatus->flash_sector_status & (1 << n))
        {
             return mode[2];
        }else
        {
            return mode[0];
        }
    }

}

void process_COMMAND_BL_READ_SECTOR_STATUS(uint32_t len)
{
    uint32_t i;
  //  uint16_t sectors_status=0;
    t_sector_status s_status;

    read_serial_port((uint8_t*)&s_status.flash_sector_status,len);
   // s_status.flash_sector_status = (uint16_t)(status[1] << 8 | status[0] );
    printf("\n   Sector Status : 0x%.4X\n",s_status.flash_sector_status);
    printf("\n  ====================================");
    printf("\n  Sector                  Protection");
    printf("\n  ====================================");
    if(s_status.flash_sector_status & (1 << 15))
    { //PCROP is active
        printf("\n  Flash protection mode : Read/Write Protection(PCROP)\n");
    }
    else
    {
        printf("\n  Flash protection mode : Write Protection\n");
    }

    for(i=0;i<8;i++)
    printf("\n  Sector%d                 %s",i,protection_type(&s_status,i));

    printf("\n");



}


void process_COMMAND_BL_EN_R_W_PROTECT(uint8_t len)
{
     uint8_t status=0;
     read_serial_port(&status,len);
     if(status)
     {
        printf("\n   FAIL\n");
     }else
     {
        printf("\n   SUCCESS\n");
     }
}

process_COMMAND_BL_DIS_R_W_PROTECT(uint8_t len)
{
     uint8_t status=0;
     read_serial_port(&status,len);
     if(status)
     {
         printf("\n   FAIL \n");
     }else
     {
        printf("\n   SUCCESS\n");
     }

}

int check_flash_status(void)
{
    uint8_t ack[2]={0};
    //uint32_t i =0;
    int ret = -1;

    uint8_t ch = 0;

    //The MCU bootloader always sends ack/nack first . read that !!
    read_serial_port(ack,2);
    if(ack[0] == 0xA5)
    {
        //CRC of last command was good .. received ACK and "len to follow"
        printf("ACK received : %d\n",ack[1]);
        //ReadFile(hComm, &ch, 1, &read, NULL);
        read_serial_port(&ch,1);
        printf("flash status : %d\n",ch);
        ret = 0;
    }
    else if ( ack[0] == 0x7F)
    {
        //CRC of last command was bad .. received NACK
        printf("\nNACK received \n");
        ret= -1;
    }

    return ret;
}

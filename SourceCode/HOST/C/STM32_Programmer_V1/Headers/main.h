#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#include <Windows.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

//enable one of the below macro according to your HOST
#define WINDOWS_HOST
//#define LINUX_HOST
//s#define OSX_HOST

#ifdef WINDOWS_HOST
    #include "WindowsSerialPort.h"
#endif

#ifdef LINUX_HOST
    #include "LinuxSerialPort.h"
#endif

#ifdef OSX_HOST
    #include "OSxSerialPort.h"
#endif

//Bl commands prototypes
void decode_menu_command_code(uint32_t command_code);

//BL Reply Process prototypes
void process_COMMAND_BL_MY_NEW_COMMAND(uint32_t len);
void process_COMMAND_BL_EN_R_W_PROTECT(uint8_t len);
void process_COMMAND_BL_DIS_R_W_PROTECT(uint8_t len);
void process_COMMAND_BL_READ_SECTOR_STATUS(uint32_t len);
void process_COMMAND_BL_MEM_WRITE(uint32_t len);
void process_COMMAND_BL_FLASH_ERASE(uint32_t len);
void process_COMMAND_BL_FLASH_MASS_ERASE(uint32_t len_to_follow);
void process_COMMAND_BL_GO_TO_ADDR(uint32_t len);
void process_COMMAND_BL_GET_RDP_STATUS(uint32_t len);
void process_COMMAND_BL_GET_CID(uint32_t len);
void process_COMMAND_BL_GET_HELP(uint32_t len);
void process_COMMAND_BL_GET_VER(uint32_t len);
int read_bootloader_reply(uint8_t command_code);
int check_flash_status(void);

//utilities Prototypes
uint32_t get_crc(uint8_t *buff, uint32_t len);
uint8_t word_to_byte(uint32_t addr, uint8_t index, uint8_t lowerfirst);

//file ops
void close_the_file(void);
uint32_t read_the_file(uint8_t *buffer, uint32_t len);
void open_the_file(void);
uint32_t calc_file_len(void);

//BL Commands
#define COMMAND_BL_GET_VER                  0x51
#define COMMAND_BL_GET_HELP                 0x52
#define COMMAND_BL_GET_CID                  0x53
#define COMMAND_BL_GET_RDP_STATUS           0x54
#define COMMAND_BL_GO_TO_ADDR               0x55
#define COMMAND_BL_FLASH_ERASE              0x56
#define COMMAND_BL_MEM_WRITE                0x57
#define COMMAND_BL_EN_R_W_PROTECT           0x58
#define COMMAND_BL_MEM_READ                 0x59
#define COMMAND_BL_READ_SECTOR_P_STATUS     0x5A
#define COMMAND_BL_OTP_READ                 0x5B
#define COMMAND_BL_DIS_R_W_PROTECT          0x5C
#define COMMAND_BL_MY_NEW_COMMAND           0x5D

//len details of the command
#define COMMAND_BL_GET_VER_LEN              6
#define COMMAND_BL_GET_HELP_LEN             6
#define COMMAND_BL_GET_CID_LEN              6
#define COMMAND_BL_GET_RDP_STATUS_LEN       6
#define COMMAND_BL_GO_TO_ADDR_LEN           10
#define COMMAND_BL_FLASH_ERASE_LEN          8
#define COMMAND_BL_MEM_WRITE_LEN(x)         (7+x+4)
#define COMMAND_BL_EN_R_W_PROTECT_LEN       8
#define COMMAND_BL_READ_SECTOR_P_STATUS_LEN   6
#define COMMAND_BL_DIS_R_W_PROTECT_LEN       6
#define COMMAND_BL_MY_NEW_COMMAND_LEN        8


//These addresses are used for GO_TO_ADDR for testing purpose.
#define GO_TO_ADDR1 0x20001234
#define GO_TO_ADDR2 0x40804434
#define GO_TO_ADDR3 0x08001234
#define GO_TO_ADDR4 0x12801234
#define GO_TO_ADDR5 0X08008248

//This is the flash sector 2 base address where we have stored the user application
#define USE_APP_FLASH_BASE_ADDR 0X08008000

#define RESET_HANDLER_ADDR_OF_APP 0x080081D8

//MCU HAL driver operation status .. currently not used.
 /**
  * @brief  HAL Status structures definition
  */
typedef enum
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

//MCU HAL Flash driver operation status .. currently not used
 /**
  * @brief  Flash HAL Status structures definition
  */
typedef enum
{
  Flash_HAL_OK       = 0x00U,
  Flash_HAL_ERROR    = 0x01U,
  Flash_HAL_BUSY     = 0x02U,
  Flash_HAL_TIMEOUT  = 0x03U,
  Flash_HAL_INV_ADDR = 0x04U
} HAL_FlashStatusTypeDef;


typedef union
{
    uint16_t flash_sector_status;
    struct
    {
        uint16_t sector0:2;
        uint16_t sector1:2;
        uint16_t sector2:2;
        uint16_t sector3:2;
        uint16_t sector4:2;
        uint16_t sector5:2;
        uint16_t sector6:2;
        uint16_t sector7:2;

    }sectors;

}t_sector_status;


#endif // MAIN_H_INCLUDED

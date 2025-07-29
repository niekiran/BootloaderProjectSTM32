/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define BUTTON2_Pin GPIO_PIN_4
#define BUTTON2_GPIO_Port GPIOC
#define BUTTON2_EXTI_IRQn EXTI4_IRQn
#define LED4_Pin GPIO_PIN_5
#define LED4_GPIO_Port GPIOC
#define BUTTON1_Pin GPIO_PIN_1
#define BUTTON1_GPIO_Port GPIOB
#define BUTTON1_EXTI_IRQn EXTI1_IRQn
#define USER_APPLICATION_Pin GPIO_PIN_12
#define USER_APPLICATION_GPIO_Port GPIOB
#define USER_APPLICATION_EXTI_IRQn EXTI15_10_IRQn
#define LED3_Pin GPIO_PIN_6
#define LED3_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000U

/*Bootloader function prototypes */

void  bootloader_uart_read_data(void);
void bootloader_jump_to_user_app(void);

void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer);
void bootloader_handle_getcid_cmd(uint8_t *pBuffer);
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer);
void bootloader_handle_go_cmd(uint8_t *pBuffer);
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer);
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer);
void bootloader_handle_en_rw_protect(uint8_t *pBuffer);
void bootloader_handle_mem_read (uint8_t *pBuffer);
void bootloader_handle_read_sector_protection_status(uint8_t *pBuffer);
void bootloader_handle_read_otp(uint8_t *pBuffer);
void bootloader_handle_dis_rw_protect(uint8_t *pBuffer);

void bootloader_send_ack(uint8_t command_code, uint8_t follow_len);
void bootloader_send_nack(void);

uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len,uint32_t crc_host);
uint8_t get_bootloader_version(void);
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len);

uint16_t get_mcu_chip_id(void);
uint8_t get_flash_rdp_level(void);
uint8_t verify_address(uint32_t go_address);
uint8_t execute_flash_erase(uint8_t sector_number , uint8_t number_of_sector);
uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len);

uint8_t configure_flash_sector_rw_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable);

uint16_t read_OB_rw_protection_status(void);

//version 1.0
#define BL_VERSION 0x10

// our bootloader commands

//#define  <command name >  <command_code>

//This command is used to read the bootloader version from the MCU
#define BL_GET_VER        0x51

//This command is used to know what are the commands supported by the bootloader
#define BL_GET_HELP       0x52

//This command is used to read the MCU chip identification number
#define BL_GET_CID        0x53

//This command is used to read the FLASH Read Protection level.
#define BL_GET_RDP_STATUS   0x54

//This command is used to jump bootloader to specified address.
#define BL_GO_TO_ADDR     0x55

//This command is used to mass erase or sector erase of the user flash .
#define BL_FLASH_ERASE          0x56

//This command is used to write data in to different memories of the MCU
#define BL_MEM_WRITE      0x57

//This command is used to enable or disable read/write protect on different sectors of the user flash .
#define BL_EN_RW_PROTECT    0x58

//This command is used to read data from different memories of the microcontroller.
#define BL_MEM_READ       0x59

//This command is used to read all the sector protection status.
#define BL_READ_SECTOR_P_STATUS 0x5A


//This command is used to read the OTP contents.
#define BL_OTP_READ       0x5B


//This command is used disable all sector read/write protection
#define BL_DIS_R_W_PROTECT        0x5C

/* ACK and NACK bytes*/
#define BL_ACK   0XA5
#define BL_NACK  0X7F

/*CRC*/
#define VERIFY_CRC_FAIL    1
#define VERIFY_CRC_SUCCESS 0

#define ADDR_VALID 0x00
#define ADDR_INVALID 0x01

#define INVALID_SECTOR 0x04

/*Some Start and End addresses of different memories of STM32F446xx MCU */
/*Change this according to your MCU */
#define SRAM1_SIZE            112*1024     // STM32F446RE has 112KB of SRAM1
#define SRAM1_END             (SRAM1_BASE + SRAM1_SIZE)
#define SRAM2_SIZE            16*1024     // STM32F446RE has 16KB of SRAM2
#define SRAM2_END             (SRAM2_BASE + SRAM2_SIZE)
#define FLASH_SIZE             512*1024     // STM32F446RE has 512KB of SRAM2
#define BKPSRAM_SIZE           4*1024     // STM32F446RE has 4KB of SRAM2
#define BKPSRAM_END            (BKPSRAM_BASE + BKPSRAM_SIZE)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

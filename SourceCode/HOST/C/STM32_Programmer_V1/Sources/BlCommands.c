

/* This file implements logic to decode the user input , prepare and send the bootloader command packet
 * over the serial port. This file is common across win/linux/mac
 */

#include "main.h"

//Decode the Bootloader command selection by the user
void decode_menu_command_code(uint32_t command_code)
{

    unsigned char data_buf[255];
    uint32_t crc32=0; int ret_value=0;

    switch(command_code)
    {
    case 0:
        printf("\n  Exiting...!");
        exit(0);

    case 1:
        printf("\n   Command == > BL_GET_VER");

        data_buf[0] = COMMAND_BL_GET_VER_LEN-1;
        data_buf[1] = COMMAND_BL_GET_VER;
        crc32       = get_crc(data_buf,COMMAND_BL_GET_VER_LEN-4);
        data_buf[2] = word_to_byte(crc32,1,1);
        data_buf[3] = word_to_byte(crc32,2,1);
        data_buf[4] = word_to_byte(crc32,3,1);
        data_buf[5] = word_to_byte(crc32,4,1);

        Write_to_serial_port(&data_buf[0],1);
        Write_to_serial_port(&data_buf[1],COMMAND_BL_GET_VER_LEN-1);

        ret_value = read_bootloader_reply(data_buf[1]);

        break;

    case 2:
        printf("\n   Command == > BL_GET_HLP");

        data_buf[0] = COMMAND_BL_GET_HELP_LEN-1;
        data_buf[1] = COMMAND_BL_GET_HELP;
        crc32       = get_crc(data_buf,COMMAND_BL_GET_HELP_LEN-4);
        data_buf[2] = word_to_byte(crc32,1,1);
        data_buf[3] = word_to_byte(crc32,2,1);
        data_buf[4] = word_to_byte(crc32,3,1);
        data_buf[5] = word_to_byte(crc32,4,1);

        Write_to_serial_port(&data_buf[0],1);
        Write_to_serial_port(&data_buf[1],COMMAND_BL_GET_HELP_LEN-1);

        ret_value = read_bootloader_reply(data_buf[1]);

        break;

    case 3:
        printf("\n   Command == > BL_GET_CID");

        data_buf[0] = COMMAND_BL_GET_CID_LEN-1;
        data_buf[1] = COMMAND_BL_GET_CID;
        crc32       = get_crc(data_buf,COMMAND_BL_GET_CID_LEN-4);
        data_buf[2] = word_to_byte(crc32,1,1);
        data_buf[3] = word_to_byte(crc32,2,1);
        data_buf[4] = word_to_byte(crc32,3,1);
        data_buf[5] = word_to_byte(crc32,4,1);

        Write_to_serial_port(&data_buf[0],1);
        Write_to_serial_port(&data_buf[1],COMMAND_BL_GET_CID_LEN-1);

        ret_value = read_bootloader_reply(data_buf[1]);

        break;

    case 4:
        printf("\n   Command == > BL_GET_RDP_STATUS");

        data_buf[0] = COMMAND_BL_GET_RDP_STATUS_LEN-1;
        data_buf[1] = COMMAND_BL_GET_RDP_STATUS;
        crc32       = get_crc(data_buf,COMMAND_BL_GET_RDP_STATUS_LEN-4);
        data_buf[2] = word_to_byte(crc32,1,1);
        data_buf[3] = word_to_byte(crc32,2,1);
        data_buf[4] = word_to_byte(crc32,3,1);
        data_buf[5] = word_to_byte(crc32,4,1);

        Write_to_serial_port(&data_buf[0],1);
        Write_to_serial_port(&data_buf[1],COMMAND_BL_GET_RDP_STATUS_LEN-1);

        ret_value = read_bootloader_reply(data_buf[1]);

        break;

    case 5:
        printf("\n   Command == > BL_GO_TO_ADDR");
        printf("\n\n   Enter Address here :");
        uint32_t go_address;
        scanf(" %x",&go_address);

        data_buf[0] = COMMAND_BL_GO_TO_ADDR_LEN-1;
        data_buf[1] = COMMAND_BL_GO_TO_ADDR;
#if 0
        data_buf[2] = word_to_byte(RESET_HANDLER_ADDR_OF_APP,1,1);
        data_buf[3] = word_to_byte(RESET_HANDLER_ADDR_OF_APP,2,1);
        data_buf[4] = word_to_byte(RESET_HANDLER_ADDR_OF_APP,3,1);
        data_buf[5] = word_to_byte(RESET_HANDLER_ADDR_OF_APP,4,1);
#endif
        data_buf[2] = word_to_byte(go_address,1,1);
        data_buf[3] = word_to_byte(go_address,2,1);
        data_buf[4] = word_to_byte(go_address,3,1);
        data_buf[5] = word_to_byte(go_address,4,1);
        crc32       = get_crc(data_buf,COMMAND_BL_GO_TO_ADDR_LEN-4);
        data_buf[6] = word_to_byte(crc32,1,1);
        data_buf[7] = word_to_byte(crc32,2,1);
        data_buf[8] = word_to_byte(crc32,3,1);
        data_buf[9] = word_to_byte(crc32,4,1);

        Write_to_serial_port(&data_buf[0],1);
        Write_to_serial_port(&data_buf[1],COMMAND_BL_GO_TO_ADDR_LEN-1);

        ret_value = read_bootloader_reply(data_buf[1]);

        break;

    case 6:
        printf("\n   This command is not supported\n");
        break;

    case 7:
        printf("\n   Command == > BL_FLASH_ERASE");

        data_buf[0] = COMMAND_BL_FLASH_ERASE_LEN-1;
        data_buf[1] = COMMAND_BL_FLASH_ERASE;
        uint32_t sector_num,nsec;

        printf("\n  Enter sector number(0-7 or 0xFF) here :");
        scanf(" %x",&sector_num);
        if(sector_num != 0xff)
        {
            printf("\n  Enter number of sectors to erase(max 8) here :");
            scanf(" %d",&nsec);

        }


        data_buf[2]= sector_num;
        data_buf[3]= nsec;
       // printf(" sector num : %d %d \n",sector_num,nsec);

        crc32       = get_crc(data_buf,COMMAND_BL_FLASH_ERASE_LEN-4);
        data_buf[4] = word_to_byte(crc32,1,1);
        data_buf[5] = word_to_byte(crc32,2,1);
        data_buf[6] = word_to_byte(crc32,3,1);
        data_buf[7] = word_to_byte(crc32,4,1);

        Write_to_serial_port(&data_buf[0],1);
        Write_to_serial_port(&data_buf[1],COMMAND_BL_FLASH_ERASE_LEN-1);

        ret_value = read_bootloader_reply(data_buf[1]);

        break;

    case 8:
        printf("\n   Command == > BL_MEM_WRITE");
        uint32_t bytes_remaining=0;
        uint32_t t_len_of_file=0;
        uint32_t bytes_so_far_sent = 0,len_to_read=0;
        uint32_t base_mem_address=0;

        data_buf[1] = COMMAND_BL_MEM_WRITE;

        //First get the total number of bytes in the .bin file.
        t_len_of_file =calc_file_len();

        //keep opening the file
        open_the_file();

        bytes_remaining = t_len_of_file - bytes_so_far_sent;

#if 0
        //Here you should  get it from user input . but
        //for testing purpose just hard coded.
        base_mem_address = USE_APP_FLASH_BASE_ADDR;
#endif
        printf("\n\n   Enter the memory write address here :");
        scanf(" %x",&base_mem_address);

        while(bytes_remaining)
        {

            if(bytes_remaining >= 128)
            {
                len_to_read = 128;
            }else
            {
                len_to_read = bytes_remaining;
            }

            //get the bytes in to buffer by reading file
            read_the_file(&data_buf[7],len_to_read);

            printf("\n   base mem address = %#.8x\n",base_mem_address);

            //populate base mem address
            data_buf[2] = word_to_byte(base_mem_address,1,1);
            data_buf[3] = word_to_byte(base_mem_address,2,1);
            data_buf[4] = word_to_byte(base_mem_address,3,1);
            data_buf[5] = word_to_byte(base_mem_address,4,1);

            data_buf[6] = len_to_read;

            /* 1 byte len + 1 byte command code + 4 byte mem base address
             * 1 byte payload len + len_to_read is amount of bytes read from file + 4 byte CRC
             */
            uint32_t mem_write_cmd_total_len = COMMAND_BL_MEM_WRITE_LEN(len_to_read);

            //first field is "len_to_follow"
            data_buf[0] =mem_write_cmd_total_len-1;

            crc32       = get_crc(&data_buf[0],mem_write_cmd_total_len-4);
            data_buf[7+len_to_read] = word_to_byte(crc32,1,1);
            data_buf[8+len_to_read] = word_to_byte(crc32,2,1);
            data_buf[9+len_to_read] = word_to_byte(crc32,3,1);
            data_buf[10+len_to_read] = word_to_byte(crc32,4,1);

            //update base mem address for the next loop
            base_mem_address+=len_to_read;

            Write_to_serial_port(&data_buf[0],1);
            Write_to_serial_port(&data_buf[1],mem_write_cmd_total_len-1);

            bytes_so_far_sent+=len_to_read;
            bytes_remaining = t_len_of_file - bytes_so_far_sent;

            printf("\n\n    bytes_so_far_sent:%d -- bytes_remaining:%d\n",bytes_so_far_sent,bytes_remaining);

            ret_value = read_bootloader_reply(data_buf[1]);

         }
        break;

    case 9:
        printf("\n   Command == > BL_EN_R_W_PROTECT");
        printf("\n\n   How many sectors do you want to protect ?: ");
        uint32_t total_sector,mode;
        char user_input;
        uint32_t sector_details = 0x00;
        uint8_t sector_numbers[8];
        scanf(" %d",&total_sector);
        for(uint32_t i=0 ; i < total_sector ; i++)
        {
            printf("\n   Enter sector number[%d]:",i+1);
            scanf(" %d",&sector_numbers[i]);
            sector_details = sector_details | (1 << sector_numbers[i]);
        }

       // printf("\n   Sector info : 0x%x",sector_details);

        printf("\n   Mode:Flash sectors Write Protection: 1");
        printf("\n   Mode:Flash sectors Read/Write Protection: 2");
        printf("\n   Enter Sector Protection Mode(1 or 2 ):");
        scanf(" %d",&mode);
        if(mode != 2 && mode != 1)
        {
            printf("\n   Invalid option : Command Dropped");
            return;
        }
        if(mode == 2)
        {
#if 0
          printf("\n   Mass Erase will be performed if you later want to change the Protection mode");
          printf("\n   Are you sure (y/n) ?:");
          scanf(" %c",&user_input);
          if( (user_input != 'Y' ) && (user_input != 'y') )
          {
            printf("\n   Command Dropped");
            return ;

          }
#endif
           printf("\n   This feature is currently not supported !");
           return ;

        }

        data_buf[0] = COMMAND_BL_EN_R_W_PROTECT_LEN-1;
        data_buf[1] = COMMAND_BL_EN_R_W_PROTECT;
        data_buf[2] = sector_details;
        data_buf[3] = mode;
        crc32       = get_crc(data_buf,COMMAND_BL_EN_R_W_PROTECT_LEN-4);
        data_buf[4] = word_to_byte(crc32,1,1);
        data_buf[5] = word_to_byte(crc32,2,1);
        data_buf[6] = word_to_byte(crc32,3,1);
        data_buf[7] = word_to_byte(crc32,4,1);

        Write_to_serial_port(&data_buf[0],1);
        Write_to_serial_port(&data_buf[1],COMMAND_BL_EN_R_W_PROTECT_LEN-1);

        ret_value = read_bootloader_reply(data_buf[1]);
        break;
    case 10:
        printf("\n   Command == > COMMAND_BL_MEM_READ");
        printf("\n   This command is not supported");
        break;
    case 11:
        printf("\n   Command == > COMMAND_BL_READ_SECTOR_P_STATUS");

        data_buf[0] = COMMAND_BL_READ_SECTOR_P_STATUS_LEN-1;
        data_buf[1] = COMMAND_BL_READ_SECTOR_P_STATUS;

        crc32       = get_crc(data_buf,COMMAND_BL_READ_SECTOR_P_STATUS_LEN-4);
        data_buf[2] = word_to_byte(crc32,1,1);
        data_buf[3] = word_to_byte(crc32,2,1);
        data_buf[4] = word_to_byte(crc32,3,1);
        data_buf[5] = word_to_byte(crc32,4,1);

        Write_to_serial_port(&data_buf[0],1);
        Write_to_serial_port(&data_buf[1],COMMAND_BL_READ_SECTOR_P_STATUS_LEN-1);

        ret_value = read_bootloader_reply(data_buf[1]);

        break;
    case 12:

        printf("\n   Command == > COMMAND_OTP_READ");
        printf("\n   This command is not supported");

        break;
    case 13:
        printf("\n   Command == > COMMAND_BL_DIS_R_W_PROTECT");
        //printf("\n   This command is not supported");
        data_buf[0] = COMMAND_BL_DIS_R_W_PROTECT_LEN-1;
        data_buf[1] = COMMAND_BL_DIS_R_W_PROTECT;
        crc32       = get_crc(data_buf,COMMAND_BL_DIS_R_W_PROTECT_LEN-4);
        data_buf[2] = word_to_byte(crc32,1,1);
        data_buf[3] = word_to_byte(crc32,2,1);
        data_buf[4] = word_to_byte(crc32,3,1);
        data_buf[5] = word_to_byte(crc32,4,1);

        Write_to_serial_port(&data_buf[0],1);
        Write_to_serial_port(&data_buf[1],COMMAND_BL_DIS_R_W_PROTECT_LEN-1);

        ret_value = read_bootloader_reply(data_buf[1]);
        break;

    case 14:
        printf("\n   Command == > COMMAND_BL_MY_NEW_COMMAND ");
        data_buf[0] = COMMAND_BL_MY_NEW_COMMAND_LEN-1;
        data_buf[1] = COMMAND_BL_MY_NEW_COMMAND;
        crc32       = get_crc(data_buf,COMMAND_BL_MY_NEW_COMMAND_LEN-4);
        data_buf[2] = word_to_byte(crc32,1,1);
        data_buf[3] = word_to_byte(crc32,2,1);
        data_buf[4] = word_to_byte(crc32,3,1);
        data_buf[5] = word_to_byte(crc32,4,1);

        Write_to_serial_port(&data_buf[0],1);
        Write_to_serial_port(&data_buf[1],COMMAND_BL_MY_NEW_COMMAND_LEN-1);

        ret_value = read_bootloader_reply(data_buf[1]);
        break;

    default:
        {
            printf("\n\n  Please input valid command code\n");
            return;
        }

    }

        if(ret_value == -2)
        {
            printf("\n\n   TimeOut : No response from the bootloader");
            printf("\n   Reset the board and Try Again !\n");
            return;
        }


}

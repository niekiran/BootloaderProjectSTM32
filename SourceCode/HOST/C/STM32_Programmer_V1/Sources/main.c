#include "main.h"

int main()
{
     /*----------------------------- Ask Menu implementation----------------------------------------*/
    printf("\n\n +==========================================+");
    printf("\n |           STM32F4 BootLoader v1           |");
    printf("\n +==========================================+\n");


    Serial_Port_Configuration();

    while(1)
    {
#if 1
        printf("\n\n +==========================================+");
        printf("\n |                   Menu                   |");
        printf("\n +==========================================+\n");
#endif
        printf("\n\n   Which BL command do you want to send ??\n");
        printf("\n   BL_GET_VER                     --> 1");
        printf("\n   BL_GET_HLP                     --> 2");
        printf("\n   BL_GET_CID                     --> 3");
        printf("\n   BL_GET_RDP_STATUS              --> 4");
        printf("\n   BL_GO_TO_ADDR                  --> 5");
        printf("\n   BL_FLASH_MASS_ERASE            --> 6");
        printf("\n   BL_FLASH_ERASE                 --> 7");
        printf("\n   BL_MEM_WRITE                   --> 8");
        printf("\n   BL_EN_R_W_PROTECT              --> 9");
        printf("\n   BL_MEM_READ                    --> 10");
        printf("\n   BL_READ_SECTOR_P_STATUS        --> 11");
        printf("\n   BL_OTP_READ                    --> 12");
        printf("\n   BL_DIS_R_W_PROTECT             --> 13");
        printf("\n   BL_MY_NEW_COMMAND              --> 14");
        printf("\n   MENU_EXIT                      --> 0");

        printf("\n\n   Type the command code here :");

        uint32_t command_code;
        scanf(" %d",&command_code);

        decode_menu_command_code(command_code);

#if 0
        printf("\n\n   Do you want to continue(y/n) ?:");
        uint8_t proceed = 0;
        scanf(" %c",&proceed);
        proceed -= 'y';
        if ( proceed)
        {
            printf("\n  ****** Thank you ! Exiting ******\n");
            break;
        }
#endif
        printf("\n\n   Press any key to continue  :");
        uint8_t ch = getch();
        purge_serial_port();
   }


}

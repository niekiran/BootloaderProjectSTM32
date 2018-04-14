
#include "main.h"

#ifdef WINDOWS_HOST



HANDLE hComm;                          // Handle to the Serial port

void Serial_Port_Configuration(void)
{
    char   ComPortName[] = "\\\\.\\COM3"; // Name of the Serial port(May Change) to be opened,
    BOOL   Status;

/*----------------------------------- Opening the Serial Port --------------------------------------------*/
    hComm = CreateFile( ComPortName,                       // Name of the Port to be Opened
                        GENERIC_READ | GENERIC_WRITE,      // Read/Write Access
                        0,                                 // No Sharing, ports cant be shared
                        NULL,                              // No Security
                        OPEN_EXISTING,                     // Open existing port only
                        0,                                 // Non Overlapped I/O
                        NULL);                             // Null for Comm Devices

    if (hComm == INVALID_HANDLE_VALUE)
    {
         printf("\n   Error! - Port %s can't be opened", ComPortName);
         printf("\n   Check board connection and Port Number\n");
         exit(-1);
    }

    else
        printf("\n   Port %s Opened\n ", ComPortName);

/*------------------------------- Setting the Parameters for the SerialPort ------------------------------*/

    DCB dcbSerialParams = { 0 };                        // Initializing DCB structure
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    Status = GetCommState(hComm, &dcbSerialParams);     //retreives  the current settings

    if (Status == FALSE)
        printf("\n   Error! in GetCommState()");

    dcbSerialParams.BaudRate = 115200;      // Setting BaudRate = 9600
    dcbSerialParams.ByteSize = 8; // Setting ByteSize = 8
    dcbSerialParams.fBinary = 1;
    dcbSerialParams.StopBits = ONESTOPBIT;    // Setting StopBits = 1
    dcbSerialParams.Parity   = NOPARITY;      // Setting Parity = None

    Status = SetCommState(hComm, &dcbSerialParams);  //Configuring the port according to settings in DCB

    if (Status == FALSE)
    {
        printf("\n   Error! in Setting DCB Structure");
    }
    else
    {
        printf("\n   Setting DCB Structure Successfull\n");
        printf("\n       Baudrate = %ld", dcbSerialParams.BaudRate);
        printf("\n       ByteSize = %d", dcbSerialParams.ByteSize);
        printf("\n       StopBits = %d", dcbSerialParams.StopBits);
        printf("\n       Parity   = %d", dcbSerialParams.Parity);
    }
/*------------------------------------ Setting Timeouts --------------------------------------------------*/
#if 1
    COMMTIMEOUTS timeouts = { 0 };

    timeouts.ReadIntervalTimeout         = 300;
    timeouts.ReadTotalTimeoutConstant    = 300;
    timeouts.ReadTotalTimeoutMultiplier  = 300;
    timeouts.WriteTotalTimeoutConstant   = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (SetCommTimeouts(hComm, &timeouts) == FALSE)
        printf("\n   Error! in Setting Time Outs");
    else
        printf("\n\n   Setting Serial Port Timeouts Successfull");

    Status = SetCommMask(hComm, EV_RXCHAR); //Configure Windows to Monitor the serial device for Character Reception

    if (Status == FALSE)
        printf("\n\n   Error! in Setting CommMask");
    else
        printf("\n\n   Setting CommMask successfull");
#endif



}

/* This function reads from the serial port and returns count of  bytes read */
uint32_t read_serial_port(uint8_t *pBuffer, uint32_t len)
{
    uint32_t no_of_bytes_read;

    ReadFile(hComm, pBuffer, len, &no_of_bytes_read, NULL);

    return no_of_bytes_read;

}

//closes the serial port
void Close_serial_port(void)
{
    CloseHandle(hComm);//Closing the Serial Port
}

//explore : https://msdn.microsoft.com/en-us/library/windows/desktop/aa363428(v=vs.85).aspx
void purge_serial_port(void)
{
     PurgeComm(hComm,PURGE_RXCLEAR|PURGE_TXCLEAR);

}


//This fun is used to Send data over the serial port of "len" bytes
void Write_to_serial_port(uint8_t *data_buf, uint32_t len)
{
    DWORD  dNoOfBytesWritten = 0;          // No of bytes written to the port
    BOOL   Status;

    Status = WriteFile( hComm,               // Handle to the Serialport
                        data_buf,            // Data to be written to the port
                        len,   // No of bytes to write into the port
                        &dNoOfBytesWritten,  // No of bytes written to the port
                        NULL);

    if (Status == TRUE)
    {
        printf("\n   Sending Command:\n");
        for(uint32_t i = 0 ; i < len ; i++)
        {
            printf("   0x%2.2x ",data_buf[i]);
            if( i % 8 == 7)
            {
                printf("\n");
            }
        }
    }
    else
        printf("\n  Error %ld in Writing to Serial Port",GetLastError());
}

#endif // WINDOWS_HOST

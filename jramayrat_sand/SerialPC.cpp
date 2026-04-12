/*******************************************************************************
 This code is originally inspired by
 https://stackoverflow.com/questions/25590539/sending-data-through-serial-port-in-c

 By: Sai Hein Si Thu   
 Updated: 2023-MAY-06
*******************************************************************************/

#include "SerialPC.h"
#include <string>


SerialPC::SerialPC() {}


SerialPC::~SerialPC() {}


int SerialPC::setup(char* portName, int baudrate) {
    /* Open the highest available serial port number */
    fprintf(stderr, "Opening serial port...");
    hSerial = CreateFileA(
                static_cast<LPCSTR>(portName), GENERIC_READ|GENERIC_WRITE, 0, NULL,
                OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL );
    if (hSerial == INVALID_HANDLE_VALUE)
    {
            fprintf(stderr, "Error\n");
            return 1;
    }
    else fprintf(stderr, "OK\n");


    /* Set device parameters (by default: 8 bits/byte, 1 start bit, 
     1 stop bit, no parity) */
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (GetCommState(hSerial, &dcbSerialParams) == 0)
    {
        fprintf(stderr, "Error getting device state\n");
        CloseHandle(hSerial);
        return 1;
    }

    //originally it's CBR_38400, which is a #define constant
    dcbSerialParams.BaudRate = baudrate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if(SetCommState(hSerial, &dcbSerialParams) == 0)
    {
        fprintf(stderr, "Error setting device parameters\n");
        CloseHandle(hSerial);
        return 1;
    }


    /* Set COM port timeout settings */
    timeouts.ReadIntervalTimeout = 2;  //original: 50
    timeouts.ReadTotalTimeoutConstant = 2;  //50
    timeouts.ReadTotalTimeoutMultiplier = 10;  //10
    timeouts.WriteTotalTimeoutConstant = 50;  //50
    timeouts.WriteTotalTimeoutMultiplier = 10;  //10
    if(SetCommTimeouts(hSerial, &timeouts) == 0)
    {
        fprintf(stderr, "Error setting timeouts\n");
        CloseHandle(hSerial);
        return 1;
    }

    return 0;
}


int SerialPC::write(std::string theta_str_n) {
    // Send specified text (remaining command line arguments)
    DWORD bytes_written, total_bytes_written = 0;
    //fprintf(stderr, "Sending bytes...");
    
    int num_bytes = strlen(theta_str_n.c_str());

    //writes to serial and checks if failed or not with (!) in one line
    if(!WriteFile(hSerial, theta_str_n.c_str(), num_bytes, &bytes_written, NULL))
    {
        fprintf(stderr, "Error\n");
        CloseHandle(hSerial);
        return 1;
    }
    
    return 0;
    //fprintf(stderr, "%d bytes written\n", bytes_written);
}


int SerialPC::read(char* buffer, unsigned int buf_size) {
    DWORD bytesRead;
    unsigned int toRead = buf_size;

    ClearCommError(hSerial, &errors, &status);  //uncomment this later if needed

    if(status.cbInQue > 0) {
        if(status.cbInQue > buf_size) {
            toRead = buf_size;
        }
        else
            toRead = status.cbInQue;
    }

    if(ReadFile(hSerial, buffer, toRead, &bytesRead, NULL)) {
        return bytesRead;
    }

    return 0;
}


int SerialPC::close() {
    if (connected == true) {
        connected = false;
        // Close serial port
        fprintf(stderr, "Closing serial port...");
        if (CloseHandle(hSerial) == 0)
        {
            fprintf(stderr, "Error\n");
            return 1;
        }
    }

    fprintf(stderr, "OK\n");

    // exit normally
    return 0;
}

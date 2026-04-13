// #pragma once


#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>


class SerialPC {
    private:
                DCB dcbSerialParams = {0};
        COMMTIMEOUTS timeouts = {0};
        bool connected;
        COMSTAT status;
        DWORD errors;
    public:
        HANDLE hSerial;
        SerialPC();
        ~SerialPC();

        int setup(char* portName, int baudrate);
        int write(std::string theta_str_n);
        int read(char* buffer, unsigned int buf_size);
        int close();

        /* int ReadSerialPort(char *buffer, unsigned int buff_size);
        //bool WriteSerialPort(char *buffer, unsigned int buff_size);  //no definition
        bool isConnected(); */
};
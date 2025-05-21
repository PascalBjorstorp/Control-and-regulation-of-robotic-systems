#ifndef UARTCOM_H
#define UARTCOM_H

// library headers
#include <stdio.h>
#include <string.h>
#include <cstdint>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()



class UARTcom
{
private:

    int _serial_port;
    float minFloat = -12;
    float maxFloat = 12;

public:
    UARTcom();
    uint8_t convertFloatToMsg(int motorSelect, float inputNumber);
    bool receivemsg(int& motor, float& angle);
    void sendmsg(int motorSelect, float inputNumber);
};
#endif // UARTCOM_H

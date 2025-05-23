#include "uartcom.h"
#include <bitset>
#include <cmath>
#include <iostream>
#include <thread>

UARTcom::UARTcom() {
    _serial_port = open("/dev/ttyACM0", O_RDWR);                     // Her åbner vi serie port forbindelsen

    // Check for errors
    if (_serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));     // Her tjekker vi for om den har fundet en forbindelse til porten
    }

    struct termios tty;                                                 // Her benytter vi structen termios som er en inbygget struct i linux til at konfigurere vores port
    if(tcgetattr(_serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB;                                             // Her slår vi parity bit fra

    tty.c_cflag &= ~CSTOPB;                                             // Sætter at der skal være 1 stop bit

    tty.c_cflag &= ~CSIZE;                                              // Clear alle size bit og bruger den næste linje til at sætte størrelsen på dataen
    tty.c_cflag |= CS8;                                                 // Her sætter vi vores port til at bruge 8 bits data

    tty.c_cflag &= ~CRTSCTS;                                            // Disable RTS/CTS som betyder at man har 2 ekstra ledninger til at fortælle hvornår den skal sende og modtage det har vi ikke så den er slået fra

    tty.c_cflag |= CREAD | CLOCAL;                                      // CREAD betyder at vi kan læse fra porten og CLOCAL betyder at vi ignorerer modem control lines
    tty.c_lflag &= ~ICANON;                                             // Her slår vi canonical mode fra som betyder at vi læser en byte ad gangen
    tty.c_lflag &= ~ECHO;                                               // Disable echo

    tty.c_lflag &= ~ECHOE;                                              // Disable erasure
    tty.c_lflag &= ~ECHONL;                                             // Disable new-line echo

    tty.c_lflag &= ~ISIG;                                               // Her slår vi fra at serial porten læser de her komandoer ud fra dataen INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                             // Her slår vi software flow control fra

    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);    // Her slår vi fra at der skal være speciel behandling af input bytes vi vil bare have dem som de er dejlig rå data

    tty.c_oflag &= ~OPOST;                                              // Her fjerner vi speciel tolkelse af output data som fx newline chars
    tty.c_oflag &= ~ONLCR;                                              // Igen bare at vi fortsætter med at fjerne special tolkelse af output data

    tty.c_cc[VTIME] = 0;                                                // Her sætter vi timeout til 0 så den venter ikke på at der kommer noget data i forhold til tid hvis den ikke får noget data så går den videre
                                                                        // men hvis den får data indenfor intervaller af timeout fortsætter den
    tty.c_cc[VMIN] = 1;                                                 // Venter på at mindst 1 byte er læst ind før den går videre altså den venter her for evigt indtil den får en byte data

    cfsetispeed(&tty, B115200);                                           // Sætter in og output baud rate til 9600
    cfsetospeed(&tty, B115200);

    if (tcsetattr(_serial_port, TCSANOW, &tty) != 0) {                   // Gemmer de nye settings vi har defineret indtil nu og tjekker for fejl
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
}

uint8_t UARTcom::convertFloatToMsg(int motorSelect, float value){
    uint8_t data = static_cast<uint8_t>(value);

    data &= 0x7F;  // Clear bit 8 to ensure it's not set

    if (motorSelect != 0) {
        data |= 0x80;  // Set bit 8
    }

    return data;
}

void UARTcom::sendmsg(int motorSelect, float inputNumber) {
    uint8_t msg = convertFloatToMsg(motorSelect, inputNumber);                                         /* Her skriver vi vores besked  */
    write(_serial_port, &msg, sizeof(msg));

    float angle = inputNumber;
    std::string debugCunt = std::bitset<8>(msg).to_string();
    //std::cout << motorSelect << ": " <<debugCunt << " Receive Angle: " << angle << std::endl;
}

/**
 * @brief Receives a single byte from UART and decodes it to motor and angle.
 * @param[out] motorSelect 0 for motor 1, 1 for motor 2
 * @param[out] angle       Angle in range [-5, 5]
 * @return true if a byte was read, false otherwise
 */

bool UARTcom::receivemsg(int& motorSelect, float& angle) {
    uint8_t msg = 0;
    ssize_t bytesRead = read(_serial_port, &msg, 1);
    if (bytesRead != 1) {
        return false; // No data read
    }

    motorSelect = (msg & 0x80) ? 1 : 0; // MSB: 0 = motor 1, 1 = motor 2

    uint8_t value = msg & 0x7F; // 7 LSB
    angle = (static_cast<float>(value) - 64.0f); // Convert to range [-64, 63]
    //std::cout << "Receive Motor: " << motorSelect << ", Angle: " << angle << std::endl;
    return true;
}

// Simulation of receiving a message
/*
bool UARTcom::receivemsg(int& motor, float& angle) {
    static float t = 0;
    t += 0.03f;
    // Simulate a sine wave for X and Y angles
    static bool toggle = false;
    toggle = !toggle;
    if (toggle) {
        motor = 0; // X
        angle = 1.0f * std::sin(t); // Simulate X tilt
    } else {
        motor = 1; // Y
        angle = 1.0f * std::cos(t); // Simulate Y tilt
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Simulate UART delay
    return true;
}
*/

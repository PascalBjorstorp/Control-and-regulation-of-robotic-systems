#include "updater.h"

int main() {
    // Create the updater object and start the main update loop
    cv::Mat img = cv::imread("/home/aksel/Documents/GitHub/Control-and-regulation-of-robotic-systems/Vision/ForSimulation/test_img_2.jpg", cv::IMREAD_COLOR);
    Updater updater(img);
    updater.update();

    UARTcom uart;
    uart.sendmsg(0, 0);
    uart.sendmsg(1, 0);
    return 0;
}

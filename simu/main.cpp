#include "updater.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

std::vector<cv::Point2f> clickedPoints;

void onMouse(int event, int x, int y, int, void*) {
    if (event == cv::EVENT_LBUTTONDOWN && clickedPoints.size() < 4) {
        clickedPoints.push_back(cv::Point2f(x, y));
        std::cout << "Clicked: " << x << ", " << y << std::endl;
    }
}

int main() {
    // Create the updater object and start the main update loop
    //cv::Mat img = cv::imread("/home/aksel/Documents/GitHub/Control-and-regulation-of-robotic-systems/Vision/ForSimulation/test_img_2.jpg", cv::IMREAD_COLOR);
    
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Could not open camera!" << std::endl;
        return -1;
    }

    // Capture the first frame for calibration
    cv::Mat img;
    cap >> img;
    if (img.empty()) {
        std::cerr << "Failed to capture image from camera!" << std::endl;
        return -1;
    }

    // Save the captured image for calibration
    cv::imwrite("calibration_image.jpg", img);

    // Interactive calibration: click 4 corners
    cv::namedWindow("Calibration Image");
    cv::setMouseCallback("Calibration Image", onMouse, nullptr);
    std::cout << "Click the 4 corners of the board in order: top-left, top-right, bottom-right, bottom-left." << std::endl;

    while (clickedPoints.size() < 4) {
        cv::Mat display = img.clone();
        for (const auto& pt : clickedPoints)
            cv::circle(display, pt, 5, cv::Scalar(0, 0, 255), -1);
        cv::imshow("Calibration Image", display);
        if (cv::waitKey(10) == 27) break; // ESC to exit
    }
    cv::destroyAllWindows();

    if (clickedPoints.size() != 4) {
        std::cerr << "Calibration failed: not enough points clicked." << std::endl;
        return -1;
    }

    // Define real-world board corners (in mm, e.g. 220x220 mm)
    std::vector<cv::Point2f> boardCorners = {
        {0, 0},
        {220, 0},
        {220, 220},
        {0, 220}
    };

    // Compute homography
    cv::Mat H = cv::findHomography(clickedPoints, boardCorners);

    // Save homography to file
    cv::FileStorage fs("homography.yml", cv::FileStorage::WRITE);
    fs << "H" << H;
    fs.release();
    std::cout << "Homography saved to homography.yml" << std::endl;

    // Pass the calibration image to the simulation
    Updater updater(img);
    updater.update();

    return 0;
}

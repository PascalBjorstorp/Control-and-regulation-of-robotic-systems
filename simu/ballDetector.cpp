#include "ballDetector.h"
#include <limits>
#include <algorithm>

BallDetector::BallDetector() {
    // Load homography from file
    cv::FileStorage fs("homography.yml", cv::FileStorage::READ);
    if (fs.isOpened()) {
        fs["H"] >> homography;
        fs.release();
    } else {
        std::cerr << "Warning: Could not load homography.yml. Ball positions will be in pixels!" << std::endl;
        homography = cv::Mat(); // Empty
    }
}

bool BallDetector::getBallPosition(float& x, float& y) {
    std::lock_guard<std::mutex> lock(posMutex);
    if(!hasBall) return false;

    x = _ballX / 8;
    y = _ballY / 8;
    
    return true;
}

void BallDetector::detectionLoop() {
    cv::VideoCapture cap(2);
    if (!cap.isOpened()) return;

    cv::Mat img, blue_img;
    while (running) {
        cap >> img;
        if (img.empty()){
            std::cerr << "Failed to capture image" << std::endl;
            continue;
        }

        // Apply homography and crop to match main.cpp
        if (!homography.empty()) {
            cv::Mat warped;
            cv::warpPerspective(img, warped, homography, cv::Size(880, 880));
            img = warped;
        }

        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(100, 150, 50), cv::Scalar(140, 255, 255), blue_img);

        std::vector<int> locations1(4), locations2(4), locations3(4), locations4(4);
        int rows = img.rows, cols = img.cols;

        #pragma omp parallel sections
        {
            #pragma omp section
            outerPoints(blue_img, 0, 0, rows/2, cols/2, locations1);
            #pragma omp section
            outerPoints(blue_img, 0, cols/2, rows/2, cols, locations2);
            #pragma omp section
            outerPoints(blue_img, rows/2, 0, rows, cols/2, locations3);
            #pragma omp section
            outerPoints(blue_img, rows/2, cols/2, rows, cols, locations4);
        }

        int top = std::min({locations1[0], locations2[0], locations3[0], locations4[0]});
        int bottom = std::max({locations1[1], locations2[1], locations3[1], locations4[1]});
        int left = std::min({locations1[2], locations2[2], locations3[2], locations4[2]});
        int right = std::max({locations1[3], locations2[3], locations3[3], locations4[3]});

        if (bottom > 0 && top < rows && right > 0 && left < cols) {
            float cx = (left + right) / 2.0f;
            float cy = (top + bottom) / 2.0f;
            {
                std::lock_guard<std::mutex> lock(posMutex);
                _ballX = cx;
                _ballY = cy;
                hasBall = true;
            }
        
            // Draw a circle at the detected ball position
            //cv::Mat imgWithCircle = img.clone();
            //cv::circle(imgWithCircle, cv::Point(static_cast<int>(cx), static_cast<int>(cy)), 15, cv::Scalar(0, 0, 255), 2);

            // Show the image (for debugging)
            //cv::imshow("Ball Detection", imgWithCircle);
            //cv::waitKey(1); // Needed to update the window
        
        } else {
            std::lock_guard<std::mutex> lock(posMutex);
            hasBall = false;
        }
    }
}

void BallDetector::outerPoints(const cv::Mat& img, int startRow, int startCol, int endRow, int endCol, std::vector<int>& locations) {
    locations[0] = std::numeric_limits<int>::max(); // top
    locations[1] = std::numeric_limits<int>::min(); // bottom
    locations[2] = std::numeric_limits<int>::max(); // left
    locations[3] = std::numeric_limits<int>::min(); // right
    for (int j = startRow; j < endRow; ++j) {
        const uchar* rowPtr = img.ptr<uchar>(j);
        for (int i = startCol; i < endCol; ++i) {
            if (rowPtr[i]) {
                if (j < locations[0]) locations[0] = j;
                if (j > locations[1]) locations[1] = j;
                if (i < locations[2]) locations[2] = i;
                if (i > locations[3]) locations[3] = i;
            }
        }
    }
}

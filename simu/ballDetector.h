#ifndef BALL_DETECTOR_H
#define BALL_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>

class BallDetector {
private:
    std::mutex posMutex;
    float _ballX = 0.0f, _ballY = 0.0f;
    std::string device;
    bool hasBall = false;

public:
    std::atomic<bool> running{false};
    BallDetector(const std::string& device = "/dev/video0");
    void detectionLoop();
    bool getBallPosition(float& x, float& y);
    void outerPoints(const cv::Mat& img, int startRow, int startCol, int endRow, int endCol, std::vector<int>& locations);
};

#endif // BALL_DETECTOR_H

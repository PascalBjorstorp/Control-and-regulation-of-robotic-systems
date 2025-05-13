#ifndef UPDATER_H
#define UPDATER_H

#include <sstream>
#include <iomanip>
#include <iostream>
#include <SFML/Graphics.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include "ball.h"
#include "maze.h"
#include "wall.h"
#include "uartcom.h"

class Updater
{
    sf::RenderWindow _window;
    sf::Font _font;
    sf::Text _angleText;
    Ball _ball;
    Maze _maze;
    UARTcom _uart;
    BallDetector _ballDetector{"/dev/video0"};

    // Thread variables
    std::thread angleRX;
    std::thread cameraThread;
    std::thread angleTX;
    std::thread physicsThread;
    std::thread ballDetect;

    // Mutex for thread-safe access to shared variables
    std::atomic<bool> running{true};
    std::mutex dataMutex;
    float receivedTiltX = 0.0f, receivedTiltY = 0.0f;
    float receivedBallX = 0.0f, receivedBallY = 0.0f;

    // Effectively a flag to indicate new data is available
    std::condition_variable dataCondVar;
    bool newDataAvailable = false;

public:
    /**
     * Constructor for the Updater class.
     * 
     * Initializes the window, font, text, ball, and maze objects.
     * Sets the window framerate limit to 60 FPS.
     */
    Updater(cv::Mat img);

    ~Updater();
    /**
     * Main update loop for the game.
     * 
     * This method handles all game logic, including:
     * - Event handling for window closure and ball reset
     * - Maze tilt updates based on manual or automatic control
     * - Ball physics updates based on tilt angles
     * - Collision detection between the ball and maze walls
     * - Display of tilt angles and auto-navigation status
     * - Rendering of all game elements (maze, walls, target, ball)
     */
    void update();
    void physicsUpdate();
    void angleUpdate();
    void cameraUpdate();
    void sendAngle();
};

#endif // UPDATER_H

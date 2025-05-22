#include "updater.h"

Updater::Updater(cv::Mat img)
: _window(sf::VideoMode(Constants::WINDOW_WIDTH, Constants::WINDOW_HEIGHT), "Maze Game"),
  _ball(Constants::WINDOW_WIDTH / 2, Constants::WINDOW_HEIGHT / 2),
    _maze(img),
    _ballDetector(){

    _window.setFramerateLimit(60);

    if (!_font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")) {
        // Handle font loading failure - program will continue without text
        std::cout << "Warning: Failed to load font. Angle display will not be shown." << std::endl;
    }

    logFile.open("updater_log.csv");
    logFile << "timestamp_ms,real_x,real_y,pred_x,pred_y\n";

    // Create text for displaying angles
    _angleText.setFont(_font);
    _angleText.setCharacterSize(18);
    _angleText.setFillColor(sf::Color::Black);
    _angleText.setPosition(Constants::WINDOW_WIDTH - 150, 10);

    running = true;
    angleRX = std::thread(&Updater::angleUpdate, this);
    cameraThread = std::thread(&Updater::cameraUpdate, this);
    angleTX = std::thread(&Updater::sendAngle, this);
    physicsThread = std::thread(&Updater::physicsUpdate, this);
    _ballDetector.running = true;
    ballDetect = std::thread(&BallDetector::detectionLoop, &_ballDetector);
}

Updater::~Updater() {
    running = false;
    dataCondVar.notify_all();
    if (angleRX.joinable()) angleRX.join();
    if (cameraThread.joinable()) cameraThread.join();
    if (angleTX.joinable()) angleTX.join();
    if (physicsThread.joinable()) physicsThread.join();
    _ballDetector.running = false;
    if (ballDetect.joinable()) ballDetect.join();
    if (logFile.is_open()) logFile.close();
}

// ...existing code...

void Updater::update(){
    sf::CircleShape ballShape;
    sf::ConvexShape mazeBackground;
    sf::VertexArray pathPoints;
    std::vector<sf::CircleShape> waypointMarkers;
    std::vector<Wall> walls;
    sf::CircleShape targetMarker;
    sf::Vector2f ballCenter;
    sf::Vector2f targetPos;
    std::vector<Point3D> waypoints = _maze.getPathWaypoints();;
    
    while (_window.isOpen()) {
        // Handle event
        sf::Event event;
        while (_window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                _window.close();
            }
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Escape) {
                    _window.close();
                }
            }
        }

        {
            std::unique_lock<std::mutex> lock(dataMutex);
            float tiltX = receivedTiltX;
            float tiltY = receivedTiltY;
            mazeBackground = _maze.getBackground();
            pathPoints = _maze.getPath();
            waypointMarkers = _maze.getWaypointMarkers();
            walls = _maze.getWalls();
            targetMarker = _maze.getTargetMarker();
            ballShape = _ball.getShape();

            // Get ball center
            ballCenter = ballShape.getPosition();

            // Get current waypoint as target (in pixels)
        }

        if (!waypoints.empty()) {
            if (waypointIdx >= waypoints.size()) waypointIdx = waypoints.size() - 1;
            targetPos = waypoints[waypointIdx].project();
        } else {
            targetPos = ballCenter;
        }

        // ...event handling and debug info...

        _window.clear(sf::Color::White);

        // Draw all game elements
        _window.draw(mazeBackground);
        _window.draw(pathPoints);
        for (const auto& marker : waypointMarkers) {
            _window.draw(marker);
        }
        _window.draw(_angleText);
        for (Wall& wall : walls) {
            _window.draw(wall.getShape());
        }
        _window.draw(targetMarker);
        _window.draw(ballShape);

        // Draw arrow from ball center to target waypoint
        sf::VertexArray arrow(sf::Lines, 2);
        arrow[0].position = ballCenter;
        arrow[0].color = sf::Color::Red;
        arrow[1].position = targetPos;
        arrow[1].color = sf::Color::Red;
        _window.draw(arrow);


        sf::Vector2f dir = targetPos - ballCenter;
        float length = std::sqrt(dir.x * dir.x + dir.y * dir.y);
        if (length > 1e-3) {
            dir /= length;
            sf::Vector2f perp(-dir.y, dir.x);
            float arrowHeadSize = 15.f;
            sf::Vector2f p1 = targetPos - dir * arrowHeadSize + perp * (arrowHeadSize / 2);
            sf::Vector2f p2 = targetPos - dir * arrowHeadSize - perp * (arrowHeadSize / 2);

            sf::VertexArray arrowHead(sf::Triangles, 3);
            arrowHead[0].position = targetPos;
            arrowHead[0].color = sf::Color::Red;
            arrowHead[1].position = p1;
            arrowHead[1].color = sf::Color::Red;
            arrowHead[2].position = p2;
            arrowHead[2].color = sf::Color::Red;
            _window.draw(arrowHead);
        }

        _window.display();
    }
}

void Updater::physicsUpdate() {
    auto prevTime = std::chrono::steady_clock::now();
    const float constValue = (5/7) * Constants::GRAVITY * 1000.0f; // m/s^2 to mm/s^2
    while (running) {
        float tiltX, tiltY;
        bool hasNewData = false;

        // Only lock while copying shared data
        {
            std::unique_lock<std::mutex> lock(dataMutex);
            tiltX = receivedTiltX;
            tiltY = receivedTiltY;
            if (newDataAvailable) {
                ballPosX_mm = receivedBallX;
                ballPosY_mm = receivedBallY;
                newDataAvailable = false;
                hasNewData = true;
            }
        }

        // Now do physics calculations outside the lock!
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - prevTime).count() / 1000.0f;
        prevTime = now;

        if (!hasNewData) {
            // Convert tilt to radians to calculate acceleration
            float ax = constValue * std::sin(tiltX * M_PI / 180.0f); // m/s^2
            float ay = constValue * std::sin(tiltY * M_PI / 180.0f);

            ballVelX += ax * dt;
            ballVelY += ay * dt;

            ballPosX_mm += ballVelX * dt;
            ballPosY_mm += ballVelY * dt;
        }

        float ballX = ((ballPosX_mm * 8.0f) - Constants::WALL_LENGTH / 2);
        float ballY = ((ballPosY_mm * 8.0f) - Constants::WALL_LENGTH / 2);

        {
            std::lock_guard<std::mutex> lock(dataMutex);
            _maze.setTiltX(tiltX);
            _maze.setTiltY(tiltY);
            _maze.updateProjection();
            _ball.setPosition3D(Point3D(ballX, ballY, 0));
            _ball.update(tiltX, tiltY);
        }
        //std::cout << "Ball Position: " << ballX << ", " << ballY << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void Updater::angleUpdate() {
    while (running) {
        int motor;
        float angle;
        if (_uart.receivemsg(motor, angle)) {
            std::unique_lock<std::mutex> lock(dataMutex);
            if (motor == 0) {
                receivedTiltX = angle  / 3;
            } else {
                receivedTiltY = angle / 3;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void Updater::cameraUpdate() {
    float prevX = 0, prevY = 0;
    auto prevTime = std::chrono::steady_clock::now();
    bool first = true;
    std::vector<Point3D> waypoints = _maze.getPathWaypoints();


    while (running) {
        float x, y;
        if (_ballDetector.getBallPosition(x, y)) {
            auto now = std::chrono::steady_clock::now();
            std::unique_lock<std::mutex> lock(dataMutex);

            receivedBallX = x;
            receivedBallY = y;

            // Log prediction error to file
            float predictedX = ballPosX_mm;
            float predictedY = ballPosY_mm;
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            if (logFile.is_open()) {
                logFile << ms << "," << x << "," << y << "," << predictedX << "," << predictedY << "\n";
            }

            if (!first) {
                float dx_mm = (x - prevX);
                float dy_mm = (y - prevY);
                float dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - prevTime).count() / 1000.0f;
                if (dt > 0) {
                    ballVelX = dx_mm / dt;
                    ballVelY = dy_mm / dt;
                }
            } else {
                ballVelX = 0;
                ballVelY = 0;
                first = false;
            }

            prevX = x;
            prevY = y;
            prevTime = now;

            sf::Vector2f targetPos = waypoints[waypointIdx].project();

            float targetX = targetPos.x / 8.0f; // convert from pixels to mm if necessary
            float targetY = targetPos.y / 8.0f;

            // Compute error in mm
            float errorX = targetX - ballPosX_mm;
            float errorY = targetY - ballPosY_mm;

            float distance = std::sqrt(errorX * errorX + errorY * errorY);

            // If close to waypoint, move to next
            if (distance < 6.0f) {
                if (waypointIdx < waypoints.size() - 1) {
                    waypointIdx++; // Move to the next waypoint
                } else {
                    waypointIdx = 0; // Reset to the first waypoint
                }
                continue;
            }
            lock.unlock();

            newDataAvailable = true;
            dataCondVar.notify_one();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(35));
    }
}

void Updater::sendAngle() {
    const float kp = 0.1f; // Proportional gain
    const float ki = 0.000f; // Integral gain
    const float kd = 0.1f; // Derivative gain
    const float gearRatio = 3.0f;
    const float maxTilt = 3.0f; // Max tilt in degrees
    const float maxTiltIntegral = 5.0f;
    float prev_errorX = 0.0f;
    float prev_errorY = 0.0f;
    float integralX = 0.0f;
    float integralY = 0.0f;
    auto prevTime = std::chrono::steady_clock::now();
    // Get current waypoint as target (in mm)
    std::vector<Point3D> waypoints = _maze.getPathWaypoints();

    while (running) {
        {
            std::unique_lock<std::mutex> lock(dataMutex);

            // Use the latest camera-updated position and velocity (in mm)
            float ballPosX = receivedBallX;
            float ballPosY = receivedBallY;

            lock.unlock();

            if (waypoints.empty()) continue;

            // Project waypoint to 2D and convert to mm if needed
            sf::Vector2f targetPos = waypoints[waypointIdx].project();

            float targetX = targetPos.x / 8.0f; // convert from pixels to mm if necessary
            float targetY = targetPos.y / 8.0f;

            // Compute error in mm
            float errorX = targetX - ballPosX;
            float errorY = targetY - ballPosY;

            auto now = std::chrono::steady_clock::now();
            float dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - prevTime).count() / 1000.0f;
            prevTime = now;

            float derivativeX = (errorX - prev_errorX) / dt; // dt = 0.1s
            float derivativeY = (errorY - prev_errorY) / dt;

            prev_errorX = errorX;
            prev_errorY = errorY;

            float distance = std::sqrt(errorX * errorX + errorY * errorY);

            // Ball velocity (in mm/s)
            lock.lock();
            float vx = ballVelX;
            float vy = ballVelY;
            lock.unlock();

            // Check if moving toward the target and above threshold
            float velocity_toward_target = (vx * errorX + vy * errorY) / (distance + 1e-6f); // projection
            float speed = std::sqrt(vx * vx + vy * vy);
            const float velocityThreshold = 10.0f; // mm/s, adjust as needed

            if (velocity_toward_target > 0 && speed > velocityThreshold) {
                // Moving toward target and fast enough: stop tilting
                integralX = 0.0f;
                integralY = 0.0f;
                _uart.sendmsg(0, 64); // Neutral
                _uart.sendmsg(1, 64);
                std::cout << "Correct direction:" << distance << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }

            integralX += errorX * dt;
            integralY += errorY * dt;

            // PD control for tilt (board coordinates)
            float tiltX = kp * errorX + kd * derivativeX;
            float tiltY = kp * errorY + kd * derivativeY;

            std::cout << "Error X: " << kp * errorX << ", Error Y: " << kp * errorY << std::endl;
            std::cout << "derivativeX: " << kd * derivativeX << ", derivativeY: " << kd* derivativeY << std::endl;

            // Clamp tilt
            tiltX = std::clamp(tiltX, -maxTilt, maxTilt);
            tiltY = std::clamp(tiltY, -maxTilt, maxTilt);

            // If not moving, apply integral control
            if(speed == 0){
                tiltX += ki * integralX;
                tiltY += ki * integralY;

                tiltX = std::clamp(tiltX, -maxTiltIntegral, maxTiltIntegral);
                tiltY = std::clamp(tiltY, -maxTiltIntegral, maxTiltIntegral);
            }

            // Convert to motor angles (apply gear ratio)
            float motorAngleX = tiltX * gearRatio;
            float motorAngleY = tiltY * gearRatio;

            //std::cout << "Motor Angle X: " << motorAngleX << ", Motor Angle Y: " << motorAngleY << std::endl;

            // Map to UART value (-64 to 63 mapped to 0-127)
            int uartX = static_cast<int>(std::round(-motorAngleX + 64));
            int uartY = static_cast<int>(std::round(-motorAngleY + 64));
            uartX = std::clamp(uartX, 0, 127);
            uartY = std::clamp(uartY, 0, 127);

            //std::cout << "Motor Angle X: " << uartX << ", Motor Angle Y: " << uartY << std::endl;

            // Send angles via UART
            _uart.sendmsg(0, uartX);
            _uart.sendmsg(1, uartY);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

/*
void Updater::sendAngle() {
    const float maxTilt = 3.0f; // Max tilt in degrees
    const float gearRatio = 3.0f;

    while (running) {
        // Read joystick input (assuming joystick 0, axes X and Y)
        float joyX = 0.0f;
        float joyY = 0.0f;
        if (sf::Joystick::isConnected(0)) {
            joyX = sf::Joystick::getAxisPosition(0, sf::Joystick::X) / 100.f; // [-1, 1]
            joyY = sf::Joystick::getAxisPosition(0, sf::Joystick::Y) / 100.f; // [-1, 1]
        }

        // Map joystick input to tilt angles
        float tiltX = std::clamp(joyX * maxTilt, -maxTilt, maxTilt);
        float tiltY = std::clamp(joyY * maxTilt, -maxTilt, maxTilt);

        // Convert to motor angles (apply gear ratio)
        float motorAngleX = tiltX * gearRatio;
        float motorAngleY = tiltY * gearRatio;

        // Map to UART value (-64 to 63 mapped to 0-127)
        int uartX = static_cast<int>(std::round(-motorAngleX + 64));
        int uartY = static_cast<int>(std::round(-motorAngleY + 64));
        uartX = std::clamp(uartX, 0, 127);
        uartY = std::clamp(uartY, 0, 127);

        // Send angles via UART
        _uart.sendmsg(0, uartX);
        _uart.sendmsg(1, uartY);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
*/

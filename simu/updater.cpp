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
    logFile << "timestamp_ms,distance,\n";

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
    tiltUpdateThread = std::thread(&Updater::tiltUpdateLoop, this);
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
    if (tiltUpdateThread.joinable()) tiltUpdateThread.join();
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
            mazeBackground = _maze.getBackground();
            pathPoints = _maze.getPath();
            waypointMarkers = _maze.getWaypointMarkers();
            walls = _maze.getWalls();
            targetMarker = _maze.getTargetMarker();
            ballShape = _ball.getShape();

            // Get current waypoint as target (in pixels)
        }

        // Get ball center
        ballCenter = ballShape.getPosition();

        if (!waypoints.empty()) {
            if (waypointIdx >= waypoints.size()) waypointIdx = waypoints.size() - 1;
            targetPos = waypoints[waypointIdx].project();
        } else {
            targetPos = ballCenter;
        }

        // ...event handling and debug info...

        // Write the angle in the top right corner
        _angleText.setString("Tilt X: " + std::to_string(currentTiltX) + "\nTilt Y: " + std::to_string(currentTiltY));
        _angleText.setPosition(Constants::WINDOW_WIDTH - _angleText.getGlobalBounds().width - 10, 10);
        _angleText.setFillColor(sf::Color::Black);

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
    float constValue = (5.0f/7.0f) * Constants::GRAVITY * 1000.0f; // m/s^2 to mm/s^2
    auto next = std::chrono::steady_clock::now();
    const auto period = std::chrono::milliseconds(1);

    while (running) {
        next += period;
        float tiltX, tiltY;
        bool hasNewData = false;

        // Only lock while copying shared data
        {
            std::unique_lock<std::mutex> lock(dataMutex);
            tiltX = currentTiltX / 3;
            tiltY = currentTiltY / 3;
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
            float ax, ay;

            if((tiltX > 0 && tiltY > 0) || (tiltX < 0 && tiltY < 0)){
                ax = -constValue * std::sin(tiltX * M_PI / 180.0f);
                ay = constValue * std::sin(tiltY * M_PI / 180.0f);
            } else {
                ax = constValue * std::sin(tiltX * M_PI / 180.0f);
                ay = -constValue * std::sin(tiltY * M_PI / 180.0f);
            }



            //std::cout << "Acceleration: " << ax << ", " << ay << std::endl;

            ballVelX += ax * dt;
            ballVelY += ay * dt;

            ballPosX_mm += ballVelX * dt;
            ballPosY_mm += ballVelY * dt;

            //std::cout << "Ball position mm: " << ballPosX_mm << ", " << ballPosY_mm << std::endl;

        }

        float ballX = ((ballPosX_mm * 8.0f) - Constants::WALL_LENGTH / 2);
        float ballY = ((ballPosY_mm * 8.0f) - Constants::WALL_LENGTH / 2);

        // Collision check and update
        bool collided = false;

        // Check X direction
        if (std::abs(ballX) > Constants::WALL_LENGTH / 2 - Constants::BALL_RADIUS) {
            collided = true;
            if (ballX > 0) {
                ballX = Constants::WALL_LENGTH / 2 - Constants::BALL_RADIUS;
            } else {
                ballX = -Constants::WALL_LENGTH / 2 + Constants::BALL_RADIUS;
            }
            // Reverse and dampen velocity
            ballVelX *= -0.5f;
            // Update mm position to match new pixel position
            ballPosX_mm = (ballX + Constants::WALL_LENGTH / 2) / 8.0f;
        }

        // Check Y direction
        if (std::abs(ballY) > Constants::WALL_LENGTH / 2 - Constants::BALL_RADIUS) {
            collided = true;
            if (ballY > 0) {
                ballY = Constants::WALL_LENGTH / 2 - Constants::BALL_RADIUS;
            } else {
                ballY = -Constants::WALL_LENGTH / 2 + Constants::BALL_RADIUS;
            }
            ballVelY *= -0.5f;
            ballPosY_mm = (ballY + Constants::WALL_LENGTH / 2) / 8.0f;
        }

        {
            std::lock_guard<std::mutex> lock(dataMutex);
            _maze.setTiltX(tiltX);
            _maze.setTiltY(tiltY);
            _maze.updateProjection();
            _ball.setPosition3D(Point3D(ballX, ballY, 0));
            _ball.update(tiltX, tiltY);
        }

        //std::cout << "Ball Position: " << ballX << ", " << ballY << std::endl;
        std::this_thread::sleep_until(next);
    }
}

void Updater::angleUpdate() {
    auto next = std::chrono::steady_clock::now();
    const auto period = std::chrono::milliseconds(1);
    float step = 0;
    while (running) {
        //step += 0.001f;
        next += period;
        int motor;
        float angle;
        /*{
        std::unique_lock<std::mutex> lock(dataMutex);
        receivedTiltX = 3.33 * sin(step);
        receivedTiltY = 3.33 * cos(step);
        }
        */

        if (_uart.receivemsg(motor, angle)) {
            std::unique_lock<std::mutex> lock(dataMutex);
            if (motor == 0) {
                receivedTiltX = angle  / 3;
            } else {
                receivedTiltY = angle / 3;
            }
        }
        std::this_thread::sleep_until(next);
    }
}

void Updater::cameraUpdate() {
    float prevX = 0, prevY = 0;
    auto prevTime = std::chrono::steady_clock::now();
    auto firstTime = std::chrono::steady_clock::now();
    bool first = true;
    std::vector<Point3D> waypoints = _maze.getPathWaypoints();
    sf::Vector2f targetPos;
    auto next = std::chrono::steady_clock::now();
    const auto period = std::chrono::milliseconds(20);

    while (false) {
        float x, y;
        next += period;
        if (_ballDetector.getBallPosition(x, y)) {
            auto now = std::chrono::steady_clock::now();

            // Calculate velocities and errors outside the lock
            float dx_mm = (x - prevX);
            float dy_mm = (y - prevY);
            float dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - prevTime).count() / 1000.0f;
            float vx = 0, vy = 0;
            if (!first && dt > 0) {
                vx = dx_mm / dt;
                vy = dy_mm / dt;
            }
            prevX = x;
            prevY = y;
            prevTime = now;
            first = false;

            // Only lock when updating shared variables
            {
                std::unique_lock<std::mutex> lock(dataMutex);
                receivedBallX = x;
                receivedBallY = y;
                ballVelX = vx;
                ballVelY = vy;
            }

            // Waypoint logic (lock only when updating waypointIdx)
            targetPos = waypoints[waypointIdx].project();
            float targetX = targetPos.x / 8.0f;
            float targetY = targetPos.y / 8.0f;
            float errorX = targetX - x;
            float errorY = targetY - y;
            float distance = std::sqrt(errorX * errorX + errorY * errorY);

            if (distance < 8.0f) {
                std::unique_lock<std::mutex> lock(dataMutex);
                if (waypointIdx < waypoints.size() - 1) {
                    waypointIdx++;
                } else {
                    waypointIdx = 0;
                }
                continue;
            }

            newDataAvailable = true;
            dataCondVar.notify_one();
        }
        std::this_thread::sleep_until(next);
    }
}

void Updater::sendAngle() {
    const float kp = 0.016f; // Proportional gain
    const float ki = 0.f; // Integral gain
    const float kd = 0.05f; // Derivative gain
    const float gearRatio = 3.0f;
    const float maxTilt = 3.33f; // Max tilt in degrees
    const float maxTiltIntegral = 7.0f;
    float prev_errorX = 0.0f;
    float prev_errorY = 0.0f;
    float integralX = 0.0f;
    float integralY = 0.0f;
    auto firstTime = std::chrono::steady_clock::now();
    auto prevTime = std::chrono::steady_clock::now();
    auto next = std::chrono::steady_clock::now();
    const auto period = std::chrono::milliseconds(500);
    // Get current waypoint as target (in mm)
    std::vector<Point3D> waypoints = _maze.getPathWaypoints();

    while (running) {
        {
            next += period;
            std::unique_lock<std::mutex> lock(dataMutex);

            // Use the latest camera-updated position and velocity (in mm)
            float ballPosX = ballPosX_mm;
            float ballPosY = ballPosY_mm;

            lock.unlock();

            if (waypoints.empty()) continue;

            // Project waypoint to 2D and convert to mm if needed
            sf::Vector2f targetPos = waypoints[waypointIdx].project();

            float targetX = targetPos.x / 8.0f; // convert from pixels to mm if necessary
            float targetY = targetPos.y / 8.0f;

            // Compute error in mm
            float errorX = targetX - (ballPosX + (Constants::BALL_RADIUS/8.0f));
            float errorY = targetY - (ballPosY + (Constants::BALL_RADIUS/8.0f));

            auto now = std::chrono::steady_clock::now();
            float dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - prevTime).count() / 1000.0f;
            prevTime = now;

            float derivativeX = (errorX - prev_errorX) / dt; // dt = 0.1s
            float derivativeY = (errorY - prev_errorY) / dt;

            prev_errorX = errorX;
            prev_errorY = errorY;

            float distance = std::sqrt(errorX * errorX + errorY * errorY);

            // Log prediction error to file
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - firstTime).count();
            if (logFile.is_open()) {
                logFile << ms << "," << distance << "\n";
            }

            lock.lock();
            // If close to waypoint, move to next
            if (distance < 8.0f) {
                if (waypointIdx < waypoints.size() - 1) {
                    waypointIdx++; // Move to the next waypoint
                } else {
                    waypointIdx = 0; // Reset to the first waypoint
                }
                continue;
            }
            lock.unlock();

            //std::cout << "Distance: " << distance << std::endl;

            // Ball velocity (in mm/s)
            lock.lock();
            float vx = ballVelX;
            float vy = ballVelY;
            lock.unlock();

            // Check if moving toward the target and above threshold
            float velocity_toward_target = (vx * errorX + vy * errorY) / (distance + 1e-6f); // projection
            float speed = std::sqrt(vx * vx + vy * vy);
            const float velocityThreshold = 10.0f; // mm/s, adjust as needed
/*
            if (velocity_toward_target > 0 && speed > velocityThreshold) {
                // Moving toward target and fast enough: stop tilting
                integralX = 0.0f;
                integralY = 0.0f;
                _uart.sendmsg(0, 64); // Neutral
                _uart.sendmsg(1, 64);

                //std::cout << "Correct direction:" << distance << std::endl;
                std::this_thread::sleep_until(next);
                continue;
            }
*/

            //integralX += errorX * dt;
            //integralY += errorY * dt;

            // PD control for tilt (board coordinates)
            float tiltX = kp * errorX + kd * derivativeX;
            float tiltY = kp * errorY + kd * derivativeY;

            //std::cout << "Error X: " << kp * errorX << ", Error Y: " << kp * errorY << std::endl;
            //std::cout << "derivativeX: " << kd * derivativeX << ", derivativeY: " << kd* derivativeY << std::endl;

            // Clamp tilt
            tiltX = std::clamp(tiltX, -maxTilt, maxTilt);
            tiltY = std::clamp(tiltY, -maxTilt, maxTilt);

            tiltX = round(tiltX * gearRatio);
            tiltY = round(tiltY * gearRatio);


            updateTiltTowardsSetpoint();
            lock.lock();
            tiltSetpointX = tiltY;
            tiltSetpointY = -tiltX;
            lock.unlock();


            //std::cout << "Speed: " << speed << ", Tilt X: " << tiltX << ", Tilt Y: " << tiltY << std::endl;

            // If not moving, apply integral control
            /*
            if(speed < 0.01 && (tiltX != 0) && (tiltY != 0)){
                //std::cout << "Before integral control: " << tiltX << ", " << tiltY << std::endl;

                tiltX += ki * integralX;
                tiltY += ki * integralY;

                tiltX = std::clamp(tiltX, -maxTiltIntegral, maxTiltIntegral);
                tiltY = std::clamp(tiltY, -maxTiltIntegral, maxTiltIntegral);

                //std::cout << "After integral control: " << tiltX << ", " << tiltY << std::endl;
            }
            */

            // Convert to motor angles (apply gear ratio)
            float motorAngleX = tiltX;
            float motorAngleY = tiltY;

            std::cout << "Motor Angle X: " << motorAngleX << ", Motor Angle Y: " << motorAngleY << std::endl;

            // Map to UART value (-64 to 63 mapped to 0-127)
            int uartX = static_cast<int>(std::round(-motorAngleX + 64));
            int uartY = static_cast<int>(std::round(motorAngleY + 64));
            uartX = std::clamp(uartX, 0, 127);
            uartY = std::clamp(uartY, 0, 127);

            //std::cout << "Motor Angle X: " << uartX << ", Motor Angle Y: " << uartY << std::endl;

            // Send angles via UART
            _uart.sendmsg(0, uartY);

            std::this_thread::sleep_for(std::chrono::milliseconds(3));

            _uart.sendmsg(1, uartX);
        }

        std::this_thread::sleep_until(next);
    }
}

void Updater::updateTiltTowardsSetpoint() {
    std::lock_guard<std::mutex> lock(dataMutex);

    float stepX = (tiltSetpointX - currentTiltX) / steps;
    float stepY = (tiltSetpointY - currentTiltY) / steps;

    // Move currentTiltX towards tiltSetpointX
    if (std::abs(currentTiltX - tiltSetpointX) > 1e-3) {
        currentTiltX += stepX;
    } else {
        currentTiltX = tiltSetpointX;
    }

    // Move currentTiltY towards tiltSetpointY
    if (std::abs(currentTiltY - tiltSetpointY) > 1e-3) {
        currentTiltY += stepY;
    } else {
        currentTiltY = tiltSetpointY;
    }
}

void Updater::tiltUpdateLoop() {
    auto next = std::chrono::steady_clock::now();
    const auto period = std::chrono::milliseconds(1); // Fast update, e.g. 5 ms

    while (running) {
        next += period;
        updateTiltTowardsSetpoint();
        std::this_thread::sleep_until(next);
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

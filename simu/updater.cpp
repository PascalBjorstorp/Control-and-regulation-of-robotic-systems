#include "updater.h"

Updater::Updater(cv::Mat img)
: _window(sf::VideoMode(Constants::WINDOW_WIDTH, Constants::WINDOW_HEIGHT), "Maze Game"),
  _ball(Constants::WINDOW_WIDTH / 2, Constants::WINDOW_HEIGHT / 2),
    _maze(img),
    _ballDetector("/dev/video2"){
    
    _window.setFramerateLimit(60);

    if (!_font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")) {
        // Handle font loading failure - program will continue without text
        std::cout << "Warning: Failed to load font. Angle display will not be shown." << std::endl;
    }

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
    //_ballDetector.running = false;
    //ballDetect = std::thread(&BallDetector::detectionLoop, &_ballDetector);
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
}

void Updater::update(){
    while (_window.isOpen()) {
        float tiltX, tiltY;
        {
            std::lock_guard<std::mutex> lock(dataMutex);
            tiltX = _maze.getTiltX();
            tiltY = _maze.getTiltY();
        }

        //std::cout << "TiltX: " << tiltX << ", TiltY: " << tiltY << std::endl;

        sf::Event event;
        while (_window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                _window.close();
        }

        // Display debugger information in the right corner
        std::stringstream ss;
        ss << "X Tilt: " << std::fixed << std::setprecision(1) << _maze.getTiltX()
           << "\nY Tilt: " << std::fixed << std::setprecision(1) << _maze.getTiltY()
           << "\nVelocity x: " << std::fixed << std::setprecision(1) << _ball.getVelocity().x
           << "\nVelocity y: " << std::fixed << std::setprecision(1) << _ball.getVelocity().y
           << "\nAuto: " << (_maze.getAutoNavigationEnabled() ? "On" : "Off");
        _angleText.setString(ss.str());

        // Clear the window
        _window.clear(sf::Color::White);

        // Draw all game elements
        _window.draw(_maze.getBackground());   // Draw background first
        _window.draw(_maze.getPath());         // Draw path
        for (const auto& marker : _maze.getWaypointMarkers()) {
            _window.draw(marker);              // Draw waypoint markers
        }
        _window.draw(_angleText);              // Draw angle text on top 
        for (Wall& wall : _maze.getWalls()) {
            _window.draw(wall.getShape());     // Draw walls
        }
        _window.draw(_maze.getTargetMarker()); // Draw target marker
        _window.draw(_ball.getShape());        // Draw ball

        // Display the window contents
        _window.display();
    }
}

void Updater::physicsUpdate() { 
     auto prevTime = std::chrono::steady_clock::now();
    while (running) {
        std::unique_lock<std::mutex> lock(dataMutex);
        
        // Time step calculation
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - prevTime).count() / 1000.0f;
        prevTime = now;

        // Variables for tilt and ball position
        float tiltX = receivedTiltX;
        float tiltY = receivedTiltY;

        // Convert tilt to radians to calculate acceleration
        float ax = (5/7) * g * std::sin(tiltX * M_PI / 180.0f); // m/s^2
        float ay = (5/7) * g * std::sin(tiltY * M_PI / 180.0f);

        // Update velocity
        ballVelX += ax * dt * 1000.0f; // mm/s
        ballVelY += ay * dt * 1000.0f; // mm/s

        // Update position (in mm)
        ballPosX_mm += ballVelX * dt;
        ballPosY_mm += ballVelY * dt;

        if (newDataAvailable) {
            // Convert to mm by dividing by 4.0f (1 pixel = 4 mm)
            ballPosX_mm = receivedBallX;
            ballPosY_mm = receivedBallY;

            // new data has been received, reset the flag
            newDataAvailable = false;
        }

        // In pixels
        float ballX = (ballPosX_mm * 4.0f) - Constants::WINDOW_WIDTH / 2; // mm to pixels
        float ballY = (ballPosY_mm * 4.0f) - Constants::WINDOW_HEIGHT / 2; // mm to pixels

        // Update the maze tilt angles based on received data
        _maze.setTiltX(tiltX);
        _maze.setTiltY(tiltY);
        _maze.updateProjection();

        // Update the ball's position based on the tilt angles and ball position
        _ball.setPosition3D(Point3D(ballX, ballY, 0));
        //_ball.update(tiltX, tiltY);

        // Sleep for a short duration to control the update rate
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void Updater::angleUpdate() {
    while (running) {
        int motor;
        float angle;
        if (_uart.receivemsg(motor, angle)) {
            std::lock_guard<std::mutex> lock(dataMutex);
            if (motor == 0) {
                receivedTiltX = angle  / 3;
            } else {
                receivedTiltY = angle / 3;
            }
            dataCondVar.notify_one();
            newDataAvailable = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void Updater::cameraUpdate() {
    float prevX = 0, prevY = 0;
    auto prevTime = std::chrono::steady_clock::now();
    bool first = true;

    while (running) {
        float x, y;
        if (_ballDetector.getBallPosition(x, y)) {
            auto now = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lock(dataMutex);
            receivedBallX = x;
            receivedBallY = y;

            if (!first) {
                float dx_mm = (x - prevX);
                float dy_mm = (y - prevY)f;
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

            newDataAvailable = true;
            dataCondVar.notify_one();   
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    } 
}

void Updater::sendAngle() {
    const float kp = 2.0f; // Proportional gain
    const float kd = 0.5f; // Derivative gain
    const float gearRatio = 3.0f;
    const float maxTilt = 15.0f; // Max tilt in degrees

    size_t waypointIdx = 0;

    while (running) {
        {
            std::unique_lock<std::mutex> lock(dataMutex);

            // Use the latest camera-updated position and velocity (in mm)
            float ballPosX = receivedBallX;
            float ballPosY = receivedBallY;
            float ballVelX_local = ballVelX;
            float ballVelY_local = ballVelY;

            // Get current waypoint as target (in mm)
            const auto& waypoints = _maze.getPathWaypoints();
            if (waypoints.empty()) continue;

            lock.unlock();

            // Project waypoint to 2D and convert to mm if needed
            sf::Vector2f targetPos = waypoints[waypointIdx].project();
            float targetX = targetPos.x / 4.0f; // convert from pixels to mm if necessary
            float targetY = targetPos.y / 4.0f;

            // Compute error in mm
            float errorX = targetX - ballPosX;
            float errorY = targetY - ballPosY;
            float distance = std::sqrt(errorX * errorX + errorY * errorY);

            // If close to waypoint, move to next
            if (distance < 5.0f && waypointIdx < waypoints.size() - 1) {
                waypointIdx++;
                continue;
            }

            // Normalize error
            float normErrorX = (distance > 0) ? errorX / distance : 0.0f;
            float normErrorY = (distance > 0) ? errorY / distance : 0.0f;

            // PD control for tilt (board coordinates)
            float tiltX = kp * normErrorY - kd * ballVelY_local;
            float tiltY = -kp * normErrorX + kd * ballVelX_local;

            // Clamp tilt
            tiltX = std::clamp(tiltX, -maxTilt, maxTilt);
            tiltY = std::clamp(tiltY, -maxTilt, maxTilt);

            // Convert to motor angles (apply gear ratio)
            float motorAngleX = tiltX * gearRatio;
            float motorAngleY = tiltY * gearRatio;

            // Map to UART value (-64 to 63 mapped to 0-127)
            int uartX = static_cast<int>(std::round(motorAngleX + 64));
            int uartY = static_cast<int>(std::round(motorAngleY + 64));
            uartX = std::clamp(uartX, 0, 127);
            uartY = std::clamp(uartY, 0, 127);

            // Send angles via UART
            _uart.sendmsg(0, uartX);
            _uart.sendmsg(1, uartY);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
}
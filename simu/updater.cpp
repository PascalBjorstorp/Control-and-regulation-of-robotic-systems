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
}

void Updater::update(){
    while (_window.isOpen()) {
        float tiltX, tiltY;
        // Get all data from maze and ball
        sf::CircleShape ballShape;
        sf::ConvexShape mazeBackground;
        sf::VertexArray pathPoints;
        std::vector<sf::CircleShape> waypointMarkers;
        std::vector<Wall> walls;
        sf::CircleShape targetMarker;;
        // Lock the mutex to safely access shared data
        {
            std::unique_lock<std::mutex> lock(dataMutex);
            tiltX = receivedTiltX;
            tiltY = receivedTiltY;
            mazeBackground = _maze.getBackground();
            pathPoints = _maze.getPath();
            waypointMarkers = _maze.getWaypointMarkers();
            walls = _maze.getWalls();
            targetMarker = _maze.getTargetMarker();
            ballShape = _ball.getShape();
        }

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
        _window.draw(ballShape);        // Draw ball

        // Display the window contents
        _window.display();
    }
}

void Updater::physicsUpdate() {
    auto prevTime = std::chrono::steady_clock::now();
    while (running) {
        float tiltX, tiltY, ballX_mm, ballY_mm;
        bool hasNewData = false;

        // Only lock while copying shared data
        {
            std::unique_lock<std::mutex> lock(dataMutex);
            tiltX = receivedTiltX;
            tiltY = receivedTiltY;
            if (newDataAvailable) {
                ballX_mm = receivedBallX;
                ballY_mm = receivedBallY;
                newDataAvailable = false;
                hasNewData = true;
            } else {
                ballX_mm = ballPosX_mm;
                ballY_mm = ballPosY_mm;
            }
        }

        // Now do physics calculations outside the lock!
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - prevTime).count() / 1000.0f;
        prevTime = now;

        float ax = Constants::GRAVITY * std::sin(tiltX * M_PI / 180.0f);
        float ay = Constants::GRAVITY * std::sin(tiltY * M_PI / 180.0f);

        ballVelX += ax * dt * 1000.0f;
        ballVelY += ay * dt * 1000.0f;

        ballPosX_mm += ballVelX * dt;
        ballPosY_mm += ballVelY * dt;

        if (hasNewData) {
            ballPosX_mm = ballX_mm;
            ballPosY_mm = ballY_mm;
        }

        float ballX = ((ballPosX_mm * 4.0f) - Constants::WALL_LENGTH / 2);
        float ballY = ((ballPosY_mm * 4.0f) - Constants::WALL_LENGTH / 2);

        {
            std::lock_guard<std::mutex> lock(dataMutex);
            _maze.setTiltX(tiltX);
            _maze.setTiltY(tiltY);
            _maze.updateProjection();
            _ball.setPosition3D(Point3D(ballX, ballY, 0));
            _ball.update(tiltX, tiltY);
        }
        //std::cout << "Ball Position: " << ballX << ", " << ballY << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void Updater::angleUpdate() {
    while (running) {
        int motor;
        float angle;
        receivedTiltY = 1/3;
        if (_uart.receivemsg(motor, angle)) {
            std::unique_lock<std::mutex> lock(dataMutex);
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

            newDataAvailable = true;
            dataCondVar.notify_one();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

void Updater::sendAngle() {
    while (true) {
        // Generate a continous number between -15 and 15
        static float t = 0;
        int motor = 0;
        float angle = 0;
        t += 0.5f;
        // Simulate a sine wave for X and Y angles
        static bool toggle = false;
        toggle = !toggle;
        if (toggle) {
            motor = 0; // X
            angle = 10.0f * std::sin(t); // Simulate X tilt
        } else {
            motor = 1; // Y
            angle = 10.0f * std::cos(t); // Simulate Y tilt
        }

        // Send the angle to the UART
        _uart.sendmsg(motor, angle);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

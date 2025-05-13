#include "updater.h"

Updater::Updater(cv::Mat img)
: _window(sf::VideoMode(Constants::WINDOW_WIDTH, Constants::WINDOW_HEIGHT), "Maze Game"), 
  _ball(Constants::WINDOW_WIDTH / 4, Constants::WINDOW_HEIGHT / 4), _maze(img) {
    
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
    ballDetect = std::thread(&_ballDetector::detectionLoop, &_ballDetector);
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
    while (running) {
        std::unique_lock<std::mutex> lock(dataMutex);
        dataCondVar.wait(lock, [this]{ return newDataAvailable || !running; });
        if (!running) break;
        float tiltX = receivedTiltX;
        float tiltY = receivedTiltY;
        float ballX = receivedBallX;
        float ballY = receivedBallY;        
        newDataAvailable = false;
        lock.unlock();

        // Update the maze tilt angles based on received data
        _maze.setTiltX(tiltX);
        _maze.setTiltY(tiltY);
        _maze.updateProjection();

        // Update the ball's position based on the tilt angles and ball position
        float simX = ballX - Constants::WINDOW_WIDTH / 2;
        float simY = ballY - Constants::WINDOW_HEIGHT / 2;
        _ball.setPosition3D(Point3D(simX, simY, 0));
        _ball.update(tiltX, tiltY);
    }
}

void Updater::angleUpdate() {
    while (running) {
        int motor;
        float angle;
        if (_uart.receivemsg(motor, angle)) {
            std::lock_guard<std::mutex> lock(dataMutex);
            if (motor == 0) {
                receivedTiltX = angle;
            } else {
                receivedTiltY = angle;
            }
            dataCondVar.notify_one();
        } else {
            // Short sleep to avoid busy waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void Updater::cameraUpdate() {
    while (running) {
        float x, y;
        if (_ballDetector.getBallPosition(x, y)) {
            std::lock_guard<std::mutex> lock(dataMutex);
            receivedBallX = x;
            receivedBallY = y;
            newDataAvailable = true;
            dataCondVar.notify_one(); // Move inside the if-block
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void Updater::sendAngle() {
    while (false) {
        // Get desired angles from Maze (implement these getters in Maze)
        //float desiredX = _maze.getDesiredAngleX();
        //float desiredY = _maze.getDesiredAngleY();

        //_uart.sendmsg(0, desiredX); // 0 for X motor
        //_uart.sendmsg(1, desiredY); // 1 for Y motor

        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Adjust as needed
    }
}
#include "maze.h"
#include <iostream>

Maze::Maze(cv::Mat img) : lineDetecter(img) {
    // Setup the background plane as a rectangular convex shape
    // This represents the flat surface of the maze that will be tilted in 3D space
    _background.setPointCount(4);
    _background.setFillColor(sf::Color(200, 200, 200));

    // Define the corners of the maze in 3D space
    // The maze is centered at (0,0,0) with dimensions slightly smaller than the window
    // This allows the maze to be fully visible when tilted
    float mazeWidth = Constants::WALL_LENGTH;
    float mazeHeight = Constants::WALL_LENGTH;
    
    _backgroundCorners3D = {
        Point3D(-mazeWidth/2, -mazeHeight/2, 0),  // Top-left
        Point3D(mazeWidth/2, -mazeHeight/2, 0),   // Top-right
        Point3D(mazeWidth/2, mazeHeight/2, 0),    // Bottom-right
        Point3D(-mazeWidth/2, mazeHeight/2, 0)    // Bottom-left
    };

    // Create all the walls that make up the maze layout
    createMaze();

    // Initialize timing for motors
    _lastUpdateTime = _motorClock.getElapsedTime().asSeconds();

     // Initialize the 3D projection of all maze elements
    updateProjection();

    makeTarget(); // Create the target marker
}

void Maze::createMaze() {
    // Border walls definition to create the outer boundary of the maze
    // Parameters for Wall constructor: (x, y, length, isHorizontal)
    
    // Top horizontal border (slightly offset to properly connect with vertical borders)
    _walls.push_back(Wall(Constants::WALL_X + 5, Constants::WALL_Y + 5, Constants::WALL_LENGTH + 10, true));
    
    // Left vertical border
    _walls.push_back(Wall(Constants::WALL_X + 5, Constants::WALL_Y, Constants::WALL_LENGTH + 15, false));
    
    // Bottom horizontal border
    _walls.push_back(Wall(Constants::WALL_X + 5, Constants::WALL_LENGTH + Constants::WALL_Y + 10, Constants::WALL_LENGTH + 10, true));
    
    // Right vertical border
    _walls.push_back(Wall(Constants::WALL_LENGTH + Constants::WALL_X + 10, Constants::WALL_Y + 10, Constants::WALL_LENGTH, false));
}

void Maze::updateTilt() {  
    // Calculate input based on keyboard state
    // Note: The motors have momentum now, so we're adjusting target positions
    const float angleChangeRate = Constants::TILT_SPEED * 10.0f; // Degrees per second

    // Primary controls (arrow keys) - full tilt speed
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))  _tiltY += Constants::TILT_SPEED;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) _tiltY -= Constants::TILT_SPEED;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up))    _tiltX -= Constants::TILT_SPEED;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down))  _tiltX += Constants::TILT_SPEED;

    // Secondary controls (WASD) - half tilt speed for finer control
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))  _tiltX += Constants::TILT_SPEED/2;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))  _tiltX -= Constants::TILT_SPEED/2;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))  _tiltY -= Constants::TILT_SPEED/2;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))  _tiltY += Constants::TILT_SPEED/2;
    
    // Update the visual projection based on new tilt angles
    updateProjection();
}

void Maze::updateProjection() {
    // Update background projection
    // 1. This creates a copy of the 3D corners,
    // 2. Rotates them based on tilt angles,
    // 3. Projects them to 2D space,
    // 4. Updates the background shape with new points
    for (size_t i = 0; i < _backgroundCorners3D.size(); i++) {
        Point3D rotated = _backgroundCorners3D[i];
        rotated.rotate(_tiltX, _tiltY);
        sf::Vector2f projected = rotated.project();
        _background.setPoint(i, projected);
    }

    // Update the target marker projection by the same process
    Point3D rotatedTarget = _targetPosition3D;
    rotatedTarget.rotate(_tiltX, _tiltY);
    _targetPosition = rotatedTarget.project();
    _targetMarker.setPosition(_targetPosition);

    // Update wall projections
    for (auto& wall : _walls) {
        wall.updateProjection(_tiltX, _tiltY);
    }

    updatePathProjection();
}

void Maze::toggleAutoNavigation() {
    // Toggle the auto-navigation mode on or off
    _autoNavigationEnabled = !_autoNavigationEnabled;
}

void Maze::updateAutoNavigation(Ball& ball) {
    if (!_autoNavigationEnabled) return;

    float currentTime = _motorClock.getElapsedTime().asSeconds();
    float timeSinceControlUpdate = currentTime - _lastControlUpdateTime;

    // Check if there are any waypoints
    if (_pathWaypoints.empty()) {
        makeTarget(ball);
        return;
    }

    // Use _currentWaypointIndex to track the active waypoint.
    Point3D currentTarget = _pathWaypoints[_currentWaypointIndex];

    // Rotate and project the target
    Point3D rotatedTarget = currentTarget;
    rotatedTarget.rotate(_tiltX, _tiltY);
    sf::Vector2f targetPos = rotatedTarget.project();

    // Get the ball's current position and velocity.
    sf::Vector2f ballPos = ball.getShape().getPosition();
    sf::Vector2f ballVel = ball.getVelocity();

    // Compute position error and distance.
    sf::Vector2f positionError = targetPos - ballPos;
    float distanceToTarget = std::sqrt(positionError.x * positionError.x + positionError.y * positionError.y);

    // If the ball is close enough to the current waypoint, move to the next waypoint.
    const float waypointThreshold = 5.0f; // adjust as needed (in pixels)
    if (distanceToTarget < waypointThreshold) {
        if (_currentWaypointIndex < _pathWaypoints.size() - 1) {
            _currentWaypointIndex++; // Proceed to next waypoint
            currentTarget = _pathWaypoints[_currentWaypointIndex];
        } else {
            // Optionally generate a new target if the final waypoint is reached.
            makeTarget(ball);
            return;
        }
        // Update rotated target and targetPos after switching waypoints.
        rotatedTarget = currentTarget;
        rotatedTarget.rotate(_tiltX, _tiltY);
        targetPos = rotatedTarget.project();
        positionError = targetPos - ballPos;
        distanceToTarget = std::sqrt(positionError.x * positionError.x + positionError.y * positionError.y);
    }

    // Normalize the error for PD control.
    sf::Vector2f normalizedPosError(0, 0);
    if (distanceToTarget > 0)
        normalizedPosError = positionError / distanceToTarget;

    // Use fixed PD gains (from class members _kp and _kd).
    float proportionalGain = _kp;
    float derivativeGain = _kd;

    // Calculate control terms for tilt.
    float propTiltX = proportionalGain * normalizedPosError.y;
    float derivTiltX = -derivativeGain * ballVel.y;

    float propTiltY = -proportionalGain * normalizedPosError.x;
    float derivTiltY = derivativeGain * ballVel.x;

    // Compute desired tilt in board coordinates.
    float desiredTiltX = propTiltX + derivTiltX;
    float desiredTiltY = propTiltY + derivTiltY;

    // Apply tilt angle limits (in board coordinates).
    float maxTiltAngle = 5.f;
    desiredTiltX = std::clamp(desiredTiltX, -maxTiltAngle, maxTiltAngle);
    desiredTiltY = std::clamp(desiredTiltY, -maxTiltAngle, maxTiltAngle);

    // Apply gearing: motor target = board target * gearRatio.
    const float gearRatio = 3.0f;
    _motorX.setTargetPosition(desiredTiltX * gearRatio);
    _motorY.setTargetPosition(desiredTiltY * gearRatio);

    if (timeSinceControlUpdate >= 0.01f) {
        _lastControlUpdateTime = currentTime;
        // Compute the control signal (voltage) and update motor states.
        float controlVoltageX = _motorX.compute(timeSinceControlUpdate);
        float controlVoltageY = _motorY.compute(timeSinceControlUpdate);
        _motorX.update(timeSinceControlUpdate);
        _motorY.update(timeSinceControlUpdate);

        std::cout << "time since: " << timeSinceControlUpdate << " s" << std::endl;

        uart.sendmsg(0, controlVoltageX);
        uart.sendmsg(1, controlVoltageY);

        // When printing, convert motor position back to board tilt.
        float boardTiltX = _motorX.getPosition() / gearRatio;
        float boardTiltY = _motorY.getPosition() / gearRatio;

        /*std::cout << "Time: " << deltaTime << " s, Board Tilt X: "
                  << boardTiltX << " rad, Motor Speed: "
                  << _motorX.getOmega() << " rad/s, Voltage: "
                  << controlVoltageX << " V" << std::endl;
*/
        /*std::cout << "Time: " << deltaTime << " s, Board Tilt Y: "
                  << boardTiltY << " rad, Motor Speed: "
                  << _motorY.getOmega() << " rad/s, Voltage: "
                  << controlVoltageY << " V" << std::endl;
*/
        // Update the tilt angles for projection.
        _tiltX = boardTiltX;
        _tiltY = boardTiltY;
    }

    updateProjection();
}

void Maze::makeTarget(){
    // Setup the target marker that the ball needs to reach
    // This is visualized as a green circle on the maze
    _targetMarker.setRadius(Constants::TARGET_RADIUS);
    _targetMarker.setFillColor(sf::Color(100, 100, 255, 150)); // Semi-transparent blue
    _targetMarker.setOrigin(Constants::TARGET_RADIUS, Constants::TARGET_RADIUS);

    std::vector<cv::Point> points = get_inters();
    float imgWidth = get_img().cols;
    float imgHeight = get_img().rows;

    float mazeWidth = Constants::WALL_LENGTH;
    float mazeHeight = Constants::WALL_LENGTH;

    float xAdd = ((-mazeWidth/2)+Constants::WINDOW_WIDTH/2)/mazeWidth;
    float yAdd = ((-mazeHeight/2)+Constants::WINDOW_HEIGHT/2)/mazeHeight;

    // Create a random numbers between 0.15 and 0.85 to ensure the target is not too close to the edges
    float multiplierX = xAdd + points[points.size()-1].x/(imgWidth);

    // Create a random number between 0.2 and 0.8 to ensure the target is not too close to the edges
    float multiplierY = yAdd + points[points.size()-1].y/(imgHeight);

    // Set the 3D position of the target
    _targetPosition3D = Point3D(
        mazeWidth * multiplierX - Constants::WINDOW_WIDTH/2,    // X position
        mazeHeight * multiplierY - Constants::WINDOW_HEIGHT/2,  // Y position
        0                                                                 // Z position
    );

    // Calculate the 2D projected position of the target
    _targetPosition = _targetPosition3D.project();
    _targetMarker.setPosition(_targetPosition);

    // Generate a path for the ball to follow
    generatePath();
}

/**
 * @brief Generates a path of waypoints for the ball to follow
 * 
 * Creates a sequence of points from the starting position to the target,
 * forming a path through the maze that avoids walls.
 */
void Maze::generatePath() {
    // Clear any existing path
    _pathPoints.clear();
    _pathWaypoints.clear();
    _currentWaypointIndex = 0;
    
    std::vector<cv::Point> points = get_inters();

    // Add intermediate waypoints to create an interesting path
    // This is a simple example - you can make this more complex
    
    float mazeWidth = Constants::WALL_LENGTH;
    float mazeHeight = Constants::WALL_LENGTH;
    float imgWidth = get_img().cols;
    float imgHeight = get_img().rows;
    float xAdd = ((-mazeWidth/2)+Constants::WINDOW_WIDTH/2)/mazeWidth;
    float yAdd = ((-mazeHeight/2)+Constants::WINDOW_HEIGHT/2)/mazeHeight;
    
    for (int i = 0; i < points.size()-1; i++) {
        // Create a random numbers between 0.15 and 0.85 to ensure the target is not too close to the edges
        float randomX = xAdd + points[i].x/(imgWidth);

        // Create a random number between 0.2 and 0.8 to ensure the target is not too close to the edges
        float randomY = yAdd + points[i].y/(imgHeight);

        // Create random waypoints that are at a reasonable distance from each other
        // and avoid being too close to the edges

        float wpX = mazeWidth * randomX - Constants::WINDOW_WIDTH/2;
        float wpY = mazeHeight * randomY - Constants::WINDOW_HEIGHT/2;
        
        _pathWaypoints.push_back(Point3D(wpX, wpY, 0));
    }

    // Add target as the final waypoint
    _pathWaypoints.push_back(_targetPosition3D);
    
    // Create visual representation of the path
    createPathVisuals();
}

/**
 * @brief Creates visual elements to display the path
 */
void Maze::createPathVisuals() {
    // Initialize waypoint markers
    _waypointMarkers.clear();
    
    for (auto& waypoint : _pathWaypoints) {
        sf::CircleShape marker;
        marker.setRadius(5.f);
        marker.setFillColor(sf::Color(100, 100, 255, 150)); // Semi-transparent blue
        marker.setOrigin(5.f, 5.f);
        
        // Project to get screen position
        sf::Vector2f pos = waypoint.project();
        marker.setPosition(pos);
        
        _waypointMarkers.push_back(marker);
    }
    
    // Create line segments between waypoints
    _pathPoints.setPrimitiveType(sf::LineStrip);
    _pathPoints.resize(_pathWaypoints.size());
    
    for (size_t i = 0; i < _pathWaypoints.size(); i++) {
        sf::Vector2f pos = _pathWaypoints[i].project();
        _pathPoints[i].position = pos;
        _pathPoints[i].color = sf::Color(100, 100, 255, 120); // Semi-transparent blue
    }
}

/**
 * @brief Updates the visual representation of the path based on current tilt
 */
void Maze::updatePathProjection() {
    // Update waypoint markers
    for (size_t i = 0; i < _pathWaypoints.size(); i++) {
        Point3D rotated = _pathWaypoints[i];
        rotated.rotate(_tiltX, _tiltY);
        sf::Vector2f pos = rotated.project();
        
        if (i < _waypointMarkers.size()) {
            _waypointMarkers[i].setPosition(pos);
            
            // Scale based on depth
            float scale = Constants::FOCAL_LENGTH / (Constants::FOCAL_LENGTH + rotated._z);
            _waypointMarkers[i].setScale(scale, scale);
        }
        
        if (i < _pathPoints.getVertexCount()) {
            _pathPoints[i].position = pos;
        }
    }
}

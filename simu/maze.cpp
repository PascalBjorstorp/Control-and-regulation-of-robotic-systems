#include "maze.h"
#include <iostream>

Maze::Maze() {
    // Setup the background plane as a rectangular convex shape
    // This represents the flat surface of the maze that will be tilted in 3D space
    _background.setPointCount(4);
    _background.setFillColor(sf::Color(200, 200, 200));

    // Define the corners of the maze in 3D space
    // The maze is centered at (0,0,0) with dimensions slightly smaller than the window
    // This allows the maze to be fully visible when tilted
    float mazeWidth = Constants::WINDOW_WIDTH - 200;
    float mazeHeight = Constants::WINDOW_HEIGHT - 200;
    
    _backgroundCorners3D = {
        Point3D(-mazeWidth/2, -mazeHeight/2, 0),  // Top-left
        Point3D(mazeWidth/2, -mazeHeight/2, 0),   // Top-right
        Point3D(mazeWidth/2, mazeHeight/2, 0),    // Bottom-right
        Point3D(-mazeWidth/2, mazeHeight/2, 0)    // Bottom-left
    };

    // Create target
    makeTarget();

    // Create all the walls that make up the maze layout
    createMaze();

    // Initialize timing for motors
    _lastUpdateTime = _motorClock.getElapsedTime().asSeconds();

     // Initialize the 3D projection of all maze elements
    updateProjection();
}

void Maze::createMaze() {
    // Border walls definition to create the outer boundary of the maze
    // Parameters for Wall constructor: (x, y, length, isHorizontal)
    
    // Top horizontal border (slightly offset to properly connect with vertical borders)
    _walls.push_back(Wall(95, 100, Constants::WINDOW_WIDTH - 190, true));
    
    // Left vertical border
    _walls.push_back(Wall(100, 95, Constants::WINDOW_HEIGHT - 190, false));
    
    // Bottom horizontal border
    _walls.push_back(Wall(100, Constants::WINDOW_HEIGHT - 100, Constants::WINDOW_WIDTH - 195, true));
    
    // Right vertical border
    _walls.push_back(Wall(Constants::WINDOW_WIDTH - 100, 100, Constants::WINDOW_HEIGHT - 195, false));
}


void Maze::updateTilt() {
    // Calculate time since last update
    float currentTime = _motorClock.getElapsedTime().asSeconds();
    float deltaTime = currentTime - _lastUpdateTime;
    _lastUpdateTime = currentTime;
    
    // Ensure reasonable delta time (in case of debugging pauses)
    if (deltaTime > 0.1f) deltaTime = 0.016f; // cap at ~60fps
    
    // Calculate target tilt based on key inputs
    float targetTiltX = _motorX.getCurrentPosition();
    float targetTiltY = _motorY.getCurrentPosition();
    
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

    // Clamp target angles to prevent excessive tilting
    targetTiltX = std::clamp(targetTiltX, -Constants::MAX_TILT_ANGLE, Constants::MAX_TILT_ANGLE);
    targetTiltY = std::clamp(targetTiltY, -Constants::MAX_TILT_ANGLE, Constants::MAX_TILT_ANGLE);
    
    // Update motor target positions
    _motorX.setTargetPosition(targetTiltX);
    _motorY.setTargetPosition(targetTiltY);
    
    // Update motor physics
    _motorX.update(deltaTime);
    _motorY.update(deltaTime);
    
    // Get actual tilt angles from motors
    _tiltX = _motorX.getCurrentPosition();
    _tiltY = _motorY.getCurrentPosition();
    
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

bool Maze::checkCollisions(Ball& ball) {
    // Create a copy of the ball's 3D position and velocity
    Point3D ballPos = ball.getPosition3D();
    sf::Vector2f ballVel = ball.getVelocity();

    // Check if ball goes out of bounds
    float mazeWidth = Constants::WINDOW_WIDTH - 200;
    float mazeHeight = Constants::WINDOW_HEIGHT - 200;

    // Checks if the ball is outside the maze bounds
    if (std::abs(ballPos._x) > mazeWidth / 2 - Constants::BALL_RADIUS ||
        std::abs(ballPos._y) > mazeHeight / 2 - Constants::BALL_RADIUS) {

        // Bounce off the edge in the x-direction
        if (std::abs(ballPos._x) > mazeWidth / 2 - Constants::BALL_RADIUS) {
            float newVelx = ballVel.x * -0.5f;
            ball.setVelocity(sf::Vector2f(newVelx, ballVel.y));

            float newX = (mazeWidth / 2 - Constants::BALL_RADIUS) * (ballPos._x > 0 ? 1 : -1);
            ball.setPosition3D(Point3D(newX, ballPos._y, ballPos._z));
        }

        // Bounce off the edge in the y-direction
        if (std::abs(ballPos._y) > mazeHeight / 2 - Constants::BALL_RADIUS) {
            float newVely = ballVel.y * -0.5f;
            ball.setVelocity(sf::Vector2f(ballVel.x, newVely));

            float newY = (mazeHeight / 2 - Constants::BALL_RADIUS) * (ballPos._y > 0 ? 1 : -1);
            ball.setPosition3D(Point3D(ballPos._x, newY, ballPos._z));
        }
        return true;
    }

    return false;
}

// To be fine tuned later
void Maze::updateAutoNavigation(Ball& ball) {
    if (!_autoNavigationEnabled) return;

    float currentTime = _motorClock.getElapsedTime().asSeconds();
    float deltaTime = currentTime - _lastUpdateTime;
    _lastUpdateTime = currentTime;

    float timeSinceControlUpdate = currentTime - _lastControlUpdateTime;

    if (timeSinceControlUpdate >= 0.05f) {
        _lastControlUpdateTime = currentTime;

        Point3D currentTarget;
        if (_pathWaypoints.size() >= 2) {
            currentTarget = _pathWaypoints[1];
        } else {
            //makeTarget();
            return;
        }

        Point3D rotatedTarget = currentTarget;
        rotatedTarget.rotate(_tiltX, _tiltY);
        sf::Vector2f targetPos = rotatedTarget.project();

        sf::Vector2f ballPos = ball.getShape().getPosition();
        sf::Vector2f ballVel = ball.getVelocity();

        sf::Vector2f positionError = targetPos - ballPos;
        float distanceToTarget = std::sqrt(positionError.x * positionError.x + positionError.y * positionError.y);

        if (distanceToTarget < 10.0f) {
            //makeTarget();
            _lastDesiredTiltX = 0;
            _lastDesiredTiltY = 0;
            return;
        }

        sf::Vector2f normalizedPosError(0, 0);
        if (distanceToTarget > 0) {
            normalizedPosError = positionError / distanceToTarget;
        }

        float ballSpeed = std::sqrt(ballVel.x * ballVel.x + ballVel.y * ballVel.y);

        // Fixed PD gains for stable control
        float proportionalGain = 0.4f;  // Proportional gain
        float derivativeGain = 0.08f;      // Derivative gain

        // Calculate control terms
        // For X tilt (affects Y movement)
        float propTiltX = proportionalGain * normalizedPosError.y; 
        float derivTiltX = -derivativeGain * ballVel.y;
        
        // For Y tilt (affects X movement)
        float propTiltY = -proportionalGain * normalizedPosError.x;
        float derivTiltY = derivativeGain * ballVel.x;
        
        // Calculate desired tilt with smooth blending
        float desiredTiltX = propTiltX + derivTiltX;
        float desiredTiltY = propTiltY + derivTiltY;
        
        // Apply tilt angle limits
        float maxTiltAngle = 2.f;
        desiredTiltX = std::clamp(desiredTiltX, -maxTiltAngle, maxTiltAngle);
        desiredTiltY = std::clamp(desiredTiltY, -maxTiltAngle, maxTiltAngle);
        
        // Apply smoother transitions with exponential smoothing
        _lastDesiredTiltX = desiredTiltX;
        _lastDesiredTiltY = desiredTiltY;     
    }

    _motorX.setTargetPosition(_lastDesiredTiltX);
    _motorY.setTargetPosition(_lastDesiredTiltY);
    _motorX.update(deltaTime);
    _motorY.update(deltaTime);

    _tiltX = _motorX.getCurrentPosition();
    _tiltY = _motorY.getCurrentPosition();

    updateProjection();
}

void Maze::makeTarget(){    
    // Setup the target marker that the ball needs to reach
    // This is visualized as a green circle on the maze
    _targetMarker.setRadius(Constants::TARGET_RADIUS);
    _targetMarker.setFillColor(sf::Color(100, 100, 255, 150)); // Semi-transparent blue
    _targetMarker.setOrigin(Constants::TARGET_RADIUS, Constants::TARGET_RADIUS);

    // Create a random numbers between 0.15 and 0.85 to ensure the target is not too close to the edges
    float randomX = 0.15 + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (0.85 - 0.15)));

    // Create a random number between 0.2 and 0.8 to ensure the target is not too close to the edges
    float randomY = 0.2 + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (0.8 - 0.2)));

    // Set the 3D position of the target
    _targetPosition3D = Point3D(
        Constants::WINDOW_WIDTH * randomX - Constants::WINDOW_WIDTH/2,    // X position 
        Constants::WINDOW_HEIGHT * randomY - Constants::WINDOW_HEIGHT/2,  // Y position
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

    // Start from the default ball starting position
    Point3D startPos = Point3D(
        Constants::WINDOW_WIDTH / 4 - Constants::WINDOW_WIDTH/2, 
        Constants::WINDOW_HEIGHT / 4 - Constants::WINDOW_HEIGHT/2,
        0
    );
    
    // Add the start position as first waypoint
    _pathWaypoints.push_back(startPos);
    /*
    // Add intermediate waypoints to create an interesting path
    // This is a simple example - you can make this more complex
    const int numWaypoints = 3 + rand() % 3; // 3-5 waypoints
    
    float mazeWidth = Constants::WINDOW_WIDTH - 200;
    float mazeHeight = Constants::WINDOW_HEIGHT - 200;
    
    for (int i = 0; i < numWaypoints; i++) {
        // Create random waypoints that are at a reasonable distance from each other
        // and avoid being too close to the edges
        float wpX = (rand() % static_cast<int>(mazeWidth * 0.7f) - mazeWidth * 0.35f);
        float wpY = (rand() % static_cast<int>(mazeHeight * 0.7f) - mazeHeight * 0.35f);
        
        _pathWaypoints.push_back(Point3D(wpX, wpY, 0));
    }
    */
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
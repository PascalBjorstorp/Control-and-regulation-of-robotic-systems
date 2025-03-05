#include "maze.h"

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

    // Clamp tilt angles to prevent excessive tilting
    _tiltX = std::clamp(_tiltX, -Constants::MAX_TILT_ANGLE, Constants::MAX_TILT_ANGLE);
    _tiltY = std::clamp(_tiltY, -Constants::MAX_TILT_ANGLE, Constants::MAX_TILT_ANGLE);

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

void Maze::updateAutoNavigation(Ball& ball) {
    // Exit early if auto-navigation is disabled
    if (!_autoNavigationEnabled) return;

    // Determine which waypoint to target
    Point3D currentTarget;
    if (_currentWaypointIndex < _pathWaypoints.size()) {
        currentTarget = _pathWaypoints[_currentWaypointIndex];
    } else {
        makeTarget();
    }

    // Get the current projected positions
    Point3D rotatedTarget = currentTarget;
    rotatedTarget.rotate(_tiltX, _tiltY);
    sf::Vector2f targetPos = rotatedTarget.project();

    // Get the current ball position in 2D screen space
    sf::Vector2f ballPos = ball.getShape().getPosition();

    // Get the current ball velocity
    sf::Vector2f ballVel = ball.getVelocity();

    // Calculate position error vector (Proportional term)
    // This represents both distance and direction to the target
    sf::Vector2f positionError = targetPos - ballPos;
    float distanceToTarget = std::sqrt(positionError.x * positionError.x + positionError.y * positionError.y);

    // Check if we've reached this waypoint
    if (distanceToTarget < 5.0f) {
        _currentWaypointIndex++;
        
        // If we've reached the end, cycle back to the first waypoint
        if (_currentWaypointIndex > _pathWaypoints.size()) {
            _currentWaypointIndex = 0;
        }
        
        return; // We'll update targeting on next frame
    }

    // Normalize the error vector to get pure direction
    // This separates direction from distance for control purposes
    sf::Vector2f normalizedPosError = positionError;
    if (distanceToTarget > 0) {
        normalizedPosError /= distanceToTarget;
    }

    // Calculate current speed (magnitude of velocity)
    // Used to adjust control response based on how fast the ball is moving
    float currentSpeed = std::sqrt(ballVel.x * ballVel.x + ballVel.y * ballVel.y);

    // Define PD controller gains
    // These values determine the control behavior:
    float Kp = 1.f; // Proportional gain - how strongly to respond to position error
                     // Higher values make the ball move more directly toward the target
                     // but may cause overshooting
    
    float Kd = 0.3f; // Derivative gain - how strongly to dampen based on velocity
                     // Higher values provide more braking/damping effect
                     // which reduces oscillation but may make movement sluggish

    // Scale proportional gain based on distance to target
    // When far away: full strength (more aggressive steering)
    // When close: reduced strength (gentler approach)
    float distanceFactor = std::min(distanceToTarget / 100.0f, 1.f);
    Kp *= distanceFactor;

    // Calculate derivative term (negative velocity - we want to counter current motion)
    sf::Vector2f velocityTerm = -ballVel;

    // Calculate desired tilt angles based on the PD controller components
    // The signs are important here to create the correct tilt direction:
    // - For Y axis: negative error.x creates positive tilt to roll ball right
    // - For X axis: positive error.y creates positive tilt to roll ball down
    float desiredTiltY = (-normalizedPosError.x * Kp - velocityTerm.x * Kd) * Constants::MAX_TILT_ANGLE;
    float desiredTiltX = (normalizedPosError.y * Kp + velocityTerm.y * Kd) * Constants::MAX_TILT_ANGLE;

    // Apply a low-pass filter for smooth tilt transitions
    // Alpha controls how quickly the actual tilt approaches the desired tilt
    // Lower values create smoother, more gradual transitions
    float alpha = 0.1f;
    _tiltY = _tiltY * (1 - alpha) + desiredTiltY * alpha;
    _tiltX = _tiltX * (1 - alpha) + desiredTiltX * alpha;

    // Ensure tilt angles remain within allowed limits
    _tiltX = std::clamp(_tiltX, -Constants::MAX_TILT_ANGLE, Constants::MAX_TILT_ANGLE);
    _tiltY = std::clamp(_tiltY, -Constants::MAX_TILT_ANGLE, Constants::MAX_TILT_ANGLE);

    // Update the visual projection based on new tilt angles
    // This ensures all maze elements (walls, background, target) reflect the new tilt
    updateProjection();

    if(distanceToTarget < 0.5) {
        makeTarget();
    }
}

void Maze::makeTarget(){    
    // Setup the target marker that the ball needs to reach
    // This is visualized as a green circle on the maze
    _targetMarker.setRadius(Constants::TARGET_RADIUS);
    _targetMarker.setFillColor(sf::Color::Green);
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
#ifndef MAZE_H
#define MAZE_H

// Maze class
#include "ball.h"
#include "linedetecter.h"
#include "wall.h"
#include "motor.h"
#include "Constants.h"
#include "uartcom.h"
#include <SFML/Graphics.hpp>
#include <vector>

class Maze : public lineDetecter{
    // Maze variables
    std::vector<Wall> _walls;
    sf::ConvexShape _background;
    std::vector<Point3D> _backgroundCorners3D;
    UARTcom uart;

    // Target variables
    sf::CircleShape _targetMarker;
    sf::Vector2f _targetPosition;
    Point3D _targetPosition3D;

    bool _autoNavigationEnabled = false;
    float _tiltX = 0.f;
    float _tiltY = 0.f;

    bool _pointHit = false;

    // Path-related members
    std::vector<Point3D> _pathWaypoints;
    std::vector<sf::CircleShape> _waypointMarkers;
    sf::VertexArray _pathPoints;
    size_t _currentWaypointIndex = 0;

    // Motors for the tilt system
    Motor _motorX;  // Controls tilt around X axis
    Motor _motorY;  // Controls tilt around Y axis
    
    // Timing variables
    sf::Clock _motorClock;
    float _lastUpdateTime = 0.0f;

    float _lastControlUpdateTime = 0.0f;  // Tracks time of last control update
    const float _controlUpdateInterval = 0.05f;  // 200ms between control updates
    float _lastDesiredTiltX = 0.0f;  // Store last calculated tilt values
    float _lastDesiredTiltY = 0.0f;

    // PD controller constants
    const float _kp = 3.f;           // Proportional gain
    const float _kd = 3.f;           // Derivative gain

public:

    /**
     * Constructor for the Maze class.
     * 
     * Initializes the maze by setting up:
     * 1. The background plane on which the maze sits
     * 2. The 3D coordinate system for the maze
     * 3. The target marker that the ball needs to reach
     * 4. All maze walls and their initial positions
     */
    Maze(cv::Mat img);

    /**
     * Creates the walls that form the maze layout.
     * 
     * This method defines the positions and dimensions of all walls in the maze,
     * including the border walls and internal passages.
     * Each wall is added to the _walls vector for rendering and collision detection.
     */
    void createMaze();

    /**
     * Updates the tilt angles of the maze based on keyboard input.
     * 
     * This method handles keyboard controls for tilting the maze:
     * - Arrow keys provide primary control with full tilt speed
     * - WASD keys provide secondary control at half speed for finer adjustments
     * 
     * After updating the tilt angles, the method ensures they stay within
     * allowed limits and updates the visual projection of all maze elements.
     */
    void updateTilt();

    /**
     * Updates the visual projection of all maze elements based on current tilt angles.
     * 
     * This method:
     * 1. Projects the background corners from 3D to 2D with rotation
     * 2. Updates all wall projections using their 3D coordinates
     * 3. Updates the target marker position with proper perspective
     * 
     * This creates the visual effect of the entire maze tilting in 3D space.
     */
    void updateProjection();

    /**
     * Toggles the auto-navigation mode for the maze.
     *
     * This method switches between manual and automatic maze control.
     * When auto-navigation is enabled, the maze will automatically tilt
     * to guide the ball toward the target marker.
    */
    void toggleAutoNavigation();

    /**
     * Implements a PD (Proportional-Derivative) controller for automatic ball navigation.
     * 
     * This method automatically tilts the maze to guide the ball toward the target marker.
     * It uses a control system approach where:
     * - The "proportional" term steers based on position error (distance to target)
     * - The "derivative" term provides damping based on current velocity
     * 
     * The combination creates smooth navigation that:
     * 1. Initially accelerates toward the target
     * 2. Gradually decelerates as it approaches
     * 3. Minimizes oscillations around the target
     * 
     * @param ball Reference to the Ball object being controlled
     */
    void updateAutoNavigation(Ball& ball);

    /**
     * Creates a target marker for the ball to reach.
     * 
     * This method generates a random target position within the maze bounds
     * and projects it to 3D space for proper perspective projection.
     */
    void makeTarget();
    void generatePath();
    void createPathVisuals();
    void updatePathProjection();

    // Getter functions
    bool getAutoNavigationEnabled() const { return _autoNavigationEnabled; }
    std::vector<Wall>& getWalls() { return _walls; }
    sf::ConvexShape& getBackground() { return _background; }
    sf::CircleShape& getTargetMarker() { return _targetMarker; }
    sf::Vector2f getTargetPosition() const { return _targetPosition; }
    Point3D getTargetPosition3D() const { return _targetPosition3D; }
    float getTiltX() const { return _tiltX; }
    float getTiltY() const { return _tiltY; }
    const sf::VertexArray& getPath() const { return _pathPoints; }
    const std::vector<sf::CircleShape>& getWaypointMarkers() const { return _waypointMarkers; }

    // Setter functions
    void setTiltX(float tiltX) { _tiltX = tiltX; }
    void setTiltY(float tiltY) { _tiltY = tiltY; }
};

#endif // MAZE_H

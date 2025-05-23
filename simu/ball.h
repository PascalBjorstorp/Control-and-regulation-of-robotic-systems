#ifndef BALL_H
#define BALL_H

// Ball class
#include "point3d.h"
#include "Constants.h"
#include <SFML/Graphics.hpp>
#include <cmath>

class Ball {
    Point3D position3D;
    sf::CircleShape shape;
    sf::Vector2f velocity;

public:
    /**
     * Constructor for the Ball class.
     * 
     * Initializes a ball at the specified window coordinates and sets up its visual representation.
     * The position is converted from window coordinates to centered 3D coordinates for proper physics.
     * 
     * @param x Initial x-coordinate in window space
     * @param y Initial y-coordinate in window space
     */
    Ball(float x, float y);

    /**
     * Updates the ball's position and appearance based on physics and maze tilt.
     * 
     * This method implements a physics model where:
     * 1. Tilt angles create an acceleration vector (simulating gravity)
     * 2. The acceleration affects velocity with friction applied
     * 3. Position is updated based on velocity
     * 4. Visual representation is updated with proper 3D perspective
     * 
     * @param tiltX Tilt angle around X-axis in degrees
     * @param tiltY Tilt angle around Y-axis in degrees
     */
    void update(float tiltX, float tiltY);

    /**
     * Checks for collisions between the ball and the maze walls.
     * 
     * This method detects collisions between the ball and outer walls.
     * If a collision is detected, the ball's velocity is adjusted to simulate
     * a bounce effect, and the method returns true to indicate a collision.
     * 
     * @param ball Reference to the ball object for collision detection
     * @return True if a collision occurred, false otherwise
     */
    bool checkCollisions();

    /**
     * Resets the ball to a specified position with zero velocity.
     * 
     * Used when restarting the game or after the ball falls into a hole.
     * Converts window coordinates to centered 3D coordinates and updates the visual representation.
     * 
     * @param x New x-coordinate in window space
     * @param y New y-coordinate in window space
     */
    void reset(float x, float y);

    // Getter functions
    const sf::CircleShape& getShape() const { return shape; }
    const sf::Vector2f& getVelocity() const { return velocity; }
    const Point3D& getPosition3D() const { return position3D; }

    // Setter functions
    void setVelocity(sf::Vector2f velocity) { this->velocity = velocity; }
    void setPosition3D(Point3D position3D) { this->position3D = position3D; }
};

#endif // BALL_H

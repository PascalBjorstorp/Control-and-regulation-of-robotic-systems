#ifndef WALL_H
#define WALL_H

// Wall class
#include "point3d.h"
#include <SFML/Graphics.hpp>

/**
 * @class Wall
 * @brief Represents a wall in the 3D maze.
 * 
 * The Wall class manages both the 3D representation of a wall segment and its
 * 2D projection for rendering. Walls can be either horizontal or vertical, and
 * are rendered as convex shapes that update their appearance as the maze tilts.
 */
class Wall {
    sf::ConvexShape shape;                  // 2D shape used for rendering
    std::vector<Point3D> originalCorners3D; // Original 3D positions (before rotation)
    std::vector<Point3D> corners3D;         // Current 3D positions (after rotation)
    bool isHorizontal;                      // Whether wall is horizontal or vertical
public:

    /**
     * @brief Constructs a new Wall object with specified position, length, and orientation.
     * 
     * This constructor:
     * 1. Creates a rectangular wall with the specified dimensions
     * 2. Converts from window coordinates to centered 3D coordinates
     * 3. Sets up the 3D corners of the wall based on orientation
     * 4. Initializes the 2D projection of the wall
     * 
     * @param x X-coordinate of the wall's starting point in screen space
     * @param y Y-coordinate of the wall's starting point in screen space
     * @param length Length of the wall
     * @param horizontal If true, wall extends along the X-axis; if false, along the Y-axis
     */
    Wall(float x, float y, float length, bool horizontal);

    /**
     * @brief Updates the wall's 2D projection based on maze tilt.
     * 
     * This method:
     * 1. Resets the 3D corners to their original positions
     * 2. Applies rotation to all corners based on the tilt angles
     * 3. Projects each rotated 3D point to 2D screen coordinates
     * 4. Updates the shape's vertices for rendering
     * 
     * @param angleX Tilt angle around the X-axis in degrees
     * @param angleY Tilt angle around the Y-axis in degrees
     */
    void updateProjection(float angleX, float angleY);

    // Getter functions
    sf::ConvexShape& getShape() { return shape; }
};

#endif // WALL_H

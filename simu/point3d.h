#ifndef POINT3D_H
#define POINT3D_H

// 3D Point structure
#include <SFML/Graphics.hpp>
#include <cmath>
#include "Constants.h"

class Point3D
{

public:
    float _x, _y, _z;

    /**
     * Constructor for a 3D point in space
     * 
     * @param x X-coordinate in 3D space
     * @param y Y-coordinate in 3D space
     * @param z Z-coordinate in 3D space (depth)
     */
    Point3D(float x = 0, float y = 0, float z = 0);

    /**
     * Projects a 3D point onto a 2D screen using perspective projection
     * 
     * This method implements a simple perspective projection where:
     * - Objects farther away (larger positive Z) appear smaller
     * - Objects closer to the viewer (smaller or negative Z) appear larger
     * 
     * The scale factor creates the perspective effect:
     * - When Z = 0, scale = 1 (no scaling)
     * - When Z > 0 (away from viewer), scale < 1 (smaller)
     * - When Z < 0 (toward viewer), scale > 1 (larger)
     * 
     * Half the window dimensions is added to center the projection in the middle of the window.
     * 
     * @return 2D coordinates for the projected point on screen
     */
    sf::Vector2f project();

    /**
     * Rotates the point around the X and Y axes
     * 
     * This method applies standard 3D rotation matrices to the point:
     * 1. First rotates around X-axis (affects Y and Z coordinates)
     * 2. Then rotates around Y-axis (affects X and Z coordinates)
     * 
     * The order of rotations is important because 3D rotations are not commutative.
     * We apply X rotation first, then Y rotation to match intuitive tilting behavior.
     * 
     * For X-axis rotation:
     * - Y' = Y*cos(θ) - Z*sin(θ)
     * - Z' = Y*sin(θ) + Z*cos(θ)
     * 
     * For Y-axis rotation:
     * - X' = X*cos(θ) + Z*sin(θ)
     * - Z' = -X*sin(θ) + Z*cos(θ)
     * 
     * Temporary variables are used to preserve original values during the calculation.
     * 
     * @param angleX Rotation angle around X-axis in degrees
     * @param angleY Rotation angle around Y-axis in degrees
     */
    void rotate(float angleX, float angleY);
};

#endif // POINT3D_H

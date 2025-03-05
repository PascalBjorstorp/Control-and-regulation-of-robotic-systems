#include "wall.h"

Wall::Wall(float x, float y, float length, bool horizontal) {
    isHorizontal = horizontal;
    shape.setPointCount(4);
    shape.setFillColor(sf::Color(54, 56, 46));

    // Store original 3D coordinates for the wall
    float halfWidth = Constants::WALL_WIDTH / 2;
    if (horizontal) {
        // Horizontal wall
        originalCorners3D = {
            Point3D(x - Constants::WINDOW_WIDTH/2, y - halfWidth - Constants::WINDOW_HEIGHT/2, 0),
            Point3D(x + length - Constants::WINDOW_WIDTH/2, y - halfWidth - Constants::WINDOW_HEIGHT/2, 0),
            Point3D(x + length - Constants::WINDOW_WIDTH/2, y + halfWidth - Constants::WINDOW_HEIGHT/2, 0),
            Point3D(x - Constants::WINDOW_WIDTH/2, y + halfWidth - Constants::WINDOW_HEIGHT/2, 0)
        };
    } else {
        // Vertical wall
        originalCorners3D = {
            Point3D(x - halfWidth - Constants::WINDOW_WIDTH/2, y - Constants::WINDOW_HEIGHT/2, 0),
            Point3D(x + halfWidth - Constants::WINDOW_WIDTH/2, y - Constants::WINDOW_HEIGHT/2, 0),
            Point3D(x + halfWidth - Constants::WINDOW_WIDTH/2, y + length - Constants::WINDOW_HEIGHT/2, 0),
            Point3D(x - halfWidth - Constants::WINDOW_WIDTH/2, y + length - Constants::WINDOW_HEIGHT/2, 0)
        };
    }

    // Copy to current corners
    corners3D = originalCorners3D;

    // Initialize shape with projected points at zero tilt
    updateProjection(0, 0);
}

void Wall::updateProjection(float angleX, float angleY) {
    // Copy original corners and apply rotation to each corner
    for (size_t i = 0; i < originalCorners3D.size(); i++) {
        corners3D[i] = originalCorners3D[i];              // Reset to original position
        corners3D[i].rotate(angleX, angleY);              // Apply rotation based on maze tilt
        sf::Vector2f projected = corners3D[i].project();  // Project to 2D
        shape.setPoint(i, projected);                     // Update the shape's vertices
    }
}

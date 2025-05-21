#include "point3d.h"

Point3D::Point3D(float x, float y, float z) : _x(x), _y(y), _z(z) {}

sf::Vector2f Point3D::project() {
    float scale = Constants::FOCAL_LENGTH / (Constants::FOCAL_LENGTH + _z);
    return sf::Vector2f(Constants::WINDOW_WIDTH/2 + _x * scale, Constants::WINDOW_HEIGHT/2 + _y * scale);
}

void Point3D::rotate(float angleX, float angleY) {
    float radX = angleX * M_PI / 180.0f;
    float radY = angleY * M_PI / 180.0f;

    // Rotate around X-axis
    float tempY = _y;
    float tempZ = _z;
    _y = tempY * cos(radX) - tempZ * sin(radX);
    _z = tempY * sin(radX) + tempZ * cos(radX);

    // Rotate around Y-axis
    float tempX = _x;
    tempZ = _z;
    _x = tempX * cos(radY) + tempZ * sin(radY);
    _z = -tempX * sin(radY) + tempZ * cos(radY);
}

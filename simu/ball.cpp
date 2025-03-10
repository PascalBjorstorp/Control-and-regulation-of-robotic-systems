#include "ball.h"

Ball::Ball(float x, float y) {
    position3D = Point3D(x - Constants::WINDOW_WIDTH/2, y - Constants::WINDOW_HEIGHT/2, 0);
    shape.setRadius(Constants::BALL_RADIUS);
    shape.setFillColor(sf::Color::Red);
    shape.setOrigin(Constants::BALL_RADIUS, Constants::BALL_RADIUS);

    // Initial position
    sf::Vector2f projected = position3D.project();
    shape.setPosition(projected);

    velocity = sf::Vector2f(0, 0);
}

void Ball::update(float tiltX, float tiltY) {
    // Convert tilt angles to radians
    float radX = tiltX * (M_PI / 180.f);
    float radY = tiltY * (M_PI / 180.f);

    // Compute acceleration based on tilt
    // Negative sine for Y-axis makes the ball roll in the intuitive direction
    // (e.g., right tilt makes the ball roll right)
    sf::Vector2f acceleration(-std::sin(radY) * Constants::GRAVITY, std::sin(radX) * Constants::GRAVITY);

    // Apply acceleration and friction
    velocity += acceleration;
    velocity *= Constants::FRICTION;

    // Update 3D position (z remains 0 as ball stays on the surface)
    position3D._x += velocity.x;
    position3D._y += velocity.y;

    // Create a rotated copy for projection
    Point3D rotatedPosition = position3D;
    rotatedPosition.rotate(tiltX, tiltY);

    // Project to 2D and update shape position
    sf::Vector2f projected = rotatedPosition.project();
    shape.setPosition(projected);

    // Apply scaling based on z position for depth effect
    float scale = Constants::FOCAL_LENGTH / (Constants::FOCAL_LENGTH + rotatedPosition._z);
    shape.setScale(scale, scale);
}

void Ball::reset(float x, float y) {
    position3D = Point3D(x - Constants::WINDOW_WIDTH/2, y - Constants::WINDOW_HEIGHT/2, 0);
    velocity = sf::Vector2f(0, 0);
    sf::Vector2f projected = position3D.project();
    shape.setPosition(projected);
}

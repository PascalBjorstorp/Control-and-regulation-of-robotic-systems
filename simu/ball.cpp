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
    velocity += acceleration * Constants::TIME_STEP;

    // Update 3D position (z remains 0 as ball stays on the surface)
    position3D._x += velocity.x * Constants::TIME_STEP;
    position3D._y += velocity.y * Constants::TIME_STEP;;

    // Create a rotated copy for projection
    Point3D rotatedPosition = position3D;
    rotatedPosition.rotate(tiltX, tiltY);

    // Project to 2D and update shape position
    sf::Vector2f projected = rotatedPosition.project();
    shape.setPosition(projected);

    // Apply scaling based on z position for depth effect
    float scale = Constants::FOCAL_LENGTH / (Constants::FOCAL_LENGTH + rotatedPosition._z);
    shape.setScale(scale, scale);

    checkCollisions();
}

bool Ball::checkCollisions() {
    // Create a copy of the ball's 3D position and velocity
    Point3D ballPos = getPosition3D();
    sf::Vector2f ballVel = getVelocity();

    // Check if ball goes out of bounds
    float mazeWidth = Constants::WALL_LENGTH;
    float mazeHeight = Constants::WALL_LENGTH;

    // Checks if the ball is outside the maze bounds
    if (std::abs(ballPos._x) > mazeWidth / 2 - Constants::BALL_RADIUS ||
        std::abs(ballPos._y) > mazeHeight / 2 - Constants::BALL_RADIUS) {

        // Bounce off the edge in the x-direction
        if (std::abs(ballPos._x) > mazeWidth / 2 - Constants::BALL_RADIUS) {
            float newVelx = ballVel.x * -0.5f;
            velocity = (sf::Vector2f(newVelx, ballVel.y));

            float newX = (mazeWidth / 2 - Constants::BALL_RADIUS) * (ballPos._x > 0 ? 1 : -1);
            position3D = (Point3D(newX, ballPos._y, ballPos._z));
        }

        // Bounce off the edge in the y-direction
        if (std::abs(ballPos._y) > mazeHeight / 2 - Constants::BALL_RADIUS) {
            float newVely = ballVel.y * -0.5f;
            velocity = (sf::Vector2f(ballVel.x, newVely));

            float newY = (mazeHeight / 2 - Constants::BALL_RADIUS) * (ballPos._y > 0 ? 1 : -1);
            position3D = (Point3D(ballPos._x, newY, ballPos._z));
        }
        return true;
    }

    return false;
}

void Ball::reset(float x, float y) {
    position3D = Point3D(x - Constants::WINDOW_WIDTH/2, y - Constants::WINDOW_HEIGHT/2, 0);
    velocity = sf::Vector2f(0, 0);
    sf::Vector2f projected = position3D.project();
    shape.setPosition(projected);
}

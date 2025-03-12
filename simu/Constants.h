#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace Constants {
    // Window settings
    inline constexpr int WINDOW_WIDTH = 1200;
    inline constexpr int WINDOW_HEIGHT = 1000;
    inline constexpr int FOCAL_LENGTH = 600;

    // Ball settings
    inline constexpr float BALL_RADIUS = 5.f;
    inline constexpr float FRICTION = 0.98f;
    inline constexpr float GRAVITY = 9.82f;

    // Maze settings
    inline constexpr float WALL_WIDTH = 10.f;
    inline constexpr float MAX_TILT_ANGLE = 10.f;
    inline constexpr float TILT_SPEED = 0.5f;
    inline constexpr float AUTO_CONTROL_STRENGTH = 0.2f;
    inline constexpr float TARGET_RADIUS = 5.f;
}

#endif // CONSTANTS_H

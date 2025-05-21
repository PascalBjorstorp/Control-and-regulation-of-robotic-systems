#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace Constants {
    // Window settings
    inline constexpr int WINDOW_WIDTH = 1000;
    inline constexpr int WINDOW_HEIGHT = 1000;
    inline constexpr int FOCAL_LENGTH = 500;

    // Ball settings
    inline constexpr float BALL_RADIUS = 64.f;
    inline constexpr float FRICTION = 1.f;
    inline constexpr float GRAVITY = 9.82f;
    inline constexpr float TIME_STEP = 0.030f;

    // Maze settings
    inline constexpr float WALL_WIDTH = 10.f;
    inline constexpr float WALL_X = 50.f;
    inline constexpr float WALL_Y = 50.f;
    inline constexpr float WALL_LENGTH = 880.f;
    inline constexpr float MAX_TILT_ANGLE = 10.f;
    inline constexpr float TILT_SPEED = 0.5f;
    inline constexpr float AUTO_CONTROL_STRENGTH = 0.2f;
    inline constexpr float TARGET_RADIUS = 5.f;
}

#endif // CONSTANTS_H

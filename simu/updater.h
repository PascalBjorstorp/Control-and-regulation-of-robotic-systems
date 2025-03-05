#ifndef UPDATER_H
#define UPDATER_H

#include <sstream>
#include <iomanip>
#include <iostream>
#include <SFML/Graphics.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

#include "ball.h"
#include "maze.h"
#include "wall.h"

class Updater
{
    sf::RenderWindow _window;
    sf::Font _font;
    sf::Text _angleText;
    Ball _ball;
    Maze _maze;
public:
    /**
     * Constructor for the Updater class.
     * 
     * Initializes the window, font, text, ball, and maze objects.
     * Sets the window framerate limit to 60 FPS.
     */
    Updater();

    /**
     * Main update loop for the game.
     * 
     * This method handles all game logic, including:
     * - Event handling for window closure and ball reset
     * - Maze tilt updates based on manual or automatic control
     * - Ball physics updates based on tilt angles
     * - Collision detection between the ball and maze walls
     * - Display of tilt angles and auto-navigation status
     * - Rendering of all game elements (maze, walls, target, ball)
     */
    void update();
};

#endif // UPDATER_H

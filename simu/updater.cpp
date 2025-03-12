#include "updater.h"

Updater::Updater(): _window(sf::VideoMode(Constants::WINDOW_WIDTH, Constants::WINDOW_HEIGHT), "Maze Game"), _ball(Constants::WINDOW_WIDTH / 4, Constants::WINDOW_HEIGHT / 4) {
    _window.setFramerateLimit(60);

    if (!_font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")) {
        // Handle font loading failure - program will continue without text
        std::cout << "Warning: Failed to load font. Angle display will not be shown." << std::endl;
    }

    // Create text for displaying angles
    _angleText.setFont(_font);
    _angleText.setCharacterSize(18);
    _angleText.setFillColor(sf::Color::Black);
    _angleText.setPosition(Constants::WINDOW_WIDTH - 150, 10);
}

void Updater::update(){
    while (_window.isOpen()) {
        sf::Event event;
        while (_window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                _window.close();

            // If the 'R' key is pressed, reset the ball position
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::R) {
                // Reset ball position
                _ball.reset(Constants::WINDOW_WIDTH / 4, Constants::WINDOW_HEIGHT / 4);
            }
        }

        // Update maze tilt
        if (_maze.getAutoNavigationEnabled()) {
            _maze.updateAutoNavigation(_ball);
        } else {
            _maze.updateTilt();
        }

        // Apply the new tilt angles to the ball
        _ball.update(_maze.getTiltX(), _maze.getTiltY());

        // Check for collisions with maze walls
        _maze.checkCollisions(_ball);

        // Display debugger information in the right corner
        std::stringstream ss;
        ss << "X Tilt: " << std::fixed << std::setprecision(1) << _maze.getTiltX()
           << "\nY Tilt: " << std::fixed << std::setprecision(1) << _maze.getTiltY()
           << "\nX Motor Speed: " << std::fixed << std::setprecision(1) << _maze.getMotorXSpeed() 
           << "\nY Motor Speed: " << std::fixed << std::setprecision(1) << _maze.getMotorYSpeed()
           << "\nVelocity x: " << std::fixed << std::setprecision(1) << _ball.getVelocity().x
           << "\nVelocity y: " << std::fixed << std::setprecision(1) << _ball.getVelocity().y
           << "\nAuto: " << (_maze.getAutoNavigationEnabled() ? "On" : "Off");
        _angleText.setString(ss.str());

        // Clear the window
        _window.clear(sf::Color::White);

        // Draw all game elements
        _window.draw(_maze.getBackground());   // Draw background first
        _window.draw(_maze.getPath());         // Draw path
        for (const auto& marker : _maze.getWaypointMarkers()) {
            _window.draw(marker);              // Draw waypoint markers
        }
        _window.draw(_angleText);              // Draw angle text on top 
        for (Wall& wall : _maze.getWalls()) {
            _window.draw(wall.getShape());     // Draw walls
        }
        _window.draw(_maze.getTargetMarker()); // Draw target marker
        _window.draw(_ball.getShape());        // Draw ball

        // Display the window contents
        _window.display();
    }
}

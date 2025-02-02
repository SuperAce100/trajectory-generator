# Real-time Mobile Robot Motion Planning Using PID Control and Potential Field-Based Obstacle Avoidance

A real-time interactive simulation demonstrating autonomous robot path planning with obstacle avoidance using gradient descent optimization and artificial potential field methods.

## Features

- **Real-time Path Planning**: Dynamic path generation considering kinematic constraints
- **Obstacle Avoidance**: Interactive obstacle placement with potential field-based avoidance
- **Smooth Motion Control**: Velocity and acceleration-based motion profiles
- **Interactive Controls**: Direct robot control and path planning through user input
- **Visual Feedback**: Real-time visualization of robot state, paths, and obstacles

## Controls

### Robot Control
- **Arrow Keys**: Move robot manually
- **A/D Keys**: Rotate robot
- **Space/Button**: Navigate to predefined target

### Path Planning
- **Click**: Set target position
- **Second Click**: Set target orientation
- **Shift + Click**: Add obstacle
- **O Key**: Toggle obstacle field visualization

## Technical Details

### Path Planning Algorithm
The simulation uses a gradient descent optimization algorithm to generate smooth, obstacle-avoiding paths. The path is represented as a polynomial curve with constraints for:
- Initial position and velocity
- Final position and approach angle
- Kinematic feasibility
- Obstacle avoidance

### Robot Kinematics
The robot model includes:
- Maximum velocity and acceleration limits
- Turning radius constraints
- Smooth velocity profiles
- PID-based heading control

### Implementation
- Built using p5.js for visualization
- Pure JavaScript implementation
- Real-time path optimization
- Modular code structure

## Setup

1. Clone the repository
2. Open `index.html` in a modern web browser
3. No additional dependencies or build steps required

## How It Works

1. **Path Generation**
   - User clicks define target position and orientation
   - System generates initial path considering robot state
   - Path is optimized for length and obstacle avoidance

2. **Path Following**
   - Robot follows generated path using PID control
   - Velocity and acceleration limits ensure smooth motion
   - Real-time path updates handle dynamic obstacles

3. **Obstacle Avoidance**
   - Obstacles create potential fields
   - Path optimization considers obstacle proximity
   - Dynamic replanning available for moving obstacles

## Future Improvements
- Add more complex robot kinematics models
- Implement A* pathfinding
- Add time-varying constraints
- Include more realistic sensor models
- Enhance path optimization with dynamic constraints


# 2D Robotic Arm Simulator with Interactive Trajectories

This project is an interactive 2D robotic arm simulator built using Python and Pygame. It allows users to control a 2-link planar robotic arm, visualize its workspace, and command it to follow different types of trajectories using inverse kinematics and spline-based path generation.

The simulator supports manual control via a virtual joystick as well as automated motion through line, arc, and multi-point trajectories.

---

## Features

- 2-DOF planar robotic arm with forward and inverse kinematics
- Real-time joystick control of joint angles
- Visualization of reachable workspace
- Trajectory generation modes:
  - 2-Point Linear Path
  - 3-Point Quadratic Bezier Arc
  - Multi-Point Catmull-Rom Spline Path
- Smooth animated motion along generated trajectories
- End-effector trajectory visualization
- Interactive UI with buttons and control points

---

## Demo Controls

### Buttons

Set 2-Point Line – Click two points to create a straight-line trajectory  
Set 3-Point Arc – Click three points to create a curved arc trajectory  
Multi-Point Traj – Click multiple points, then click again to execute  
Clear Workspace – Clears trajectories and control points  

### Mouse

Click to place trajectory control points.  
Drag the joystick knob to manually move joints.  
Release mouse to reset joystick.

---

## Technical Overview

### Kinematics
Forward kinematics computes joint positions using trigonometry.  
Inverse kinematics computes joint angles from desired end-effector position.

### Trajectory Planning
Linear interpolation for 2-point motion.  
Quadratic Bezier curve constrained through midpoint for 3-point arcs.  
Catmull-Rom spline for smooth multi-point trajectories.

### Visualization
Workspace is precomputed by sweeping joint angles.  
Arm links, joints, trajectory, and control points are rendered dynamically.

---

## Requirements

Python 3.8 or higher  
pygame

Install dependencies:

pip install pygame

---

## Running the Simulator

python main.py

---

## Project Structure

.
├── main.py
└── README.md

---

## Use Cases

Learning robot kinematics and inverse kinematics  
Visualizing trajectory planning algorithms  
Prototyping motion planning logic  
Educational demonstrations

---

## Future Improvements

Add velocity and acceleration profiles  
Obstacle avoidance  
Export trajectories for physical robots  
ROS interface for hardware integration

---

## Author

Tejas Sriganesh  
GitHub: https://github.com/tejasms03  
LinkedIn: https://linkedin.com/in/tejas-sriganesh  
Email: TM4715@nyu.edu  

---

## License

MIT License — see LICENSE file for details.

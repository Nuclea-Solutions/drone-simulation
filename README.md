# Drone Simulation with PID Flight Controller

Advanced drone simulation featuring realistic flight dynamics, collision detection, and real-time telemetry visualization.

## Features

🚁 **Realistic Flight Controller**
- Cascaded PID control loops (rate → attitude → position)
- Multiple flight modes: Stabilize, Altitude Hold, Position Hold
- Conservative gains to maintain horizontal flight attitude

🎯 **Advanced Controls**
- Real-time sliders for target positioning
- Direct coordinate input fields
- Position hold buttons (current location lock)
- One-click simulation reset

💥 **Physics-Based Collision Detection**
- Real MuJoCo collision detection with obstacle geometry
- Crash behavior with PID disable and gravity tumbling
- Collision depth measurement and logging
- Emergency disarm on severe impacts

📊 **Comprehensive Telemetry**
- Real-time position display (X, Y, Z coordinates)
- Speed monitoring (individual axes + total speed)
- Distance to target calculation
- Attitude angle limits to prevent excessive tilt

🏗️ **3D Environment**
- Multiple obstacle types: towers, boxes, spheres, pillars
- Room boundaries with walls and ceiling
- Visual and physics collision matching

## Files

### Core Simulation
- `drone_collision_course.py` - Main simulation with all features
- `flight_controller.py` - PID-based flight controller implementation
- `drone_with_obstacles.xml` - MuJoCo world with physics obstacles

### Additional Examples
- `drone_simple_with_pid.py` - Basic PID flight controller demo
- `drone_simple.xml` - Simple drone model without obstacles
- `mujoco_drone_pid.py` - Original realistic flight controller

## Installation

1. Create virtual environment:
```bash
python3 -m venv venv
source venv/bin/activate
```

2. Install dependencies:
```bash
pip install mujoco viser numpy
```

## Usage

1. **Run Main Simulation:**
```bash
source venv/bin/activate
python drone_collision_course.py
```

2. **Open Browser:** Navigate to `http://localhost:8082`

3. **Flight Operations:**
   - Click "ARM" to enable the flight controller
   - Use "POSITION HOLD" or "🔒 HOLD CURRENT POSITION" to start flying
   - Adjust target position with sliders or direct coordinate input
   - Monitor real-time telemetry in the GUI

## Flight Controller Details

### PID Architecture
- **Rate Controllers:** Roll/Pitch/Yaw angular velocity control
- **Attitude Controllers:** Roll/Pitch angle stabilization  
- **Position Controllers:** X/Y position hold with tilt angle limits
- **Altitude Controller:** Z-axis position control with thrust adjustment

### Key Improvements
- **Conservative Gains:** Reduced from kp=1.0 to kp=0.2 to prevent overshoot
- **Attitude Limits:** Maximum tilt angle limited to ~8.5° (was 28.6°)
- **Velocity Damping:** Speed-based gain reduction when moving >1 m/s
- **Collision Recovery:** PID disable during crashes with physics takeover

## Controls

| Control | Description |
|---------|-------------|
| ARM/DISARM | Enable/disable flight controller |
| POSITION HOLD | Fly to slider target coordinates |
| 🔒 HOLD CURRENT | Lock drone at current position |
| 🔄 RESET | Reset simulation to initial state |
| Target Sliders | Adjust X, Y, Z target positions |
| Direct Input | Enter precise coordinates |

## Telemetry Display

- **Current Position:** Real-time X, Y, Z coordinates
- **Speed:** Individual axis velocities + total speed
- **Distance to Target:** 3D and horizontal distance
- **Collision Counter:** Number of detected crashes
- **Last Crash:** Most recent collision details

## Collision Detection

The simulation features realistic collision detection:
- Physics-based contact detection using MuJoCo
- Collision depth measurement
- Automatic PID disable on impact
- Emergency disarm for severe crashes (>5cm penetration)
- Random tumbling motion during crashes

## Development

Built with:
- **MuJoCo** - Physics simulation and collision detection
- **Viser** - Real-time 3D visualization and GUI
- **NumPy** - Mathematical operations and array handling

## Troubleshooting

**Drone won't respond:**
- Ensure drone is ARMED first
- Check that a flight mode is selected (POSITION HOLD)
- Verify target coordinates are within room boundaries

**Excessive oscillation:**
- PID gains are already conservative - check for crashes
- Use RESET to clear any stuck states
- Position hold may take time to stabilize

**Performance issues:**
- Close other browser tabs using port 8082
- Restart simulation if memory usage is high

---

Generated with Claude Code and optimized for realistic drone flight simulation.
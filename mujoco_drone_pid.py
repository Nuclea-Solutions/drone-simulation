import mujoco
import numpy as np
import time
import viser
from pathlib import Path
from flight_controller import FlightController


# Load the model
model = mujoco.MjModel.from_xml_path("drone_simple.xml")
data = mujoco.MjData(model)

# Create viser server
server = viser.ViserServer()

# Initialize flight controller
flight_controller = FlightController()

# Add ground visualization
server.scene.add_box(
    "/ground",
    dimensions=(40, 40, 0.01),
    position=(0, 0, -0.005),
    color=(180, 180, 180),
    opacity=0.5,
)

# Add grid for reference
for i in range(-10, 11):
    # X-axis lines
    server.scene.add_line_segments(
        f"/grid/x_{i}",
        points=np.array([[[i, -10, 0], [i, 10, 0]]]),
        colors=np.array([[100, 100, 100]]),
        line_width=1.0,
    )
    # Y-axis lines
    server.scene.add_line_segments(
        f"/grid/y_{i}",
        points=np.array([[[-10, i, 0], [10, i, 0]]]),
        colors=np.array([[100, 100, 100]]),
        line_width=1.0,
    )

# Add drone visualization
drone_handle = server.scene.add_frame(
    "/drone",
    position=(0, 0, 1),
    show_axes=False,
)

# Drone body
drone_body = server.scene.add_box(
    "/drone/body",
    dimensions=(0.2, 0.2, 0.06),
    position=(0, 0, 0),
    color=(50, 50, 50),  # Gray when disarmed
)

# Arms and motors
motor_positions = [(0.25, 0.25), (-0.25, 0.25), (-0.25, -0.25), (0.25, -0.25)]
motor_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]

for i, ((x, y), color) in enumerate(zip(motor_positions, motor_colors)):
    # Arms
    server.scene.add_box(
        f"/drone/arm{i}",
        dimensions=(0.35, 0.04, 0.02) if abs(x) > abs(y) else (0.04, 0.35, 0.02),
        position=(x/2, y/2, 0),
        color=(80, 80, 80),
    )
    # Motors
    server.scene.add_icosphere(
        f"/drone/motor{i}",
        radius=0.08,
        position=(x, y, 0.02),
        color=color,
    )

# Add coordinate frame at drone position for debugging
debug_frame = server.scene.add_frame(
    "/drone_debug",
    show_axes=True,
    axes_length=0.3,
    axes_radius=0.01,
)

# GUI controls
server.gui.add_markdown("# Realistic Flight Controller")
server.gui.add_markdown("### Flight Modes")

# Flight mode buttons
mode_folder = server.gui.add_folder("Flight Modes")
with mode_folder:
    stabilize_btn = server.gui.add_button("Stabilize Mode")
    altitude_btn = server.gui.add_button("Altitude Hold")
    position_btn = server.gui.add_button("Position Hold")
    acro_btn = server.gui.add_button("Acro Mode")

# Control inputs
control_folder = server.gui.add_folder("Control Inputs")
with control_folder:
    # For stabilize/altitude modes
    roll_input = server.gui.add_slider(
        "Roll (deg)", min=-30, max=30, step=1, initial_value=0
    )
    pitch_input = server.gui.add_slider(
        "Pitch (deg)", min=-30, max=30, step=1, initial_value=0
    )
    yaw_rate_input = server.gui.add_slider(
        "Yaw Rate (deg/s)", min=-90, max=90, step=5, initial_value=0
    )
    
    # For altitude/position hold
    altitude_input = server.gui.add_slider(
        "Target Altitude (m)", min=0.5, max=5.0, step=0.1, initial_value=1.5
    )
    
    # For position hold
    x_pos_input = server.gui.add_slider(
        "Target X (m)", min=-5, max=5, step=0.1, initial_value=0
    )
    y_pos_input = server.gui.add_slider(
        "Target Y (m)", min=-5, max=5, step=0.1, initial_value=0
    )

# PID tuning (advanced)
tuning_folder = server.gui.add_folder("PID Tuning (Advanced)")
with tuning_folder:
    # Attitude PID gains
    roll_kp = server.gui.add_slider("Roll Angle Kp", 0, 10, 0.1, 5.0)
    pitch_kp = server.gui.add_slider("Pitch Angle Kp", 0, 10, 0.1, 5.0)
    
    # Rate PID gains
    roll_rate_kp = server.gui.add_slider("Roll Rate Kp", 0, 0.5, 0.01, 0.15)
    pitch_rate_kp = server.gui.add_slider("Pitch Rate Kp", 0, 0.5, 0.01, 0.15)
    
    # Altitude PID gains
    alt_kp = server.gui.add_slider("Altitude Kp", 0, 5, 0.1, 2.0)
    alt_ki = server.gui.add_slider("Altitude Ki", 0, 2, 0.05, 0.5)
    alt_kd = server.gui.add_slider("Altitude Kd", 0, 3, 0.1, 1.0)

# Status display
status_folder = server.gui.add_folder("Status")
with status_folder:
    mode_text = server.gui.add_text("Mode", initial_value="Disarmed", disabled=True)
    altitude_text = server.gui.add_text("Altitude", initial_value="0.0 m", disabled=True)
    attitude_text = server.gui.add_text("Attitude", initial_value="R:0 P:0 Y:0", disabled=True)
    motor_text = server.gui.add_text("Motors", initial_value="0 0 0 0", disabled=True)

# Notifications display
notifications_folder = server.gui.add_folder("Notifications", expand_by_default=True)
with notifications_folder:
    notification_text = server.gui.add_markdown("**Status:** Ready to arm")
    
# Action buttons
arm_button = server.gui.add_button("Arm")
disarm_button = server.gui.add_button("Disarm")
reset_button = server.gui.add_button("Reset")

# State
armed = False
current_mode = "stabilize"

print("MuJoCo + Viser Drone with Realistic Flight Controller")
print("Open http://localhost:8082 to see the visualization")
print("Features:")
print("- Cascaded PID control (angle → rate → motors)")
print("- Multiple flight modes")
print("- Real-time PID tuning")
print("Press Ctrl+C to stop")

def reset_drone():
    """Reset drone to ground"""
    global armed
    mujoco.mj_resetData(model, data)
    data.qpos[2] = 0.1  # Start on ground
    flight_controller.disarm()
    armed = False
    mode_text.value = "Disarmed"
    drone_body.color = (50, 50, 50)  # Gray when disarmed - direct property update
    notification_text.content = "🔄 **Drone Reset** - Ready to arm"

def update_pid_gains():
    """Update PID gains from GUI"""
    flight_controller.roll_angle_pid.gains.kp = roll_kp.value
    flight_controller.pitch_angle_pid.gains.kp = pitch_kp.value
    flight_controller.roll_rate_pid.gains.kp = roll_rate_kp.value
    flight_controller.pitch_rate_pid.gains.kp = pitch_rate_kp.value
    flight_controller.altitude_pid.gains.kp = alt_kp.value
    flight_controller.altitude_pid.gains.ki = alt_ki.value
    flight_controller.altitude_pid.gains.kd = alt_kd.value

# Button callbacks
@arm_button.on_click
def _(_):
    global armed
    armed = True
    flight_controller.arm()
    mode_text.value = f"Armed - {flight_controller.flight_mode}"
    drone_body.color = (0, 255, 0)  # Green when armed - direct property update
    notification_text.content = "✅ **ARMED** - Drone is ready for flight! Select a flight mode."

@disarm_button.on_click
def _(_):
    global armed
    armed = False
    flight_controller.disarm()
    mode_text.value = "Disarmed"
    drone_body.color = (50, 50, 50)  # Gray when disarmed - direct property update
    notification_text.content = "❌ **DISARMED** - Motors stopped. Safe to approach."

@reset_button.on_click
def _(_):
    reset_drone()

@stabilize_btn.on_click
def _(_):
    if armed:
        flight_controller.set_mode("stabilize")
        mode_text.value = "Armed - Stabilize"
        drone_body.color = (0, 200, 255)  # Light blue for stabilize - direct property update
        notification_text.content = "🎯 **STABILIZE MODE** - Use Roll/Pitch sliders to control attitude"
    else:
        notification_text.content = "⚠️ **ARM FIRST** - Please arm the drone before selecting flight mode"

@altitude_btn.on_click
def _(_):
    if armed:
        flight_controller.set_mode("altitude_hold")
        flight_controller.altitude_setpoint = altitude_input.value
        mode_text.value = "Armed - Altitude Hold"
        drone_body.color = (255, 165, 0)  # Orange for altitude hold - direct property update
        notification_text.content = f"📏 **ALTITUDE HOLD** - Maintaining {altitude_input.value:.1f}m height"
    else:
        notification_text.content = "⚠️ **ARM FIRST** - Please arm the drone before selecting flight mode"

@position_btn.on_click
def _(_):
    if armed:
        flight_controller.set_mode("position_hold")
        flight_controller.altitude_setpoint = altitude_input.value
        flight_controller.position_setpoint = np.array([x_pos_input.value, y_pos_input.value])
        mode_text.value = "Armed - Position Hold"
        drone_body.color = (255, 0, 255)  # Magenta for position hold - direct property update
        notification_text.content = f"📍 **POSITION HOLD** - Locked at ({x_pos_input.value:.1f}, {altitude_input.value:.1f}m)"
    else:
        notification_text.content = "⚠️ **ARM FIRST** - Please arm the drone before selecting flight mode"

@acro_btn.on_click
def _(_):
    if armed:
        flight_controller.set_mode("acro")
        mode_text.value = "Armed - Acro"
        drone_body.color = (255, 0, 0)  # Red for acro (advanced mode) - direct property update
        notification_text.content = "🚁 **ACRO MODE** - Direct rate control - Advanced users only!"
    else:
        notification_text.content = "⚠️ **ARM FIRST** - Please arm the drone before selecting flight mode"

# Update callbacks for control inputs
@roll_input.on_update
def _(_):
    flight_controller.roll_setpoint = np.radians(roll_input.value)

@pitch_input.on_update
def _(_):
    flight_controller.pitch_setpoint = np.radians(pitch_input.value)

@yaw_rate_input.on_update
def _(_):
    flight_controller.yaw_rate_setpoint = np.radians(yaw_rate_input.value)

@altitude_input.on_update
def _(_):
    if flight_controller.flight_mode in ["altitude_hold", "position_hold"]:
        flight_controller.altitude_setpoint = altitude_input.value

@x_pos_input.on_update
def _(_):
    if flight_controller.flight_mode == "position_hold":
        flight_controller.position_setpoint[0] = x_pos_input.value

@y_pos_input.on_update
def _(_):
    if flight_controller.flight_mode == "position_hold":
        flight_controller.position_setpoint[1] = y_pos_input.value

# Initialize
reset_drone()

try:
    step_count = 0
    while True:
        # Update PID gains
        update_pid_gains()
        
        # Get drone state
        drone_pos = data.qpos[:3].copy()
        drone_quat = data.qpos[3:7].copy()
        drone_vel = data.qvel[:3].copy() if len(data.qvel) >= 3 else np.zeros(3)
        drone_angvel = data.qvel[3:6].copy() if len(data.qvel) >= 6 else np.zeros(3)
        
        # Prepare state for flight controller
        state = {
            'position': drone_pos,
            'quaternion': drone_quat,
            'velocity': drone_vel,
            'angular_velocity': drone_angvel
        }
        
        # Get motor commands from flight controller
        if armed and drone_pos[2] > 0.05:  # Only control if armed and off ground
            motor_commands = flight_controller.update(state, model.opt.timestep)
            data.ctrl[:] = motor_commands
        else:
            data.ctrl[:] = 0
            # Auto-disarm if on ground
            if armed and drone_pos[2] <= 0.05:
                armed = False
                flight_controller.disarm()
                mode_text.value = "Auto-disarmed (on ground)"
                drone_body.color = (50, 50, 50)  # Gray when disarmed - direct property update
                notification_text.content = "🛬 **AUTO-DISARMED** - Drone landed safely"
        
        # Step physics
        mujoco.mj_step(model, data)
        step_count += 1
        
        # Update visualization every few steps
        if step_count % 5 == 0:
            # Update drone visualization
            drone_handle.position = tuple(drone_pos)
            drone_handle.wxyz = tuple(drone_quat)
            debug_frame.position = tuple(drone_pos)
            debug_frame.wxyz = tuple(drone_quat)
            
            # Update status display
            roll, pitch, yaw = flight_controller.quaternion_to_euler(drone_quat)
            altitude_text.value = f"{drone_pos[2]:.2f} m"
            attitude_text.value = f"R:{np.degrees(roll):.1f} P:{np.degrees(pitch):.1f} Y:{np.degrees(yaw):.1f}"
            motor_text.value = f"{data.ctrl[0]:.2f} {data.ctrl[1]:.2f} {data.ctrl[2]:.2f} {data.ctrl[3]:.2f}"
        
        # Small delay
        time.sleep(0.002)
        
except KeyboardInterrupt:
    print("\nShutting down...")
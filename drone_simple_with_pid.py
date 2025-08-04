import mujoco
import numpy as np
import time
import viser
from flight_controller import FlightController

# Load drone model
model = mujoco.MjModel.from_xml_path("drone_simple.xml")
data = mujoco.MjData(model)

# Create server
server = viser.ViserServer()

# Initialize REAL PID flight controller
flight_controller = FlightController()

# Simple drone visualization
drone_frame = server.scene.add_frame("/drone", position=(0, 0, 0.1))
server.scene.add_box("/drone/body", dimensions=(0.2, 0.2, 0.06), color=(100, 100, 100))

# Environment - Ground, walls, ceiling, obstacles
# Ground
server.scene.add_box("/ground", dimensions=(10, 10, 0.1), position=(0, 0, -0.05), color=(200, 200, 200))

# Ceiling
server.scene.add_box("/ceiling", dimensions=(10, 10, 0.1), position=(0, 0, 5.0), color=(150, 150, 255), opacity=0.7)

# Walls (room boundaries)
server.scene.add_box("/wall_north", dimensions=(10, 0.2, 5), position=(0, 5, 2.5), color=(255, 200, 200), opacity=0.8)
server.scene.add_box("/wall_south", dimensions=(10, 0.2, 5), position=(0, -5, 2.5), color=(255, 200, 200), opacity=0.8)
server.scene.add_box("/wall_east", dimensions=(0.2, 10, 5), position=(5, 0, 2.5), color=(200, 255, 200), opacity=0.8)
server.scene.add_box("/wall_west", dimensions=(0.2, 10, 5), position=(-5, 0, 2.5), color=(200, 255, 200), opacity=0.8)

# Obstacles - Various shapes and colors
# Red tower obstacle
server.scene.add_box("/obstacle_tower", dimensions=(1.0, 1.0, 2.0), position=(2, 2, 1.0), color=(255, 0, 0))

# Blue box obstacles
server.scene.add_box("/obstacle_box1", dimensions=(0.8, 0.8, 1.5), position=(-2, 1.5, 0.75), color=(0, 0, 255))
server.scene.add_box("/obstacle_box2", dimensions=(1.2, 0.6, 1.0), position=(1, -2.5, 0.5), color=(255, 255, 0))

# Floating sphere obstacles
server.scene.add_icosphere("/obstacle_sphere1", radius=0.4, position=(-1.5, -1.5, 2.5), color=(255, 0, 255))
server.scene.add_icosphere("/obstacle_sphere2", radius=0.3, position=(3, -1, 3.5), color=(0, 255, 255))

# Tall brown pillar obstacle
server.scene.add_box("/obstacle_pillar", dimensions=(0.4, 0.4, 4.0), position=(-3, 2, 2.0), color=(139, 69, 19))

# Additional obstacles to make it challenging
server.scene.add_box("/obstacle_arch1", dimensions=(0.3, 2.0, 0.3), position=(0, 3, 3.5), color=(128, 128, 128))  # Arch part 1
server.scene.add_box("/obstacle_arch2", dimensions=(0.3, 2.0, 0.3), position=(0, 1, 3.5), color=(128, 128, 128))  # Arch part 2

# Simple GUI with PID status
status = server.gui.add_markdown("**STATUS: DISARMED**")
height_display = server.gui.add_markdown("**Height: 0.0m**")
mode_display = server.gui.add_markdown("**Mode: Stabilize**")
motors_display = server.gui.add_markdown("**Motors: 0 0 0 0**")

# Simple controls
arm_button = server.gui.add_button("ARM")
altitude_hold_button = server.gui.add_button("ALTITUDE HOLD")
position_hold_button = server.gui.add_button("POSITION HOLD")

altitude_slider = server.gui.add_slider("Target Height", 0.5, 4.5, 0.1, 2.0)
x_position_slider = server.gui.add_slider("X Position", -4.0, 4.0, 0.1, 0.0)
y_position_slider = server.gui.add_slider("Y Position", -4.0, 4.0, 0.1, 0.0)

# State
armed = False
flying = False

print("Drone Obstacle Course with REAL PID Controller - http://localhost:8082")
print("🚁 Navigate through the colorful obstacles!")
print("1. ARM → 2. ALTITUDE HOLD or POSITION HOLD → 3. Use sliders to fly around")

@arm_button.on_click
def arm_drone(_):
    global armed, flying
    if not armed:
        armed = True
        flying = False  # Reset flying state
        flight_controller.arm()  # Real PID controller
        status.content = "**STATUS: ✅ ARMED - Ready to fly**"
        mode_display.content = "**Mode: Armed - Select flight mode**"
        arm_button.text = "DISARM"
        print("✅ ARMED with PID controller")
    else:
        armed = False
        flying = False
        flight_controller.disarm()
        status.content = "**STATUS: ❌ DISARMED**"
        mode_display.content = "**Mode: Disarmed**"
        arm_button.text = "ARM"
        print("❌ DISARMED")

@altitude_hold_button.on_click
def start_altitude_hold(_):
    global flying
    if armed and not flying:
        flying = True
        flight_controller.set_mode("altitude_hold")  # Real PID mode
        flight_controller.altitude_setpoint = altitude_slider.value
        status.content = f"**STATUS: 🚁 ALTITUDE HOLD - {altitude_slider.value:.1f}m**"
        mode_display.content = "**Mode: Altitude Hold (PID Active)**"
        print(f"🚁 PID ALTITUDE HOLD at {altitude_slider.value:.1f}m")

@position_hold_button.on_click
def start_position_hold(_):
    global flying
    if armed and not flying:
        flying = True
        flight_controller.set_mode("position_hold")  # GPS-like mode
        flight_controller.altitude_setpoint = altitude_slider.value
        flight_controller.position_setpoint = np.array([x_position_slider.value, y_position_slider.value])
        status.content = f"**STATUS: 🚁 POSITION HOLD - ({x_position_slider.value:.1f}, {y_position_slider.value:.1f}, {altitude_slider.value:.1f})**"
        mode_display.content = "**Mode: Position Hold (GPS-like PID)**"
        print(f"🚁 PID POSITION HOLD at ({x_position_slider.value:.1f}, {y_position_slider.value:.1f}, {altitude_slider.value:.1f})")

@altitude_slider.on_update
def update_altitude_target(_):
    if flying:
        flight_controller.altitude_setpoint = altitude_slider.value
        if flight_controller.flight_mode == "altitude_hold":
            status.content = f"**STATUS: 🚁 ALTITUDE HOLD - {altitude_slider.value:.1f}m**"
        elif flight_controller.flight_mode == "position_hold":
            status.content = f"**STATUS: 🚁 POSITION HOLD - ({x_position_slider.value:.1f}, {y_position_slider.value:.1f}, {altitude_slider.value:.1f})**"

@x_position_slider.on_update
def update_x_target(_):
    if flying and flight_controller.flight_mode == "position_hold":
        flight_controller.position_setpoint[0] = x_position_slider.value
        status.content = f"**STATUS: 🚁 POSITION HOLD - ({x_position_slider.value:.1f}, {y_position_slider.value:.1f}, {altitude_slider.value:.1f})**"

@y_position_slider.on_update
def update_y_target(_):
    if flying and flight_controller.flight_mode == "position_hold":
        flight_controller.position_setpoint[1] = y_position_slider.value
        status.content = f"**STATUS: 🚁 POSITION HOLD - ({x_position_slider.value:.1f}, {y_position_slider.value:.1f}, {altitude_slider.value:.1f})**"

# Reset drone - start higher so it doesn't auto-disarm
mujoco.mj_resetData(model, data)
data.qpos[2] = 1.0  # Start at 1m height, not on ground

try:
    while True:
        # Get full drone state for PID controller
        drone_pos = data.qpos[:3].copy()
        drone_quat = data.qpos[3:7].copy()
        drone_vel = data.qvel[:3].copy() if len(data.qvel) >= 3 else np.zeros(3)
        drone_angvel = data.qvel[3:6].copy() if len(data.qvel) >= 6 else np.zeros(3)
        
        # Use REAL PID flight controller
        if armed and flying:
            state = {
                'position': drone_pos,
                'quaternion': drone_quat,
                'velocity': drone_vel,
                'angular_velocity': drone_angvel
            }
            # This uses the full cascaded PID controller!
            motor_commands = flight_controller.update(state, model.opt.timestep)
            data.ctrl[:] = motor_commands
        else:
            data.ctrl[:] = 0
        
        # Disable auto-disarm for testing - manual disarm only
        # if armed and flying and drone_pos[2] <= 0.05:
        #     print("🛬 AUTO-DISARMED - Landed")
        
        # MuJoCo physics step
        mujoco.mj_step(model, data)
        
        # Update display
        if int(time.time() * 50) % 10 == 0:
            drone_frame.position = (drone_pos[0], drone_pos[1], drone_pos[2])
            height_display.content = f"**Height: {drone_pos[2]:.1f}m**"
            motors_display.content = f"**Motors: {data.ctrl[0]:.1f} {data.ctrl[1]:.1f} {data.ctrl[2]:.1f} {data.ctrl[3]:.1f}**"
        
        time.sleep(0.002)
        
except KeyboardInterrupt:
    print("Shutdown complete")
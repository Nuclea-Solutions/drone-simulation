import mujoco
import numpy as np
import time
import viser
from flight_controller import FlightController

# Load drone model WITH obstacles
model = mujoco.MjModel.from_xml_path("drone_with_obstacles.xml")
data = mujoco.MjData(model)

# Create server
server = viser.ViserServer()

# Initialize REAL PID flight controller
flight_controller = FlightController()

# Simple drone visualization (viser display - matches MuJoCo obstacles)
drone_frame = server.scene.add_frame("/drone", position=(0, 0, 0.1))
server.scene.add_box("/drone/body", dimensions=(0.2, 0.2, 0.06), color=(100, 100, 100))

# Environment visualization (must match MuJoCo XML)
server.scene.add_box("/ground", dimensions=(10, 10, 0.1), position=(0, 0, -0.05), color=(200, 200, 200))
server.scene.add_box("/ceiling", dimensions=(10, 10, 0.1), position=(0, 0, 5.0), color=(150, 150, 255), opacity=0.3)

# Walls
server.scene.add_box("/wall_north", dimensions=(10, 0.2, 5), position=(0, 5, 2.5), color=(255, 200, 200), opacity=0.8)
server.scene.add_box("/wall_south", dimensions=(10, 0.2, 5), position=(0, -5, 2.5), color=(255, 200, 200), opacity=0.8)
server.scene.add_box("/wall_east", dimensions=(0.2, 10, 5), position=(5, 0, 2.5), color=(200, 255, 200), opacity=0.8)
server.scene.add_box("/wall_west", dimensions=(0.2, 10, 5), position=(-5, 0, 2.5), color=(200, 255, 200), opacity=0.8)

# Obstacles (visual - matching MuJoCo physics)
server.scene.add_box("/obstacle_tower", dimensions=(1.0, 1.0, 2.0), position=(2, 2, 1.0), color=(255, 0, 0))
server.scene.add_box("/obstacle_box1", dimensions=(0.8, 0.8, 1.5), position=(-2, 1.5, 0.75), color=(0, 0, 255))
server.scene.add_box("/obstacle_box2", dimensions=(1.2, 0.6, 1.0), position=(1, -2.5, 0.5), color=(255, 255, 0))
server.scene.add_icosphere("/obstacle_sphere1", radius=0.4, position=(-1.5, -1.5, 2.5), color=(255, 0, 255))
server.scene.add_icosphere("/obstacle_sphere2", radius=0.3, position=(3, -1, 3.5), color=(0, 255, 255))
server.scene.add_box("/obstacle_pillar", dimensions=(0.4, 0.4, 4.0), position=(-3, 2, 2.0), color=(139, 69, 19))
server.scene.add_box("/obstacle_arch1", dimensions=(0.3, 2.0, 0.3), position=(0, 3, 3.5), color=(128, 128, 128))
server.scene.add_box("/obstacle_arch2", dimensions=(0.3, 2.0, 0.3), position=(0, 1, 3.5), color=(128, 128, 128))

# GUI
server.gui.add_markdown("# 🚁 REAL COLLISION COURSE")
server.gui.add_markdown("**⚠️ OBSTACLES HAVE REAL PHYSICS - CRASHES WILL HAPPEN!**")

status = server.gui.add_markdown("**STATUS: DISARMED**")
height_display = server.gui.add_markdown("**Height: 0.0m**")
collision_display = server.gui.add_markdown("**Collisions: 0**")
crash_log = server.gui.add_markdown("**Last Crash: None**")

# Current position display
server.gui.add_markdown("### 📍 Current Drone Position")
current_x = server.gui.add_text("Current X", initial_value="0.00", disabled=True)
current_y = server.gui.add_text("Current Y", initial_value="0.00", disabled=True)
current_z = server.gui.add_text("Current Z", initial_value="1.00", disabled=True)
distance_to_target = server.gui.add_text("Distance to Target", initial_value="0.00 m", disabled=True)

# Speed display
server.gui.add_markdown("### ⚡ Current Speed")
speed_x = server.gui.add_text("Speed X", initial_value="0.00 m/s", disabled=True)
speed_y = server.gui.add_text("Speed Y", initial_value="0.00 m/s", disabled=True)  
speed_z = server.gui.add_text("Speed Z", initial_value="0.00 m/s", disabled=True)
total_speed = server.gui.add_text("Total Speed", initial_value="0.00 m/s", disabled=True)

# Controls
arm_button = server.gui.add_button("ARM")
position_hold_button = server.gui.add_button("POSITION HOLD")
hold_current_button = server.gui.add_button("🔒 HOLD CURRENT POSITION")
reset_button = server.gui.add_button("🔄 RESET SIMULATION")

# Sliders (original)
altitude_slider = server.gui.add_slider("Target Height", 0.5, 4.5, 0.1, 2.0)
x_position_slider = server.gui.add_slider("X Position", -4.0, 4.0, 0.1, 0.0)
y_position_slider = server.gui.add_slider("Y Position", -4.0, 4.0, 0.1, 0.0)

# Direct coordinate input fields
server.gui.add_markdown("### 🎯 Direct Coordinate Input")
x_input = server.gui.add_number("Target X", initial_value=0.0, step=0.1)
y_input = server.gui.add_number("Target Y", initial_value=0.0, step=0.1) 
z_input = server.gui.add_number("Target Z", initial_value=2.0, step=0.1)
goto_button = server.gui.add_button("🚁 GO TO COORDINATES")

# State
armed = False
flying = False
collision_count = 0
last_crash_obstacle = ""
crashed = False  # New state for crashed drone

print("🚁 REAL COLLISION DRONE COURSE - http://localhost:8082")
print("⚠️  Physical obstacles - crashes will affect the drone!")
print("🔴 Try flying into the red tower to see what happens!")

def check_collisions():
    """Check for collisions and log them"""
    global collision_count, last_crash_obstacle, crashed
    
    # Get contact information from MuJoCo
    if data.ncon > 0:  # Contacts detected
        for i in range(min(data.ncon, len(data.contact))):  # Prevent IndexError
            contact = data.contact[i]
            geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            
            # Check if drone parts are colliding with obstacles
            drone_parts = ['body', 'arm1', 'arm2', 'arm3', 'arm4', 'rotor1', 'rotor2', 'rotor3', 'rotor4']
            obstacle_parts = ['obstacle_tower', 'obstacle_box1', 'obstacle_box2', 'obstacle_sphere1', 
                            'obstacle_sphere2', 'obstacle_pillar', 'obstacle_arch1', 'obstacle_arch2',
                            'wall_north', 'wall_south', 'wall_east', 'wall_west', 'ceiling']
            
            collision_detected = False
            crashed_into = ""
            
            if geom1_name in drone_parts and geom2_name in obstacle_parts:
                collision_detected = True
                crashed_into = geom2_name
            elif geom2_name in drone_parts and geom1_name in obstacle_parts:
                collision_detected = True
                crashed_into = geom1_name
                
            if collision_detected and crashed_into != last_crash_obstacle:
                collision_count += 1
                last_crash_obstacle = crashed_into
                
                # Get collision depth/distance (simpler than force)
                collision_depth = contact.dist
                
                print(f"💥 COLLISION #{collision_count}: {crashed_into.upper()}")
                print(f"   Collision depth: {collision_depth:.3f}m")
                print(f"   Drone position: ({data.qpos[0]:.2f}, {data.qpos[1]:.2f}, {data.qpos[2]:.2f})")
                print(f"   Current target: ({flight_controller.position_setpoint[0]:.2f}, {flight_controller.position_setpoint[1]:.2f})")
                
                collision_display.content = f"**Collisions: {collision_count}**"
                crash_log.content = f"**Last Crash: {crashed_into.replace('_', ' ').title()} (depth: {abs(collision_depth):.3f}m)**"
                
                # ANY collision disables PID and lets physics take over
                if not crashed:
                    crashed = True
                    print(f"🔥 CRASH! PID disabled - falling under gravity!")
                    # Add some random spin/tumble when crashing
                    data.qvel[3:6] += np.random.normal(0, 5, 3)  # Random angular velocity
                    return "crash"
                
                # Deep collision - emergency disarm
                if abs(collision_depth) > 0.05:  # 5cm deep collision
                    print(f"🔥 DEEP CRASH! Emergency disarm!")
                    return "emergency"
                    
    return False

@arm_button.on_click
def arm_drone(_):
    global armed, flying, crashed
    if not armed:
        armed = True
        flying = False
        crashed = False  # Reset crash state
        flight_controller.arm()
        status.content = "**STATUS: ✅ ARMED - Ready to crash!**"
        arm_button.text = "DISARM"
        print("✅ ARMED - Ready for collision course!")
    else:
        armed = False
        flying = False
        crashed = False
        flight_controller.disarm()
        status.content = "**STATUS: ❌ DISARMED**"
        arm_button.text = "ARM"
        print("❌ DISARMED")

@position_hold_button.on_click
def start_position_hold(_):
    global flying
    print(f"🔴 POSITION HOLD BUTTON CLICKED")
    print(f"   Armed: {armed}")
    print(f"   Flying: {flying}")
    print(f"   Crashed: {crashed}")
    
    if not armed:
        status.content = "**⚠️ ARM FIRST - Need to arm drone before position hold**"
        print("⚠️ POSITION HOLD IGNORED - Drone not armed!")
        return
        
    if crashed:
        status.content = "**⚠️ CRASHED - Reset or disarm/arm to recover**"
        print("⚠️ POSITION HOLD IGNORED - Drone is crashed!")
        return
    
    # Always set position hold mode (whether flying or not)
    flying = True
    flight_controller.set_mode("position_hold")
    flight_controller.altitude_setpoint = altitude_slider.value
    flight_controller.position_setpoint = np.array([x_position_slider.value, y_position_slider.value])
    status.content = f"**STATUS: 🚁 POSITION HOLD ({x_position_slider.value:.1f}, {y_position_slider.value:.1f}, {altitude_slider.value:.1f})**"
    print(f"✅ POSITION HOLD ACTIVATED")
    print(f"   Target: ({x_position_slider.value:.1f}, {y_position_slider.value:.1f}, {altitude_slider.value:.1f})")
    print(f"   Flight mode: {flight_controller.flight_mode}")

@hold_current_button.on_click  
def hold_current_position(_):
    global flying
    print(f"🔒 HOLD CURRENT POSITION CLICKED")
    print(f"   Armed: {armed}")
    print(f"   Flying: {flying}")
    print(f"   Crashed: {crashed}")
    
    if not armed:
        status.content = "**⚠️ ARM FIRST - Need to arm drone before holding position**"
        print("⚠️ HOLD CURRENT IGNORED - Drone not armed!")
        return
        
    if crashed:
        status.content = "**⚠️ CRASHED - Reset or disarm/arm to recover**"
        print("⚠️ HOLD CURRENT IGNORED - Drone is crashed!")
        return
    
    # Get current drone position
    drone_pos = data.qpos[:3].copy()
    
    # Set position hold to current location
    flying = True
    flight_controller.set_mode("position_hold")
    flight_controller.altitude_setpoint = drone_pos[2]
    flight_controller.position_setpoint = np.array([drone_pos[0], drone_pos[1]])
    
    # Update sliders to match current position
    altitude_slider.value = drone_pos[2]
    x_position_slider.value = drone_pos[0]
    y_position_slider.value = drone_pos[1]
    
    # Update input fields too
    x_input.value = drone_pos[0]
    y_input.value = drone_pos[1]
    z_input.value = drone_pos[2]
    
    status.content = f"**STATUS: 🔒 HOLDING CURRENT ({drone_pos[0]:.1f}, {drone_pos[1]:.1f}, {drone_pos[2]:.1f})**"
    print(f"✅ HOLDING CURRENT POSITION")
    print(f"   Locked at: ({drone_pos[0]:.2f}, {drone_pos[1]:.2f}, {drone_pos[2]:.2f})")
    print(f"   Flight mode: {flight_controller.flight_mode}")

@reset_button.on_click
def reset_simulation(_):
    global armed, flying, crashed, collision_count, last_crash_obstacle
    
    # Reset all state variables
    armed = False
    flying = False
    crashed = False
    collision_count = 0
    last_crash_obstacle = ""
    
    # Reset flight controller
    flight_controller.disarm()
    
    # Reset MuJoCo simulation
    mujoco.mj_resetData(model, data)
    data.qpos[2] = 1.0  # Start at 1m height
    
    # Reset GUI elements
    status.content = "**STATUS: 🔄 SIMULATION RESET - Ready to start!**"
    collision_display.content = "**Collisions: 0**"
    crash_log.content = "**Last Crash: None**"
    arm_button.text = "ARM"
    
    # Reset sliders and input fields to default
    altitude_slider.value = 2.0
    x_position_slider.value = 0.0
    y_position_slider.value = 0.0
    x_input.value = 0.0
    y_input.value = 0.0
    z_input.value = 2.0
    
    print("🔄 SIMULATION RESET - All systems ready!")
    print("   Drone position reset to (0, 0, 1m)")
    print("   Collision counter reset")
    print("   Flight controller disarmed")

@altitude_slider.on_update
def update_altitude(_):
    if flying:
        flight_controller.altitude_setpoint = altitude_slider.value

@x_position_slider.on_update  
def update_x(_):
    if flying and flight_controller.flight_mode == "position_hold":
        flight_controller.position_setpoint[0] = x_position_slider.value
        status.content = f"**STATUS: 🚁 FLYING TO ({x_position_slider.value:.1f}, {y_position_slider.value:.1f}, {altitude_slider.value:.1f})**"
        print(f"🎯 X TARGET CHANGED: {x_position_slider.value:.1f}")

@y_position_slider.on_update
def update_y(_):
    if flying and flight_controller.flight_mode == "position_hold":
        flight_controller.position_setpoint[1] = y_position_slider.value
        status.content = f"**STATUS: 🚁 FLYING TO ({x_position_slider.value:.1f}, {y_position_slider.value:.1f}, {altitude_slider.value:.1f})**"
        print(f"🎯 Y TARGET CHANGED: {y_position_slider.value:.1f}")

@goto_button.on_click
def goto_coordinates(_):
    global flying
    if armed:
        # Start flying if not already
        if not flying:
            flying = True
            flight_controller.set_mode("position_hold")
        
        # Set target coordinates from input fields
        target_x = x_input.value
        target_y = y_input.value
        target_z = z_input.value
        
        flight_controller.altitude_setpoint = target_z
        flight_controller.position_setpoint = np.array([target_x, target_y])
        
        # Update sliders to match input fields
        altitude_slider.value = target_z
        x_position_slider.value = target_x
        y_position_slider.value = target_y
        
        status.content = f"**STATUS: 🚁 GOING TO ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})**"
        print(f"🎯 DIRECT COORDINATE COMMAND:")
        print(f"   Target: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})")
        print(f"   Flight controller setpoint updated")
    else:
        status.content = "**⚠️ ARM FIRST - Need to arm drone before flying**"
        print("⚠️ Cannot fly - drone not armed!")

# Reset drone
mujoco.mj_resetData(model, data)
data.qpos[2] = 1.0

try:
    while True:
        # Get drone state
        drone_pos = data.qpos[:3].copy()
        drone_quat = data.qpos[3:7].copy()
        drone_vel = data.qvel[:3].copy() if len(data.qvel) >= 3 else np.zeros(3)
        drone_angvel = data.qvel[3:6].copy() if len(data.qvel) >= 6 else np.zeros(3)
        
        # Control - only use PID if not crashed
        if armed and flying and not crashed:
            state = {
                'position': drone_pos,
                'quaternion': drone_quat,
                'velocity': drone_vel,
                'angular_velocity': drone_angvel
            }
            
            # STABILITY IMPROVEMENT: Reduce PID gains when approaching target
            distance_to_target_val = np.sqrt(
                (flight_controller.position_setpoint[0] - drone_pos[0])**2 + 
                (flight_controller.position_setpoint[1] - drone_pos[1])**2
            )
            
            # CONSERVATIVE PID GAINS - Prevent excessive tilt
            # Much lower gains to keep drone more horizontal
            flight_controller.x_pos_pid.gains.kp = 0.2  # Was 1.0 - too aggressive
            flight_controller.y_pos_pid.gains.kp = 0.2  # Was 1.0 - too aggressive
            flight_controller.x_pos_pid.gains.kd = 0.8  # More damping
            flight_controller.y_pos_pid.gains.kd = 0.8  # More damping
            
            # ATTITUDE ANGLE LIMITS - Keep drone more horizontal
            flight_controller.x_pos_pid.gains.output_limit = 0.15  # Was 0.5 - max ~8.5 degrees tilt
            flight_controller.y_pos_pid.gains.output_limit = 0.15  # Was 0.5 - max ~8.5 degrees tilt
            
            # Additional velocity-based damping when moving fast
            horizontal_speed = np.sqrt(drone_vel[0]**2 + drone_vel[1]**2)
            if horizontal_speed > 1.0:  # If moving faster than 1 m/s
                # Reduce gains further to prevent runaway
                speed_factor = max(0.5, 2.0 / horizontal_speed)  # Scale down with speed
                flight_controller.x_pos_pid.gains.kp *= speed_factor
                flight_controller.y_pos_pid.gains.kp *= speed_factor
            
            # DEBUG: Print enhanced info
            if int(time.time() * 500) % 100 == 0:
                roll, pitch, yaw = flight_controller.quaternion_to_euler(drone_quat)
                print(f"🐛 DEBUG:")
                print(f"   Position: ({drone_pos[0]:.2f}, {drone_pos[1]:.2f}, {drone_pos[2]:.2f})")
                print(f"   Target: ({flight_controller.position_setpoint[0]:.2f}, {flight_controller.position_setpoint[1]:.2f}, {flight_controller.altitude_setpoint:.2f})")
                print(f"   Velocity: ({drone_vel[0]:.2f}, {drone_vel[1]:.2f}, {drone_vel[2]:.2f}) m/s")
                print(f"   Attitude: Roll={np.degrees(roll):.1f}° Pitch={np.degrees(pitch):.1f}° Yaw={np.degrees(yaw):.1f}°")
                print(f"   Distance: {distance_to_target_val:.2f}m | PID Kp: {flight_controller.x_pos_pid.gains.kp:.3f} | Max tilt: {np.degrees(flight_controller.x_pos_pid.gains.output_limit):.1f}°")
            
            motor_commands = flight_controller.update(state, model.opt.timestep)
            data.ctrl[:] = motor_commands
        else:
            # No thrust - let gravity and physics take over
            data.ctrl[:] = 0
        
        # Physics step
        mujoco.mj_step(model, data)
        
        # Check for collisions AFTER physics step
        crash_result = check_collisions()
        if crash_result == "crash":
            # Crash detected - PID disabled, physics takes over
            status.content = "**STATUS: 💥 CRASHED - Falling under gravity!**"
        elif crash_result == "emergency" and armed:
            # Emergency disarm
            armed = False
            flying = False
            crashed = False
            flight_controller.disarm()
            status.content = "**STATUS: 🔥 EMERGENCY DISARMED - HEAVY CRASH!**"
            arm_button.text = "ARM"
        
        # Update display
        if int(time.time() * 50) % 10 == 0:
            drone_frame.position = (drone_pos[0], drone_pos[1], drone_pos[2])
            drone_frame.wxyz = (drone_quat[0], drone_quat[1], drone_quat[2], drone_quat[3])
            height_display.content = f"**Height: {drone_pos[2]:.1f}m**"
            
            # Update current position display
            current_x.value = f"{drone_pos[0]:.2f}"
            current_y.value = f"{drone_pos[1]:.2f}" 
            current_z.value = f"{drone_pos[2]:.2f}"
            
            # Update speed display
            speed_x.value = f"{drone_vel[0]:.2f} m/s"
            speed_y.value = f"{drone_vel[1]:.2f} m/s"
            speed_z.value = f"{drone_vel[2]:.2f} m/s"
            total_speed_val = np.sqrt(drone_vel[0]**2 + drone_vel[1]**2 + drone_vel[2]**2)
            total_speed.value = f"{total_speed_val:.2f} m/s"
            
            # Calculate and display distance to target
            if flying and flight_controller.flight_mode == "position_hold":
                target_x = flight_controller.position_setpoint[0] 
                target_y = flight_controller.position_setpoint[1]
                target_z = flight_controller.altitude_setpoint
                
                distance_3d = np.sqrt(
                    (target_x - drone_pos[0])**2 + 
                    (target_y - drone_pos[1])**2 + 
                    (target_z - drone_pos[2])**2
                )
                distance_xy = np.sqrt(
                    (target_x - drone_pos[0])**2 + 
                    (target_y - drone_pos[1])**2
                )
                
                distance_to_target.value = f"3D: {distance_3d:.2f}m | XY: {distance_xy:.2f}m"
                
                # Check if drone has reached target (within 0.1m tolerance)
                if distance_3d < 0.1 and not hasattr(goto_coordinates, 'arrival_logged'):
                    print(f"✅ TARGET REACHED! Distance: {distance_3d:.3f}m")
                    print(f"   Current: ({drone_pos[0]:.2f}, {drone_pos[1]:.2f}, {drone_pos[2]:.2f})")
                    print(f"   Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
                    status.content = f"**STATUS: ✅ TARGET REACHED ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})**"
                    goto_coordinates.arrival_logged = True
                elif distance_3d >= 0.1:
                    # Reset arrival flag when moving away from target
                    if hasattr(goto_coordinates, 'arrival_logged'):
                        delattr(goto_coordinates, 'arrival_logged')
            else:
                distance_to_target.value = "Not flying"
        
        time.sleep(0.002)
        
except KeyboardInterrupt:
    print("Crash course completed!")
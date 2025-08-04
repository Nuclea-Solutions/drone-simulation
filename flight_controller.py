import numpy as np
from dataclasses import dataclass
from typing import Tuple
import time


@dataclass
class PIDGains:
    """PID controller gains"""
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    integral_limit: float = 1.0
    output_limit: float = 1.0


class PIDController:
    """Single-axis PID controller with anti-windup and derivative filtering"""
    
    def __init__(self, gains: PIDGains):
        self.gains = gains
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        self.derivative_filter = 0.0
        self.filter_coefficient = 0.8  # Low-pass filter for derivative
        
    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        self.derivative_filter = 0.0
        
    def update(self, error: float, dt: float = None) -> float:
        """Update PID controller with error signal"""
        if dt is None:
            current_time = time.time()
            if self.last_time is None:
                dt = 0.01  # Default timestep
            else:
                dt = current_time - self.last_time
            self.last_time = current_time
        
        # Proportional term
        p_term = self.gains.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = np.clip(
            self.integral, 
            -self.gains.integral_limit, 
            self.gains.integral_limit
        )
        i_term = self.gains.ki * self.integral
        
        # Derivative term with filtering
        if dt > 0:
            derivative = (error - self.last_error) / dt
            self.derivative_filter = (
                self.filter_coefficient * self.derivative_filter + 
                (1 - self.filter_coefficient) * derivative
            )
        d_term = self.gains.kd * self.derivative_filter
        
        self.last_error = error
        
        # Calculate output with limiting
        output = p_term + i_term + d_term
        return np.clip(output, -self.gains.output_limit, self.gains.output_limit)


class FlightController:
    """Cascaded flight controller for quadcopter
    
    Implements:
    - Inner loop: Angular rate control
    - Middle loop: Attitude control
    - Outer loop: Position control
    - Altitude control
    """
    
    def __init__(self):
        # Angular rate controllers (rad/s)
        self.roll_rate_pid = PIDController(PIDGains(
            kp=0.15, ki=0.1, kd=0.004, 
            integral_limit=0.5, output_limit=1.0
        ))
        self.pitch_rate_pid = PIDController(PIDGains(
            kp=0.15, ki=0.1, kd=0.004,
            integral_limit=0.5, output_limit=1.0
        ))
        self.yaw_rate_pid = PIDController(PIDGains(
            kp=0.2, ki=0.1, kd=0.0,
            integral_limit=0.5, output_limit=1.0
        ))
        
        # Attitude controllers (rad)
        self.roll_angle_pid = PIDController(PIDGains(
            kp=5.0, ki=0.0, kd=0.0,
            integral_limit=0.5, output_limit=3.0  # Max rate setpoint
        ))
        self.pitch_angle_pid = PIDController(PIDGains(
            kp=5.0, ki=0.0, kd=0.0,
            integral_limit=0.5, output_limit=3.0
        ))
        
        # Position controllers (m)
        self.x_pos_pid = PIDController(PIDGains(
            kp=1.0, ki=0.0, kd=0.5,
            integral_limit=2.0, output_limit=0.5  # Max tilt angle
        ))
        self.y_pos_pid = PIDController(PIDGains(
            kp=1.0, ki=0.0, kd=0.5,
            integral_limit=2.0, output_limit=0.5
        ))
        
        # Altitude controller (m)
        self.altitude_pid = PIDController(PIDGains(
            kp=2.0, ki=0.5, kd=1.0,
            integral_limit=2.0, output_limit=4.0  # Max vertical acceleration
        ))
        
        # Flight mode
        self.flight_mode = "stabilize"  # stabilize, altitude_hold, position_hold, acro
        
        # Setpoints
        self.roll_setpoint = 0.0
        self.pitch_setpoint = 0.0
        self.yaw_rate_setpoint = 0.0
        self.altitude_setpoint = 1.0
        self.position_setpoint = np.array([0.0, 0.0])
        
        # Base thrust for hover (will be tuned)
        self.base_thrust = 1.91
        
    def quaternion_to_euler(self, q: np.ndarray) -> Tuple[float, float, float]:
        """Convert quaternion (w,x,y,z) to Euler angles (roll, pitch, yaw)"""
        w, x, y, z = q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
            
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def motor_mixing(self, roll: float, pitch: float, yaw: float, thrust: float) -> np.ndarray:
        """Mix control inputs to motor commands
        
        Motor layout (X configuration):
        1 (front-right)    0 (front-left)
                  X
        3 (back-right)     2 (back-left)
        """
        # Motor mixing for X configuration
        # Each motor contributes to thrust, roll, pitch, and yaw
        motor_commands = np.array([
            thrust + roll - pitch - yaw,  # Front-left
            thrust - roll - pitch + yaw,  # Front-right
            thrust + roll + pitch + yaw,  # Back-left
            thrust - roll + pitch - yaw,  # Back-right
        ])
        
        # Ensure motor commands are non-negative and within limits
        motor_commands = np.clip(motor_commands, 0.0, 5.0)
        
        return motor_commands
    
    def update(self, state: dict, dt: float = 0.002) -> np.ndarray:
        """Update flight controller
        
        Args:
            state: Dictionary containing:
                - position: [x, y, z]
                - quaternion: [w, x, y, z]
                - velocity: [vx, vy, vz]
                - angular_velocity: [wx, wy, wz]
            dt: Time step
            
        Returns:
            Motor commands [m0, m1, m2, m3]
        """
        # Extract state
        position = state['position']
        quaternion = state['quaternion']
        velocity = state['velocity']
        angular_velocity = state['angular_velocity']
        
        # Convert quaternion to Euler angles
        roll, pitch, yaw = self.quaternion_to_euler(quaternion)
        
        # Control outputs
        roll_out = 0.0
        pitch_out = 0.0
        yaw_out = 0.0
        thrust_out = self.base_thrust
        
        if self.flight_mode == "acro":
            # Direct rate control
            roll_out = self.roll_rate_pid.update(
                self.roll_setpoint - angular_velocity[0], dt
            )
            pitch_out = self.pitch_rate_pid.update(
                self.pitch_setpoint - angular_velocity[1], dt
            )
            yaw_out = self.yaw_rate_pid.update(
                self.yaw_rate_setpoint - angular_velocity[2], dt
            )
            
        elif self.flight_mode == "stabilize":
            # Angle stabilization
            # Outer loop: angle -> rate setpoint
            roll_rate_sp = self.roll_angle_pid.update(
                self.roll_setpoint - roll, dt
            )
            pitch_rate_sp = self.pitch_angle_pid.update(
                self.pitch_setpoint - pitch, dt
            )
            
            # Inner loop: rate control
            roll_out = self.roll_rate_pid.update(
                roll_rate_sp - angular_velocity[0], dt
            )
            pitch_out = self.pitch_rate_pid.update(
                pitch_rate_sp - angular_velocity[1], dt
            )
            yaw_out = self.yaw_rate_pid.update(
                self.yaw_rate_setpoint - angular_velocity[2], dt
            )
            
        elif self.flight_mode == "altitude_hold":
            # Altitude control
            altitude_error = self.altitude_setpoint - position[2]
            thrust_adjustment = self.altitude_pid.update(altitude_error, dt)
            thrust_out = self.base_thrust + thrust_adjustment
            
            # Still do attitude stabilization
            roll_rate_sp = self.roll_angle_pid.update(
                self.roll_setpoint - roll, dt
            )
            pitch_rate_sp = self.pitch_angle_pid.update(
                self.pitch_setpoint - pitch, dt
            )
            
            roll_out = self.roll_rate_pid.update(
                roll_rate_sp - angular_velocity[0], dt
            )
            pitch_out = self.pitch_rate_pid.update(
                pitch_rate_sp - angular_velocity[1], dt
            )
            yaw_out = self.yaw_rate_pid.update(
                self.yaw_rate_setpoint - angular_velocity[2], dt
            )
            
        elif self.flight_mode == "position_hold":
            # Position control
            pos_error_x = self.position_setpoint[0] - position[0]
            pos_error_y = self.position_setpoint[1] - position[1]
            
            # Position errors to angle setpoints
            # Note: pitch controls x, roll controls y (with sign flip)
            self.pitch_setpoint = self.x_pos_pid.update(pos_error_x, dt)
            self.roll_setpoint = -self.y_pos_pid.update(pos_error_y, dt)
            
            # Altitude control
            altitude_error = self.altitude_setpoint - position[2]
            thrust_adjustment = self.altitude_pid.update(altitude_error, dt)
            thrust_out = self.base_thrust + thrust_adjustment
            
            # Cascade to attitude control
            roll_rate_sp = self.roll_angle_pid.update(
                self.roll_setpoint - roll, dt
            )
            pitch_rate_sp = self.pitch_angle_pid.update(
                self.pitch_setpoint - pitch, dt
            )
            
            roll_out = self.roll_rate_pid.update(
                roll_rate_sp - angular_velocity[0], dt
            )
            pitch_out = self.pitch_rate_pid.update(
                pitch_rate_sp - angular_velocity[1], dt
            )
            yaw_out = self.yaw_rate_pid.update(
                self.yaw_rate_setpoint - angular_velocity[2], dt
            )
        
        # Mix controls to motor commands
        motor_commands = self.motor_mixing(roll_out, pitch_out, yaw_out, thrust_out)
        
        return motor_commands
    
    def set_mode(self, mode: str):
        """Set flight mode and reset controllers"""
        self.flight_mode = mode
        
        # Reset all controllers on mode change
        self.roll_rate_pid.reset()
        self.pitch_rate_pid.reset()
        self.yaw_rate_pid.reset()
        self.roll_angle_pid.reset()
        self.pitch_angle_pid.reset()
        self.x_pos_pid.reset()
        self.y_pos_pid.reset()
        self.altitude_pid.reset()
        
    def arm(self):
        """Arm the flight controller"""
        self.set_mode("stabilize")
        
    def disarm(self):
        """Disarm the flight controller"""
        self.set_mode("stabilize")
        # Reset all setpoints
        self.roll_setpoint = 0.0
        self.pitch_setpoint = 0.0
        self.yaw_rate_setpoint = 0.0
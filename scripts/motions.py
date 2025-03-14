import subprocess
import time
import numpy as np

def publish_trajectory(x, y, z, yaw):
    """
    Publish position commands to PX4 via ROS 2.
    """
    command = [
        "ros2", "topic", "pub", "/custom_trajectory", "px4_msgs/msg/TrajectorySetpoint",
        f"{{ position: [ {x}, {y}, {z} ], yaw: {yaw} }}", "-1"
    ]
    subprocess.run(command, check=True)

def move_to_waypoint(target_coords, yaw, speed=1.0, step_size=0.1, tolerance=0.2):
    """
    Moves the drone smoothly to the target waypoint using position setpoints.
    - `speed`: Maximum movement speed in meters per second.
    - `step_size`: The time step in seconds for gradual movement.
    - `tolerance`: Allowed error before considering the waypoint reached.
    """
    current_coords = np.array([0.0, 0.0, 0.0])  # Replace with actual position feedback
    target_coords = np.array(target_coords)
    
    while np.linalg.norm(current_coords - target_coords) > tolerance:
        direction = target_coords - current_coords
        distance = np.linalg.norm(direction)
        
        if distance > 0:
            step = direction / distance * speed * step_size  # Move step_size seconds worth of movement
            current_coords += step  # Simulating drone movement
        
        # Publish new position setpoint
        publish_trajectory(current_coords[0], current_coords[1], current_coords[2], yaw)
        
        time.sleep(step_size)  # Simulating real-time movement

motions = [
    (0.0, 0.0, -15.0, 0.0),
    (0.0, 0.0, -10.0, 0.0),
    (20.0, 0.0, -10.0, 0.0),
    (0.0, 0.0, -10.0, 0.0)
]

for waypoint in motions:
    move_to_waypoint(waypoint[:3], waypoint[3], speed=50.0)

print("Drone movement complete!")

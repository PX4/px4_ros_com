#!/bin/bash

# Define a function to publish the trajectory messages
publish_trajectory() {
  ros2 topic pub /custom_trajectory px4_msgs/msg/TrajectorySetpoint \
    "{ position: [ $1, $2, $3 ], velocity: [ $5, $6, $7 ], yaw: $4 }" -1
}

# Define motion patterns (velocity_x, velocity_y, velocity_z, yaw)
motions=(
  "0.0 0.0 -6.0 0.0  0.0 0.0 5.0"      # Up
  "0.0 0.0 -10.0 0.0  0.0 0.0 5.0"     # Down
  "8.0 0.0 -10.0 0.0  5.0 0.0 0.0"      # Forward
  "0.0 0.0 -10.0 0.0  5.0 0.0 0.0"     # Backward
  "0.0 8.0 -10.0 0.0  0.0 5.0 0.0"      # Sideways Right
  "0.0 0.0 -10.0 0.0  0.0 5.0 0.0"     # Sideways Left
  "0.0 0.0 -5.0 3.14  "     # Rotate (yaw = 180 degrees)
  "0.0 0.0 -6.0 0.0  0.0 0.0 2.0 "      # Up
  "0.0 0.0 -10.0 0.0  0.0 0.0 2.0"     # Down
  "8.0 0.0 -10.0 0.0  2.0 0.0 0.0"      # Forward
  "0.0 0.0 -10.0 0.0  2.0 0.0 0.0"     # Backward
  "0.0 8.0 -10.0 0.0  0.0 2.0 0.0"      # Sideways Right
  "0.0 0.0 -10.0 0.0  0.0 2.0 0.0"     # Sideways Left
  "0.0 0.0 -10.0 3.14  "     # Rotate (yaw = 180 degrees)
)


publish_trajectory 0 0 -3.0 0 0 0 5
sleep 10
# Repeat each motion 4 times
for i in {0..6}; do
    # Get motion parameters (velocity_x, velocity_y, velocity_z, yaw)
    IFS=' ' read -r x y z yaw vx vy vz <<< "${motions[$i]}"
    
    # Publish trajectory for the current motion
    echo "Publishing motion $((i+1))"
    publish_trajectory "$x" "$y" "$z" "$yaw" "$vx" "$vy" "$vz"
    
    sleep 10
done

echo "Drone movement complete!"
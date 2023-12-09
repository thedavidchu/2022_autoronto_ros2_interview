# Bring 'ros2' command into scope

cd ../../

# Build package
colcon build --packages-select autoronto_interview

# Source the setup files
./install/setup.bash

# Run
ros2 run autoronto_interview davidchus_test &
ros2 run autoronto_interview solution


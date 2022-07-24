# Bring 'ros2' command into scope
source /opt/ros/humble/setup.bash;

# Check dependencies
cd ~/dev_ws/src/autoronto_interview/;
rosdep install -i --from-path src --rosdistro humble -y;

# Build package
colcon build --packages-select autoronto_interview;

# Source the setup files
. install/setup.bash;

# Run
if [[ $1 == "solution" ]]; then
    ros2 run autoronto_interview solution;
else
    ros2 run autoronto_interview davidchus_test;
fi
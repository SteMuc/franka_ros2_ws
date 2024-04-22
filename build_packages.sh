source /ros_entrypoint.sh
sudo rm -rf build devel log
colcon build --symlink-install
source install/setup.bash

if ! grep -q "source /workspaces/install/setup.bash" ~/.bashrc; then
    # If not present, add the line to ~/.bashrc
    echo "source /workspaces/install/setup.bash" >> ~/.bashrc
fi
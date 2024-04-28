source /ros_entrypoint.sh
sudo rm -rf build devel log
sudo rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys libfranka
colcon build --symlink-install

source install/setup.bash
#source workspaces
echo 'Source workspace'

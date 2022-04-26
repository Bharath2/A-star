sudo apt install ros-noetic-moveit
sudo apt install python3-wstool
mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src
git clone https://github.com/ros-planning/moveit_tutorials.git -b master
git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel
cd ~/ws_moveit/src
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
## incase of error
sudo sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" >
/etc/apt/sources.list.d/ros-latest.list'
sudo apt update
##
cd ~/ws_moveit
catkin build
(or)
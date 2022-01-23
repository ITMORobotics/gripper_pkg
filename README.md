# gripper_pkg
Repo for controlling gripper via ROS service
## Installation
1. Install `pyserial` via command:
`pip3 install pyserial`
2. Create `catkin_ws/src` directory via command:
`mkdir - p ~/catkin_ws/src`
3. Clone project into this directory:
`git clone git@github.com:ITMORobotics/gripper_pkg.git`
4. Return into `~/catkin_ws` and build project:
`sudo catkin_make`
5. Source `setup.bash` file:
 `source ~/catkin_ws/devel/setup.bash`
6. Add user in `dialout` group:
`sudo adduser your_user dialout`
7. Log out and log back in

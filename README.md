# ARF_CONTROLLER
An Artificial Potential Field Controller is built to drive multi robot system from one position to another without colliding with each other.The system is simulated in Gazebo. 

# Create a Workspace
mkdir -p ~/apf_ws/src

cd ~/apf_ws/

catkin_make

# Source your Workspace
source devel/setup.bash

**Note:** Either put it permanently in bashrc or source it for every new terminal opened.

# Clone this repository in your workspace source folder
git clone https://github.com/Tashmoy966/ARF_CONTROLLER.git

# Clone the official volta ROS package into your source folder

**Note:** Its better to keep every thing in a common directory eg volta

git clone https://github.com/botsync/volta.git

git cone https://github.com/botsync/volta_simulation.git (Simulation Essentials)

# Launch the Multi Robot System

Start the roscore

**Launching MRS Group** : roslaunch airl_mrs start.launch

**Start the ARF controller** :rosrun airl_mrs apf_controller.py --goal_center 10.0 10.0



**Note**: Due to ros distro version conflicts sometimes control spawner doesn't works for simulating volta robot in Gazebo 

An alternative is to use other robot models to testing 

**Turtelbot3** can be used:

# Installing Tb3 ROS Packages to your source folder

git clone https://github.com/ROBOTIS-GIT/turtlebot3.git

git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git

# Build Your Workspace
catkin_make

# Launch the Multi Robot System

Start the roscore

**Launching MRS Group** : roslaunch airl_mrs main.launch

**Start the ARF controller** :rosrun airl_mrs apf_controller.py --goal_center 10.0 10.0


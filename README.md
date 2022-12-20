
# robot_tester
The goal of this package is to test if we can integrate our solution with your robots. 

  ### Requirements to Run Test Package

1. This package runs on **ROS Melodic** on **Ubuntu 18.04**
2. Installing *pika* package. You can do that by executing `pip install pika`
3. Running the navigation stack of your robot
4. After running the navigation stack, you should have access to these rostopics
	- **/cmd_vel:** (topic of type twist to send velocity commands to make the robot moves.)
	- **/amcl_pose:** (topic from move_base navigation stack to get the current position of the robot.)
- **Note**: If you could not access these rostopics, **please contact us**. 

### Steps to Run Test Package

1. First, clone the ros package inside ***catkin_ws/src***
	```bash
	cd ~/catkin_ws/src
	git clone https://github.com/riotu-lab/robot_tester.git
	```
	
2. Then, you need to compile your workspace
	```bash
	cd ~/catkin_ws
	catkin_make
	source  ~/.bashrc
	```
	
3. Finally, launch the package using
	```bash
	roslaunch robot_tester test.launch host_value:=167.172.94.216 port_value:=5672
	```
	
### Next Step
After executing all of these commands with success, please coordinate a meeting with us to test if the robot is able to receive motion commands from us and moves.
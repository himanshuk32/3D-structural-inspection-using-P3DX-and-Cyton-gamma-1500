# cyton_1500_controllers
Dynamixel controllers for ROS Indigo allowing position control of Dynamixel chains or robotic manipulators

## See SteveMacenski/cyton_gamma_300-1500_operation_and_simulation for use and full robot launch instructions. 
https://github.com/SteveMacenski/cyton_gamma_300-1500_operation_and_simulation

## Controllers
The gripper controllers and manipulator controller configurations are stored in yaml files in the main directory. The min/max/init values were tuned for my specific robots. They should be good estimates for yours as well but I recommend finding the extreme values for each axis and updating.

## launch
> roslaunch cyton_1500_controllers controller_manager.launch

Will bring up a connection to the hardware.

> roslaunch cyton_1500_controllers start_controller.launch

Will bring up the controllers in the main namespace for feedback and commanding.

> rosrun cyton_1500_controllers dynamixel_joint_state_publisher.py 

Will bring up the state publisher at 1 Hz (configurable) in the /joint_states topic.

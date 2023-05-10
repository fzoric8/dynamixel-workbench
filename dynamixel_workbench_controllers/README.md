# dynamixel_workbench 


ROS package for dynamixel workbench robot manipulator. 


## Motivation

Motivation is to develop dynamixel control so we're able to control robotic arm. 


## Current status

It is possible to execute simple trajectories. There are multiple problems present at the moment. 

Problems are: 
 - It's not yet clear how to command motors independently 
 - Revolution direction is not consistent 
 - Integration with the UAV


## Used links

[dynamixel_ros_control](http://resibots.eu/dynamixel_control_hw/)  
[dynamixel_position_control](http://wiki.ros.org/action/fullsearch/Camera%2BDynamixelRobotSample/dynamixelpositioncontrol?action=fullsearch&context=180&value=linkto%3A%22Camera%2BDynamixelRobotSample%2Fdynamixelpositioncontrol%22)  
[dynamixel_metapackage_control](http://wiki.ros.org/action/fullsearch/Camera%2BDynamixelRobotSample/dynamixelpositioncontrol?action=fullsearch&context=180&value=linkto%3A%22Camera%2BDynamixelRobotSample%2Fdynamixelpositioncontrol%22)  
[create_joint_position_controller](http://wiki.ros.org/dynamixel_controllers/Tutorials/CreatingJointPositionController)
[quick_start_guide](https://www.youtube.com/watch?v=SpdxjsCO9sE&ab_channel=ROBOTISOpenSourceTeam)  


### Use SDK to run simple node
``` 
rosrun dynamixel_sdk_examples read_write_node
```
Publish joint position on the topic: 
```
rostopic pub -1 /set_position_dynamixel_sdk_examples/SetPosition "id: 1, position: 500"
```

## Control table 

In order to control arm it is neccessary to setup dynamixel Control Table. 

Motor Control Table can be found [here](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/). 

# TODO: 

- [x] Explore OpenManipulator --> new OpenManipulator doesn't have MoveIt!
- [x] Find out how to enable `joint_trajectory_controller` interface for the currently used manipulator
- [x] Add joint_states for moveit
- [x] Fix motor direction
- [ ] Add offset fix
- [ ] Find out how to control end effector position (roscpp/moveit node for commanding `arm` move_group)
- [ ] Continuous planning/servoing 

# omni_ros
ROS Integration for our Youbot + arm robots.

<img src="http://www.resibots.eu/_images/omnigrasper.jpg"/>

## Packages

### omni_bringup

Launch and configuration files to start the robot in its different versions. Currently, the three main ones are

- `omnigrasper.launch`
  - arguments
    - use_base (bool): enable the Youbot
    - use_arm (bool): enable the arm
    - use_mocap (bool): launch the node for motion capture integration
    - arm_control_mode (string) either velocity, position or safevel
  - what it does
    - load URDF description the TF tree
    - start a control node for the arm (dynamixel hardware interface)
    - start a control node for the YouBot mobile base (youbot driver)
    <!-- - TODO: Topics published
    - TODO: Subscribed topics -->
- `omni_mocap.launch`
  - starts the ROS node to receive motion capture data on Omnigrasper
- `velocity_teleop.launch`
  - use a joypad to command the arm, assuming it is in safevel mode

### omni_controllers

Controllers specifically built for this robot, for instance the `omni_controllers/OmnigrasperSpeedSafeController` that commande the arm in velocity, ensuring that it never hits the floor or the mobile base.

### omni_description

Has the URDFs of the robot in its different configurations.

### omni_driver

Some C++ glu code to command the arm.

## Documentation/Dependencies

| launch/node | required ros packages | other requirements |
| --- | --- | --- |
| omni_mocap | [mocap_optitrack](https://github.com/resibots/mocap_optitrack) | &nbsp;|

**Work stalled**

## Authors

- Maintainer: Dorian Goepp
- Authors: Federico Allocati, Dorian Goepp

## License

[CeCILL]

[CeCILL]: http://www.cecill.info/index.en.html
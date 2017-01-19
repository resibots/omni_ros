# omni_ros
ROS Integration for our omnidirectional robots.

## Packages


### omni_bringup
There are two launch files and their associated configuration files:

#### `omnipointer.launch`
does the following
- load URDF description the TF tree
- control node for the arm (dynamixel hardware interface)
- control node for the YouBot mobile base (youbot driver)

Topics published
  TODO

Subscribed topics
  TODO

#### `omni_mocap.launch`
start the ROS node to receive motion capture data on Omnigrasper

### omni_description
### omni_driver
### omni_ros

## Documentation/Dependencies

| launch/node | required ros packages | other requirements |
| --- | --- | --- |
| omni_mocap | [mocap_optitrack](https://github.com/resibots/mocap_optitrack) | &nbsp;|

**Work stalled**

## Authors

- Maintainer: Dorian Goepp
- Author: Federico Allocati

## License

[CeCILL]

[CeCILL]: http://www.cecill.info/index.en.html
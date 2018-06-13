# omni_description

#### Omnidirectional robots descriptions in form of URDF files

## How to generate the URDF models

In `robots/` you can find generated URDF models for use in ros-independent simulators. These need to be re-generated every time the original Xacro models are changed. An example for `omnigrasper_arm_only.urdf` (from the `omni_description` folder):

```
rosrun xacro xacro --inorder robots/omnigrasper_arm_only.urdf.xacro > robots/omnigrasper_arm_only.urdf.
```

## Authors

- Author/Maintainer: Federico Allocati

## Available models

- **omnipointer**: The full omnipointer, with base, plate and arm
- **omnipointer_arm_only**: The omnipointer arm attached to the ground

## Dependencies
- [Youbot Description]: Our own version of the Youbot Description ROS package. The indigo-devel branch is used.

## LICENSE

[CeCILL]

[CeCILL]: http://www.cecill.info/index.en.html
[Youbot Description]: https://github.com/resibots/youbot_description

# MyT-Controller
Second assignment for Robotics course @ USI 21/22.

**Final Score:** 9,00/10

## Prerequisites
- Python 3
- ROS2 (Galactic)
- CoppeliaSim

## Details
The homework is divided into 3 tasks, that are incremental:

1. Write an open-loop controller that moves the MyT in such a way that it follows an "8" trajectory. Test it in the default empty scene.

2. Using the wall scene file, write a controller to move the MyT straight ahead. Note that this scene rotates randomly the wall every time it is reset. We assume the robot is heading toward a wall somewhere in front of it; the wall is not necessarily orthogonal to the direction the MyT is originally pointing to. Write the controller in such a way that the MyT moves straight ahead until it is close to the wall (without hitting the wall), then turns in place in such a way to face the wall as precisely as possible (i.e., the robot's x-axis should be orthogonal to the wall). To sense the wall orientation once you are close to it, you should use proximity sensors. Feel free to define a convenient distance threshold at which you decide to stop.

3. Using the controller built in task 2, once the MyT arrives close to the wall, it should then turn in such a way that it is facing opposite to the wall, then move and stop in such a way that the robot is as close as possible to a point that is 2 meters away from the wall. Note that the proximity sensors don't have such a long range, so at some point, you'll have to rely on odometry.

## Implementation

All 3 mandatory exercises have been implemented

In order to launch one of these, run the following commands:

1. Run CoppeliaSim (in the case of the last 2 exercises load also the scene)

2. Launch the bridge `ros2 launch thymioid main.launch device:="tcp:host=localhost;port=33333" simulation:=True name:=thymio0`

3. `ros2 launch ali_thymio working_3.launch.py thymio_name:=thymio0`
(This command is launching the most advanced task implemented, to launch the first and second task, instead of “working_3” substitute “working_1” or “working_2”)

<img src="https://github.com/arstek131/MyT-Controller/blob/main/example_screen.png"  style="width: 40vw; min-width: 140px;"/>

# ASR-0-AvoidObstacle
Práctica 0 - Arquitecturas Software para Robots 2023

## Goals
Create an autonomous robot machine using a finite state machine (FSM) to avoid obstacles:
1. The robot starts motionless, and starts its behavior when a button on the robot is pressed.
2. The robot advances until it finds an obstacle less than one meter in front of it.
3. When it find an obstacle, the robot turns 90 degrees, and makes an arc movement to go over it.
4. If while you are making the arc, you find a new obstacle, do the same thing from point 3 again.

## Implementation ⚙️ 
To get hands-on with this practice, the first thing we did was sketch a state diagram in which you could see at a glance the different states through which the robot would go through in its operation.
![image](https://user-images.githubusercontent.com/102520602/220710032-3e1737e7-7e79-4a19-873f-d2f558d0b4ee.png)

This was the first sketch we did and as you can see there are the states: stopped, go forward, obstacle turn, arc turn, turn exit.

We also made a sketch of the node diagram
![image](https://user-images.githubusercontent.com/102520602/220708277-a76c78c3-ae46-4199-93d8-09a805fcfab1.png)

Later we thought that in order for the practice to be more complete, we could add some more things that would provide the robot with better functioning in addition to what was already requested
.
### Additional implementations to practice 👨‍🔧 
  - **Bumper:** To make the detection of the robot as accurate as possible we decided to take readings from the bumper because the laser only detects things that are at the same height as it, but does not detect anything below it. For this reason, we helped with the bumper because it could detect any object that the laser does not capture below it. When the robot crash with an undetected object, it moves back a few meters and begins to make an arc to try to avoid the obstacle as if it had been detected by the laser.
  
  - **Wheel drop:** We also decided to add the wheel drop topic to know when the robot is in the air because a person has lifted it. When it is detected that the wheels have been lifted, it stops walking and returns to the start state. This allows a greater handling of the robot in unexpected situations.
 
  - **Leds and sounds:** To make it easier and more comfortable to debug the robot's status, we decided to add colored leds and sounds to indicate what it was doing on inside the robot in each case. For example, when the robot changes state, it emits a certain sound or when the robot detects an unexpected object, the led turns red.


  - **New node:** (completar luego)


### State diagram 📊 
Since we decided to create new features and new states, we had to restructure the state diagram. Some of the states were renamed and others created from scratch. In addition, the Lidar node was added with the leds apart from the main node.

![image](https://user-images.githubusercontent.com/102520602/220708440-e7275a4a-ae5e-452b-82c2-cbe4f51f5af9.png)

We have also created a node diagram to better show how everything works inside

![image](https://user-images.githubusercontent.com/102520602/220709216-bfa4a8a9-f4bf-4d44-aa6e-132700800bf3.png)

## Observations 🔎 
  - Grip change: During the practice we realized that the robot did not walk the same on the floor and on the carpet. This is due to the fact that they have different friction and influence the movement of the robot to a greater or lesser extent. This causes the robot to not be able to perform the turns well in some cases.
  
  - Sensor remaping: For the correct functioning of the robot and its correct implementation, we decided to remap the topics as can be seen in the **avoid_obstacle.launch.py** file.
 ```python
 remappings=[
                        ('input_scan', '/scan_filtered'),
                        ('output_vel', '/cmd_vel'),
                        ('input_button', '/events/button'),
                        ('lidar_led', '/commands/led2'),
                        ('status_led', '/commands/led1'),
                        ('output_sound', '/commands/sound'),
                        ('input_bumper', '/events/bumper'),
                        ('input_wheel_drop', '/events/wheel_drop')
                      ])
```

  - Parameters in params.yaml: Also, we put all the parameters that the robot uses to work in a file called params.yaml. From this file you can easily configure the parameters such as the speed, the angle of rotation, the distance to the obstacle... 

```c++
lidar_led_feedback:
  ros__parameters:
    obstacle_distance: 1.0

avoid_obstacle:
  ros__parameters:
    yaw_time: 2.5
    back_time: 1.0
    dodge_time: 13.0
    scan_timeout: 1.0
    speed_linear: 0.3
    speed_angular: 0.9
    speed_linear_factor: 0.8
    speed_angular_factor: 0.4
    obstacle_distance: 1.0
```

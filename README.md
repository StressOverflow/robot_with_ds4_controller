# Map with controller 🎮

[![distro](https://img.shields.io/badge/Ubuntu%2022-Jammy%20Jellyfish-violet)](https://releases.ubuntu.com/22.04/)
[![distro](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Language](https://img.shields.io/badge/Language-C%2B%2B-orange)](https://isocpp.org/)
[![main](https://github.com/dgarcu/mapwithcontroller/actions/workflows/colcon_tests.yaml/badge.svg?branch=kobuki-main)](https://github.com/dgarcu/mapwithcontroller/actions/workflows/colcon_tests.yaml)

## Motivation 💡

We ([StressOverflow]), as students of [Universidad Rey Juan Carlos], are starting to cut our teeth with [Nav2](https://navigation.ros.org/). Basically, the first step you have to accomplish if you want to navigate is to have a map to navigate with!

We have been told that the best practice is to map manually by moving around the robot so that the perceived environment is as realistic as possible. So we started to operate the robot using the keyboard of our laptops; however, there was a problem. We are currently using [TurtleBot](https://www.turtlebot.com/turtlebot2/), and we lay our laptops right above the robot. So, in order to access the keyboard, we have to lay down every few steps to maneuver the robot. After a couple of hours, we realized that we will not be [*Forever Young*](https://www.youtube.com/watch?v=mwG6g5boyF4), so we need to take care of our backs!

That was when we thought of using a controller to maneuver the robot from a comfortable and civilized position... And here we are.

https://user-images.githubusercontent.com/92941081/233775494-aa1f4994-df3c-4415-9400-04e96f5c6f14.mp4

## Installation 💾

### Main requirements ✅

1. First of all, we need [Ubuntu](https://ubuntu.com/). It is a Linux Debian-based operating system, A.K.A Linux Distro. We are working with the 
**[22.04 Jammy Jellyfish 🪼](https://releases.ubuntu.com/22.04/)** version. You might be already familiar with it, however 
[here](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview) you have a step-by-step tutorial on how to install it on your machine.

2. [ROS](https://www.ros.org/). ROS stands for *Robot Operating System*, and it is the **middleware** that the vast majority of robots out there use 
nowadays, and so are we! We are focusing on the latest stable release, which is **[ROS 2 Humble Hawksbill 🐢](https://docs.ros.org/en/humble/index.html)**. 
From there you can navigate through all the documentation, but [here](https://docs.ros.org/en/humble/Installation.html) is a shorcut to the installation
page.

### Dependencies

- [Python](http://python.org/) 2.7 or 3.3+ (for Debian/Ubuntu you need to
  install the *python2.7-dev* or *python3.3-dev* package)
- [python-setuptools](https://pythonhosted.org/setuptools/)
- hcitool (usually available in the *bluez-utils* or equivalent package)

These packages will normally be installed automatically by the setup script,
but you may want to use your distro's packages if available:

- [pyudev](http://pyudev.readthedocs.org/) 0.16 or higher
- [python-evdev](http://pythonhosted.org/evdev/) 0.3.0 or higher


### Stable release

Installing the latest release is simple by using [pip](http://www.pip-installer.org/):

``` bash
sudo pip install ds4drv
```
Once we have everything installed, clone this repo to your `src` path, like so:

```bash
cd <your-workspace-path>/src
git clone https://github.com/dgarcu/mapwithcontroller.git
```

Once finished, you need to import the  **third party** repos that we will need. This will clone into your `ThirdParty` dir the [**ds4_driver package**](https://github.com/naoki-mizuno/ds4_driver.git):

```bash
vcs import --recursive < mapwithcontroller/thirdparty.repos
```

### Installation 

This driver depends on `ds4drv`. Some features of this driver depend on pull
requests have not yet been merged upstream. Until they are merged, use
[`naoki-mizuno/ds4drv`](https://github.com/naoki-mizuno/ds4drv/tree/devel)
(`devel` branch).

```bash
cd ThirdParty/ds4drv
mkdir -p ~/.local/lib/python3.10/site-packages
python3 setup.py install --prefix ~/.local
sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sed -i '5d;6d;15d' setup.py

```

Compile and source this package just like any other ROS package. To run,

```bash
ros2 launch map_with_controller controller.launch.py

```

### Using 💭

ds4drv has two different modes to find DS4 devices, decide which one to use
depending on your use case. Raw bluetooth mode and hidraw mode. The fisrt one 
is not supported yet beacuse the bluetooth protocolos are not as fast as de USB
portocols. So we use Hidraw mode

This mode uses the Linux kernel feature *hidraw* to talk to already existing
devices on the system.

To use the DS4 via USB in this mode, simply connect your DS4 to your computer via
a micro USB cable.
  
When the controller is paired, the launcher will show it, then you can start to use it.

### Controls 🕹️

There is a dead-man button added as a safety feature. The robot will only move while said button is pressed.

- dead-man button: **X**
- Clear emergencies: **L1** + **R1**
- Move forward: **R2**
- Move backward: **L2**
- Turn: **Left joystick**

There is a `timestamp` with the last message received from the controller. If this `timestamp` exceeds a reasonable time, the robot will be stopped until new messages are detected. This could happen when the controller suddenly disconnects from the PC.

**UPDATE**. Now you will be able to change this timeout in the [params file](./config/params.yaml).

> Be **SPECIALLY CAREFUL** with this parameter, since it is the only number between you and a runaway robot. If the controller is disconnected, the robot will continue to move with the last speed commanded. You have been warned!

### User feedback

The controller will use some signals to indicate the status of the node.

#### Controller connected.

The controller will softly rumble for `1s`. Also, the LED bar will iluminate in steady blue light. 🔵

> This state will not change until the first press of the *dead-man* button.

![blue](./doc/img/blue.gif)

#### Controller enabled.

The controller will hardly rumble for `25ms`. Also, the LED bar will iluminate in steady green light. 🟢

> This is the only mode where the robot will actually move! Is activated by **HOLDING** the *dead-man* button.

![green](./doc/img/green.gif)

#### Controller disabled.

The controller will iluminate it's LED bar with slowly blinking yellow light. 🟡

> After `60s` in this state, the controller will change to `IDLE` mode. The light will became steady dim blue 🔵.

![blinking_yellow](./doc/img/blinking_yellow.gif)

#### Controller error

The controller might enter to this state from several events, like bump into something, reaching a cliff or being lifted from the ground.

The controller will iluminate it's LED bar with fast blinking red light. 🔴

> You will see in the terminal more detailed instructions about how to exit from this state.

![blinking_red](./doc/img/blinking_red.gif)

### Kobuki LEDs

There are two progammable LEDs onboard, which are used to display the node status.

- **LED 1** is used for the controller itself.
- **LED 2** is used for the *permissions* of the controller.

|         |   LED S   |   LED 1   |   LED 2   |
|:-------:|:---------:|:---------:|:---------:|
| `ControllerState::CONNECTED` |     🟢     |      🟢     |      ⚫     |
| `ControllerState::DISCONNECTED` |     🟢     |      🔴     |      ⚫     |
| `ControllerState::IDLE` |     🟢     |      🟡     |      ⚫     |
| `ControllerState::ENABLED` |     🟢     |      🟢     |      🟢      |
| `ControllerState::DISABLED` |     🟢     |      🟢     |      🟡      |
| `ControllerState::ERROR` |     🟢     |      🟢     |      🔴     |

- `ControllerState::CONNECTED`: The controller is connected to the node.
- `ControllerState::DISCONNECTED`: The controller is disconnected from the node.
- `ControllerState::IDLE`: The controller is connected, but has not been touch for a while.
  > This state is only reachable from `ControllerState::CONNECTED` or `ControllerState::DISABLED`.
- `ControllerState::ENABLED`: The controller is connected, and it is allowed to command the robot.
- `ControllerState::DISABLED`: The controller is connected, and it is **not** allowed to command the robot.
- `ControllerState::ERROR`: The controller is conneccted, but it is some error or emergency which prevents the robot to move.

Along with the LED's, for each state is also associated a sound.

> You can disable either the LED's or the Sound through the [params file](./config/params.yaml).

### Params

In [params.yaml](./config/params.yaml) you will be able to modify the maximum velocity of the robot.

**However, as a security feature, it will be clamped by a maximum absolute velocity which is hardcoded.**

> **Friendly reminder** The average walking speed for a human is around 1-1.5 m/s. Remember that you are a human wired to a robot which uses your laptop as a hat.

```yaml
controller_node:
  ros__parameters:
    max_linear_vel: 0.5
    max_angular_vel: 1.0
    controller_timeout: 0.25
    enable_leds: true
    enable_sounds: true
```

## Features

https://github.com/naoki-mizuno/ds4_driver.git

https://github.com/naoki-mizuno/ds4drv.git

https://github.com/chrippa/ds4drv.git

## Tests 🧾

This package was tested on the following scenarios:

### Original DS4 Controller

|         |   Kobuki   |   Tiago   |
|:-------:|:---------:|:---------:|
| Real World 🌍 |     ✔️     |      〰️     |
| Simulation 🖥️ |     ✔️     |      ✔️     |


https://user-images.githubusercontent.com/92941081/233832351-5fe179eb-ce52-4793-bb3d-00435851b09b.mp4


https://user-images.githubusercontent.com/92941081/233832354-9eb7d7fc-cf8a-46e5-88b8-9888fea8fedc.mp4


## Importat notes

- **This package was only tested with an original DS4 Controller via USB. Bluetooth was not tested. DS4 Compatible Controllers were not tested either.**

## Future improvements ✔️

- [x] Parameters. The maximum speed of the robot in each of the maneuvers should be modifiable without the need to recompile the code. Other types of parameters of interest would also be added if needed.
- [x] "Dead-man" button. The controller should only work if one of the buttons is held down, as a safety feature.
- [x] Feedback. The controller should notify the user when it is ready to operate the robot, either with lights or vibration, as well as the output of the terminal.
- [x] Kobuki dedicated branch. Although the package is designed for any robot, a specific branch for the Kobuki TurtleBot would allow it to use its resources, such as the speaker or the LEDs, to emit signals and feedback to the user.

## About

This is a project made by the [StressOverflow], a student group from the [Universidad Rey Juan Carlos].
Copyright &copy; 2023.

Maintainers:

- [Carlos Escarcena] (c.escarcena.2021@alumnos.urjc.es)
- [Arantxa García] (a.garciab.2021@alumnos.urjc.es)
- [Diego García] (d.garciac.2021@alumnos.urjc.es)
- [Teresa Ortega] (mt.ortega.2021@alumnos.urjc.es)

## License

[![License](https://img.shields.io/badge/License-Apache_2.0-yellowgreen.svg)](https://www.apache.org/licenses/LICENSE-2.0) 

This work is licensed under a [APACHE LICENSE, VERSION 2.0][apache2.0].

[apache2.0]: https://www.apache.org/licenses/LICENSE-2.0

[Universidad Rey Juan Carlos]: https://www.urjc.es/
[StressOverflow]: https://github.com/orgs/Docencia-fmrico/teams/stressoverflow
[Carlos Escarcena]: https://github.com/cescarcena2021
[Arantxa García]: https://github.com/arantxagb
[Diego García]: https://github.com/dgarcu
[Teresa Ortega]: https://github.com/mtortega2021

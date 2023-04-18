# Mapwithcontroller

[![distro](https://img.shields.io/badge/Ubuntu%2022-Jammy%20Jellyfish-violet)](https://releases.ubuntu.com/22.04/)
[![distro](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Language](https://img.shields.io/badge/Language-C%2B%2B-orange)](https://isocpp.org/)

<p align="center">
  <img src="https://raw.githubusercontent.com/kobuki-base/kobuki_core/devel/resources/kobuki.png"/>
</p>

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
    $ sudo pip install ds4drv
```
Once we have everything installed, clone this repo to your `src` path, like so:

```bash
cd <your-workspace-path>/src
git clone https://github.com/dgarcu/mapwithcontroller.git
```

Once finished, you need to import the  **third party** repos that we will need. This will clone into your `ThirdParty` dir the [**ds4_driver package**](https://github.com/naoki-mizuno/ds4_driver.git):

```bash
vcs import < map_with_controller/thirdparty.repos
```

### Installation 

This driver depends on `ds4drv`. Some features of this driver depend on pull
requests have not yet been merged upstream. Until they are merged, use
[`naoki-mizuno/ds4drv`](https://github.com/naoki-mizuno/ds4drv/tree/devel)
(`devel` branch).

```console
$ cd Thirdparty/ds4drv
$ mkdir -p ~/.local/lib/python3.10/site-packages
$ python3 setup.py install --prefix ~/.local
$ sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```

Compile and source this package just like any other ROS package. To run,

```console
$ ros2 launch map_with_controller controller.launch.py
```

### Using

ds4drv has two different modes to find DS4 devices, decide which one to use
depending on your use case. Raw bluetooth mode and hidraw mode. The fisrt one 
is not supported yet beacuse the bluetooth protocolos are not as fast as de USB
portocols. So we use Hidraw mode

This mode uses the Linux kernel feature *hidraw* to talk to already existing
devices on the system.

To use the DS4 via USB in this mode, simply connect your DS4 to your computer via
a micro USB cable.
  
When the controller is paired, the launcher will show it, then you can start to use it.

### Controls

- Move forward: **R2**
- Move backward: **L2**
- Turn: **Left joystick**

## Features

https://github.com/naoki-mizuno/ds4_driver.git

https://github.com/naoki-mizuno/ds4drv.git

https://github.com/chrippa/ds4drv.git

## License

Apache 2.0

## Authors

Carlos Escarcena (c.escarcena.2021@alumnos.urjc.es)

Diego García (d.garciac.2021@alumnos.urjc.es)

Arantxa García (a.garciab.2021@alumnos.urjc.es)

María Teresa Ortega (mt.ortega.2021@alumnos.urjc.es)



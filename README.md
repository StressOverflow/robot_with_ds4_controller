# Mapwithcontroller

[![distro](https://img.shields.io/badge/Ubuntu%2022-Jammy%20Jellyfish-violet)](https://releases.ubuntu.com/22.04/)
[![distro](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Language](https://img.shields.io/badge/Language-C%2B%2B-orange)](https://isocpp.org/)
[![main](https://github.com/Docencia-fmrico/seekandcapture-stressoverflow/actions/workflows/colcon.yaml/badge.svg?branch=SeekAndCapture)](https://github.com/Docencia-fmrico/seekandcapture-stressoverflow/actions/workflows/colcon.yaml)

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

´´´bash

    $ sudo pip install ds4drv
´´´

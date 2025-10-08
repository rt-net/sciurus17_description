[English](README.en.md) | [日本語](README.md)

# sciurus17_description

[![industrial_ci](https://github.com/rt-net/sciurus17_description/actions/workflows/industrial_ci.yml/badge.svg?branch=ros2)](https://github.com/rt-net/sciurus17_description/actions/workflows/industrial_ci.yml)

ROS 2 package with URDF description macro for [Sciurus17](https://rt-net.jp/products/sciurus17/).

> [!NOTE]
> This ROS package was separated from [rt-net/sciurus17_ros](https://github.com/rt-net/sciurus17_ros).
> See [rt-net/sciurus17_ros#134](https://github.com/rt-net/sciurus17_ros/issues/134) for details.

## Table of Contents

- [sciurus17\_description](#sciurus17_description)
  - [Supported ROS distributions](#supported-ros-distributions)
  - [Requirements](#requirements)
  - [Installation](#installation)
  - [How to Use](#how-to-use)
  - [Proprietary Rights](#proprietary-rights)

## Supported ROS distributions

### ROS 2

- [Humble Hawksbill](https://github.com/rt-net/sciurus17_description/tree/humble)
- [Jazzy Jalisco](https://github.com/rt-net/sciurus17_description/tree/jazzy)

## Requirements

- OS
  - Ubuntu Desktop 24.04
- ROS 2
  - Jazzy Jalisco

## Installation

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# Clone package
git clone -b $ROS_DISTRO https://github.com/rt-net/sciurus17_description.git

# Install dependencies
rosdep install -r -y -i --from-paths .

# Build & Install
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

## How to Use

Display a Sciurus17 robot model on RViz with the following command:

```sh
ros2 launch sciurus17_description display.launch.py
```

![display_launch](https://rt-net.github.io/images/sciurus17/display_launch.png)

## Proprietary Rights

Sciurus17 is an upper-body robot developed by RT Corporation for research purposes.
Please read the [license information contained in this repository](./LICENSE) to find out more about licensing.
Companies are permitted to use Sciurus17 and the materials made available here for internal, research and development purposes only.
If you are interested in building your own robot for your personal use by utilizing the information made available here, take your time to visit our website and purchase relevant components and parts – that will certainly help us keep going!
Otherwise, if you are interested in manufacturing and commercializing products based on the information herein, please contact us to arrange a license and collaboration agreement with us.

We have obtained permission from ROBOTIS Co., Ltd. to use CAD models relating to servo motors XM540 and XM430. 
The proprietary rights relating to any components or parts manufactured by ROBOTIS and used in this product, including but not limited to copyrights, trademarks, and other intellectual property rights, shall remain vested in ROBOTIS.

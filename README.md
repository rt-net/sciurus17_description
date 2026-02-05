[English](README.en.md) | [日本語](README.md)

# sciurus17_description

[![industrial_ci](https://github.com/rt-net/sciurus17_description/actions/workflows/industrial_ci.yml/badge.svg?branch=ros2)](https://github.com/rt-net/sciurus17_description/actions/workflows/industrial_ci.yml)

[Sciurus17](https://rt-net.jp/products/sciurus17/)のURDFファイルを含むROS 2パッケージです。

> [!NOTE]
> 本ROSパッケージは[rt-net/sciurus17_ros](https://github.com/rt-net/sciurus17_ros)から分離したものです。
> 
> 詳細は[rt-net/sciurus17_ros#134](https://github.com/rt-net/sciurus17_ros/issues/134)を参照してください。

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

次のコマンドを実行するとRViz上にSciurus17のモデルが表示されます。

```sh
ros2 launch sciurus17_description display.launch.py
```

![display_launch](https://rt-net.github.io/images/sciurus17/display_launch.png)

## Proprietary Rights

Sciurus17は、アールティが開発した研究用上半身ロボットです。
本リポジトリのデータ等に関するライセンスについては、[LICENSE](./LICENSE)ファイルをご参照ください。
企業による使用については、自社内において研究開発をする目的に限り、本データの使用を許諾します。
本データを使って自作されたい方は、義務ではありませんが弊社ロボットショップで部品をお買い求めいただければ、励みになります。
商業目的をもって本データを使用する場合は、商業用使用許諾の条件等について弊社までお問合せください。

サーボモータのXM540やXM430に関するCADモデルの使用については、ROBOTIS社より使用許諾を受けています。
Sciurus17に使用されているROBOTIS社の部品類にかかる著作権、商標権、その他の知的財産権は、ROBOTIS社に帰属します。

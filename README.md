# AWS DeepRacer servo package

## Overview

The AWS DeepRacer servo ROS package creates the `servo_node`, which is part of the core AWS DeepRacer application and launches from the `deepracer_launcher`. For more information about the application and the components, see the  [aws-deepracer-launcher repository](https://github.com/aws-deepracer/aws-deepracer-launcher).

This node is responsible for mapping the input servo throttle and steering angle ratios to raw PWM values and writing that to the buffer so that car moves accordingly. It also sets and gets the [min, mid, max, polarity] calibration values for steering and throttle and the tail light LED PWM values.

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation
Follow these steps to install the AWS DeepRacer servo package.

### Prerequisites

The AWS DeepRacer device comes with all the prerequisite packages and libraries installed to run the `servo_pkg`. For more information about the preinstalled set of packages and libraries on the DeepRacer and about installing the required build systems, see [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md).

The `servo_pkg` specifically depends on the following ROS 2 packages as build and run dependencies:

* `deepracer_interfaces_pkg`: This package contains the custom message and service type definitions used across the AWS DeepRacer core application.

## Downloading and building

Open a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the servo_pkg on the DeepRacer device:

        git clone https://github.com/aws-deepracer/aws-deepracer-servo-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-servo-pkg
        rosws update

1. Resolve the `servo_pkg` dependencies:

        cd ~/deepracer_ws/aws-deepracer-servo-pkg && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the `servo_pkg` and `deepracer_interfaces_pkg`:

        cd ~/deepracer_ws/aws-deepracer-servo-pkg && colcon build --packages-select servo_pkg deepracer_interfaces_pkg

## Usage

The `servo_node` provides the core functionality to combine the camera data from the cameras connected to the USB slots at the front of the AWS DeepRacer vehicle. Although the node is built to work with the AWS DeepRacer application, you can run it independently for development, testing, and debugging purposes.

### Run the node

To launch the built `servo_node` as the root user on the AWS DeepRacer device, open another terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-servo-pkg/install/setup.bash

1. Launch the `servo_node` using the launch script:

        ros2 launch servo_pkg servo_pkg_launch.py

## Launch files

The `servo_pkg_launch.py`, included in this package, provides an example demonstrating how to launch the `servo_node`.

    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='servo_pkg',
                namespace='servo_pkg',
                executable='servo_node',
                name='servo_node'
            )
        ])


## Node details

### servo_node

#### Subscribed topics

| Topic name | Message type | Description |
| ---------- | ------------ | ----------- |
|/`ctrl_pkg`/`servo_msg`|`ServoCtrlMsg`|This message holds the scaled throttle and angle values in the range of [-1, 1] that are translated based on the calibration to raw PWM values.|
|/`ctrl_pkg`/`raw_pwm`|`ServoCtrlMsg`|This message holds the raw PWM throttle and angle values to be set on the motor and servo.|

#### Services

| Service name | Service type | Description |
| ---------- | ------------ | ----------- |
|`servo_gpio`|`ServoGPIOSrv`|A service that is called enable or disable the servo GPIO pin.|
|`set_calibration`|`SetCalibrationSrv`|A service that is called to set the [min, mid, max, polarity] PWM values for the speed and steering calibration and store them in a file.|
|`get_calibration`|`GetCalibrationSrv`|A service that is called to get the [min, mid, max, polarity] PWM values for the speed and steering calibration from the saved file.|
|`set_led_state`|`SetLedCtrlSrv`|A service that is called to set the [red, green, blue] channel PWM values for the tail light LEDs and store them in a file.|
|`get_led_state`|`GetLedCtrlSrv`|A service that is called to get the [red, green, blue] channel PWM values for the tail light LEDs from the saved file.|

## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)


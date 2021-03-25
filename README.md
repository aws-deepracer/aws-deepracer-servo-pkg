# DeepRacer Servo Package

## Overview

The DeepRacer Servo ROS package creates the *servo_node* which is part of the core AWS DeepRacer application and will be launched from the deepracer_launcher. More details about the application and the components can be found [here](https://github.com/awsdeepracer/aws-deepracer-launcher).

This node is responsible to map the input servo throttle and steering angle ratios to raw PWM values and writing that to the buffer so that car moves accordingly. It also provides functionality to set and get the [min, mid, max, polarity] calibration values for steering and throttle from a file, set and get the tail light LED PWM values from a file.

## License

The source code is released under Apache 2.0 (https://aws.amazon.com/apache-2-0/).

## Installation

### Prerequisites

The DeepRacer device comes with all the pre-requisite packages and libraries installed to run the servo_pkg. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/awsdeepracer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page.

The servo_pkg specifically depends on the following ROS2 packages as build and execute dependencies:

* *deepracer_interfaces_pkg* - This packages contains the custom message and service type definitions used across the AWS DeepRacer core application.

## Downloading and Building

Open up a terminal on the DeepRacer device and run the following commands as root user.

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the servo_pkg on the DeepRacer device:

        git clone https://github.com/awsdeepracer/aws-deepracer-servo-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-servo-pkg
        rosws update

1. Resolve the servo_pkg dependencies:

        cd ~/deepracer_ws/aws-deepracer-servo-pkg && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the servo_pkg and deepracer_interfaces_pkg:

        cd ~/deepracer_ws/aws-deepracer-servo-pkg && colcon build --packages-select servo_pkg deepracer_interfaces_pkg

## Usage

The servo_node provides the core functionality to combine the camera data from the cameras connected to the USB slots at the front of the DeepRacer vehicle. Although the nodes is built to work with the AWS DeepRacer application, it can be run independently for development/testing/debugging purposes.

### Run the node

To launch the built servo_node as root user on the DeepRacer device open up another terminal on the DeepRacer device and run the following commands as root user:

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-servo-pkg/install/setup.bash

1. Launch the servo_node using the launch script:

        ros2 launch servo_pkg servo_pkg_launch.py

## Launch Files

The  servo_pkg_launch.py is also included in this package that gives an example of how to launch the servo_node.

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


## Node Details

### servo_node

#### Subscribed Topics

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
|/ctrl_pkg/servo_msg|ServoCtrlMsg|This message holds the scaled throttle and angle values in the range of [-1, 1] that will be translated based on the calibration to raw PWM values.|
|/ctrl_pkg/raw_pwm|ServoCtrlMsg|This message holds the raw PWM throttle and angle values to be set on the motor and servo.|

#### Services

| Service Name | Service Type | Description |
| ---------- | ------------ | ----------- |
|servo_gpio|ServoGPIOSrv|A service that is called enable/disable the servo GPIO pin.|
|set_calibration|SetCalibrationSrv|A service that is called to set the [min, mid, max, polarity] pwm values for the speed and steering calibration and store them in a file.|
|get_calibration|GetCalibrationSrv|A service that is called to get the [min, mid, max, polarity] pwm values for the speed and steering calibration from the saved file.|
|set_led_state|SetLedCtrlSrv|A service that is called to set the [red, green, blue] channel pwm values for the tail light LEDs and store them in a file.|
|get_led_state|GetLedCtrlSrv|A service that is called to get the [red, green, blue] channel pwm values for the tail light LEDs from the saved file.|

## Resources

* AWS DeepRacer Opensource getting started: [https://github.com/awsdeepracer/aws-deepracer-launcher/blob/main/getting-started.md](https://github.com/awsdeepracer/aws-deepracer-launcher/blob/main/getting-started.md)


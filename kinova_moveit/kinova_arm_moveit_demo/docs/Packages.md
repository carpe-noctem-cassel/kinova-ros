# Installation

## Repos hinzufügen
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt update
```

## system Pakete

```bash
sudo apt -y install git \
                    ros-melodic-desktop-full \
                    ros-melodic-actionlib \
                    ros-melodic-ar-track-alvar \
                    ros-melodic-base-local-planner \
                    ros-melodic-catkin \
                    ros-melodic-freenect-launch \
                    ros-melodic-openni-launch \
                    ros-melodic-gazebo-ros \
                    ros-melodic-gazebo-plugins \
                    ros-melodic-moveit-ros \
                    ros-melodic-trac-ik-kinematics-plugin \
                    ros-melodic-moveit-planners-ompl \
                    ros-melodic-moveit-fake-controller-manager
```

## Github repos

over https:
```bash
git clone https://github.com/Kinovarobotics/kinova-ros.git
git clone https://github.com/JenniferBuehler/common-sensors.git
```

over ssh:
```bash
git clone git@github.com:Kinovarobotics/kinova-ros.git
git clone git@github.com:JenniferBuehler/common-sensors.git
```

## Start
```bash
roslaunch kinova_arm_moveit_demo test.launch
```
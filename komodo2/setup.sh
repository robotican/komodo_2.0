#!/bin/bash

# installation file for komodo2 over ROS Kinetic and ubuntu 16.04 #

GREEN_TXT='\e[0;32m'
WHITE_TXT='\e[1;37m'
RED_TXT='\e[31m'
NO_COLOR='\033[0m'
LOGS_FOLDER_PATH="${HOME}/catkin_ws/src/setup_logs"

printf "${WHITE_TXT}\n***Installing Komodo2 ROS-Kinetic Package***\n${NO_COLOR}"
sleep 1

# validate ros version #
printf "${WHITE_TXT}\nChecking ROS Version...\n${NO_COLOR}"
sleep 1
version=`rosversion -d`
if [ "$version" == "kinetic" ]; then
  printf "${GREEN_TXT}ROS version OK${NO_COLOR}\n"
  sleep 1
else
  printf "${RED_TXT}Error: found ROS version ${version}, please install ROS Kinetic and try again${NO_COLOR}\n"
  exit 1
fi

# validate catkin_ws/src folder exist #
printf "${WHITE_TXT}\nChecking if catkin_ws/src folder exist...\n${NO_COLOR}"
sleep 1
cd ~ 
if [ ! -d "catkin_ws" ]; then
  printf "${RED_TXT}~/catkin_ws folder does not exist. Create workspace named catkin_ws and try again ${NO_COLOR}\n"
  exit 1
else
cd catkin_ws 
if [ ! -d "src" ]; then
  printf "${RED_TXT}~/catkin_ws/src folder does not exist. Create workspace named catkin_ws with src directory inside ${NO_COLOR}\n"
  exit 1
fi

fi
printf "${GREEN_TXT}found catkin_ws/src folder${NO_COLOR}\n"
sleep 1

# preparing error logs folder #
if [ ! -d ${LOGS_FOLDER_PATH} ]; then
  mkdir ${LOGS_FOLDER_PATH}
fi

# third party packages #
printf "${WHITE_TXT}Installing ROS and 3rd party packages...\n${NO_COLOR}"
{
sleep 1
# ROS packages #
sudo apt-get update
sudo apt-get dist-upgrade 
sudo apt-get upgrade 
sudo apt-get -y install ros-kinetic-controller-manager 
sudo apt-get -y install ros-kinetic-control-toolbox  
sudo apt-get -y install ros-kinetic-transmission-interface 
sudo apt-get -y install ros-kinetic-joint-limits-interface 
sudo apt-get -y install ros-kinetic-gazebo-ros-control 
sudo apt-get -y install ros-kinetic-ros-controllers 
sudo apt-get -y install ros-kinetic-ros-control 
sudo apt-get -y install ros-kinetic-moveit
sudo apt-get -y install ros-kinetic-moveit-ros-planning
sudo apt-get -y install ros-kinetic-moveit-ros-planning-interface
sudo apt-get -y install ros-kinetic-move-base
sudo apt-get -y install ros-kinetic-navigation
sudo apt-get -y install ros-kinetic-hector-slam
sudo apt-get -y install ros-kinetic-gmapping
sudo apt-get -y install ros-kinetic-twist-mux
sudo apt-get -y install ros-kinetic-pid
sudo apt-get -y install ros-kinetic-ar-track-alvar
sudo apt-get -y install joystick
sudo apt-get -y install ros-kinetic-hector-gazebo-plugins
sudo apt-get -y install ros-kinetic-serial
sudo apt-get -y install espeak espeak-data libespeak-dev
sudo apt-get -y install ros-kinetic-robot-localization
sudo apt-get -y install ros-kinetic-trac-ik ros-kinetic-moveit-kinematics 
sudo apt-get -y install ros-kinetic-urg-node 
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"
sleep 1
} 2> ${LOGS_FOLDER_PATH}/third_party.txt

# install softkinetic drivers #
printf "${WHITE_TXT}Installing softkinetic driver...\n${NO_COLOR}"
{
sleep 1
#TODO: INSTALL DEPTH CAM
sleep 1
} 2> ${LOGS_FOLDER_PATH}/softkinetic_driver.txt

# usb rules #
printf "${WHITE_TXT}Installing USB rules...\n${NO_COLOR}"
{
sleep 1
sudo apt -y install setserial #for setting port latency
sudo cp ~/catkin_ws/src/komodo2/komodo2/rules/roboteq.rules /etc/udev/rules.d
sudo cp ~/catkin_ws/src/komodo2/komodo2/rules/49-teensy.rules /etc/udev/rules.d
sudo cp ~/catkin_ws/src/komodo2/komodo2/rules/bms_battery.rules /etc/udev/rules.d
sudo cp ~/catkin_ws/src/komodo2/komodo2/rules/hokuyo.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"
sleep 1
} 2> ${LOGS_FOLDER_PATH}/usb_rules.txt

# compiling komodo2#
printf "${WHITE_TXT}Compiling komodo2 package...\n${NO_COLOR}"
{
sleep 1
cd ~/catkin_ws
catkin_make --pkg pr2_controllers_msgs robotican_msgs_srvs
catkin_make -DCMAKE_BUILD_TYPE="Release"
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"
sleep 1
} 2> ${LOGS_FOLDER_PATH}/compilation.txt

# remove empty txt files #
for f in $LOGS_FOLDER_PATH/*.txt
do
    echo "checking $f"
    if ! [ -s $f ]; then
        echo "deleting $f"
        rm -f $f
    fi
done

# if logs folder is not empty, some errors has occured #
cd $LOGS_FOLDER_PATH
if ! [ "$(ls -A $LOGS_FOLDER_PATH)" ]; then
    printf "${GREEN_TXT}Installation process finished successfully.\n\n"  ${NO_COLOR}
    printf "${GREEN_TXT}Please reboot to apply changes\n\n${NO_COLOR}"
    rmdir $LOGS_FOLDER_PATH
    exit 0
else
    printf "${RED_TXT}Installation process finished, but some error occurred. please refer to ${LOGS_FOLDER_PATH} folder for more details.\n\n${NO_COLOR}"
    exit 1
fi

alias clion='/opt/clion-2017.3.1/bin/clion.sh'

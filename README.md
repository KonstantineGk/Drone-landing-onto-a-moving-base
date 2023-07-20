# Drone-landing-onto-a-moving-base
During the Robotic Systems course, collaborated with a team to accomplish the goal of landing a Crazyflie
2.1 drone onto a moving base. The moving base consisted of an Amigobot with a specially designed plate,
beneath which an inert drone was positioned. Communication was established using the Loco positioning
system, which involved a cage with eight anchors. Implemented a Python script for control and integrated
all the components using ROS. The project was successfully deployed in real-life conditions, with the drone
landing on the randomly moving base.

This project was completed during the "Robotic Systems" course. (University of Patras)

1)First you need to install ROS in a linux system.
2) Follow this installation from https://github.com/whoenig/crazyflie_ros to install the crazyflie libraries.
3) Add the G0_n_Land.launch file in the launch files and the Go_n_Land.py file in the .py files.
4) Finally modify the launch file to fit your parameters.( Adress, etc...)
5) Roslaunch the Go_n_Land.launch file.

Description:
1) Go_n_land.py: Software to guide and land the drone to the base using PI control.
2) Go_n_Land.launch: ROS launch file to call the location topics from 2 drones and launch the above python script.

# AUT700-MePo
Repository to store AUT-700 Mechatronics and Robot Programming related exercises


## Exercise 3. Task 2.2 - turtle_bot_controller
Download the package by cloning this repository

Go to folder you want to clone this repository e.g. ~/home
> cd ~/

Clone the folder as "mepo700" and go into it and remove .git/ folder
> git clone https://github.com/Jasikainen/AUT700-MePo.git mepo700

> cd mepo700

> rm -rf .git

Copy the wanted package from cloned repository to your ~/catkin_ws/src and do catkin_make
> cp -r turtle_bot_controller/ ~/catkin_ws/src/turtle_bot_controller

> cd ~/catkin_ws/

> catkin_make


### How to perform scripts specified
Run for example "scripts/gotopoint.py" to go to a specific point in workspace

First do following to start turtlebot3 simulation
> export TURTLEBOT3_MODEL=waffle_pi

> roslaunch turtlebot3_gazebo turtlebot3_world.launch

Run from package the specific script
> rosrun turtle_bot_controller gotopoint.py <float1> <float2>



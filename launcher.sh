m#!/bin/bash


a=$(find "$(pwd)" -name  *.py ) 
chmod +x $a
gnome-terminal -x sh -c "roscore; bash"
roslaunch pet_package launcher.launch


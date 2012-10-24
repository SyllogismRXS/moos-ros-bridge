Installation Notes
==================
The MOOS/ROS Bridge was developed and tested on Ubuntu. Contact Kevin DeMarco <kevin.demarco@gmail.com> if you have difficulties installing or using the MOOS/ROS Bridge

Commands that should be entered at the terminal are denoted with a '$'

Dependencies
------------
ROS - www.ros.org
MOOS - http://oceanai.mit.edu/moos-ivp/pmwiki/pmwiki.php?n=Site.Download

Acquiring the Code
------------------
Install git  
$ sudo apt-get install git-core

I keep all of my git repositories in the same folder for organizational reasons.  Typically, I keep them in the ~/git-repos folder.  So create it...

$ mkdir ~/git-repos  
$ cd ~/git-repos

Make a clone of the repo
$ git clone git://github.com/SyllogismRXS/moos-ros-bridge.git

Building the Code
-----------------
Add your new repo to your $ROS_PACKAGE_PATH environment variable.  
$ echo "export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/git-repos/moos-ros-bridge" >> ~/.bashrc
$ . ~/.bashrc

Of course, if you didn't clone your repo into ~/git-repos/, then you will need to modify the previous command accordingly.

Reindex your ROS packages  
$ rospack profile

Build moosros_tester  
$ rosmake moosros_tester

Build moosros  
$ rosmake moosros

Launch Example
--------------
Start ROS Core  
$ roscore

In a new terminal, start the MOOS Database  
$ MOOSDB

In a new terminal, start MOOS/ROS Bridge  
$ cd ~/git-repos/moos-ros-bridge/moosros
$ rosrun moosros Bridge counters.xml bridge.moos

In a new terminal, start ROS counter 
$ rosrun moosros_tester counter

In a new terminal, observe CounterFromROS  
$ uMS  
Make sure HostName is LOCALHOST and Port is 9000. Click on "Connect"

At this point, you should see the CounterFromROS variable incrementing.

----------------------------------------
Kevin DeMarco <kevin.demarco@gmail.com>
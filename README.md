# moos-ros-bridge

A communication bridge between the MOOS and ROS publish-and-subscribe
middleware systems.

If you use the moos-ros-bridge in your research, please cite our research
paper:

    @inproceedings{demarco2011implementation,
        title={An implementation of ROS on the Yellowfin autonomous underwater vehicle (AUV)},
        author={DeMarco, Kevin and West, Michael E and Collins, Thomas R},
        booktitle={OCEANS 2011},
        pages={1--7},
        year={2011},
        organization={IEEE}
    }
    
http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=6107001&tag=1

# Build Instructions

## Install Dependencies

First, install MOOS as normal. Then, install some extra dependencies from your
package manager:

    $ sudo apt-get install librapidxml-dev

## Build in a catkin workspace

1. Setup a ROS catkin workspace, if you don't have one already:

        $ mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/
        $ catkin_make

2. Clone this repo into into ~/catkin_ws/src:

        $ cd ~/catkin_ws/src
        $ git clone https://github.com/SyllogismRXS/moos-ros-bridge.git
        
3. Build it:

        $ cd ~/catkin_ws
        $ catkin_make

# Run Example

Launch Example
===============
Start ROS Core
$ roscore

In a new terminal, start the MOOS Database  
$ MOOSDB

In a new terminal, start MOOS/ROS Bridge  
$ cd ~/repos/moos-ros-bridge/moosros  
$ rosrun moosros Bridge counters.xml bridge.moos

In a new terminal, start ROS counter  
$ rosrun moosros_tester counter

In a new terminal, observe CounterFromROS  
$ uMS  
Make sure HostName is LOCALHOST and Port is 9000. Click on "Connect"

At this point, you should see the CounterFromROS variable incrementing.

Using a ROS Launch file 
========================= 

In order to use a ros launch file with MOOS, you have to wrap the roslaunch
call and the pAntler call inside of a bash script. There is an example bash
script under /path/to/moosros_tester/scripts called counter.sh. This bash
script makes the roslaunch call to an example launch filed located at
moosros_tester/counter.launch, makes the appropriate call to pAntler, uses
MOOS' uMAC to wait for the user to kill the processes, and then kills the
roslaunch and pAntler processes.

Note the use of $(rospack find moosros) call in counter.sh to point to the
directory that holds the bridge.moos file. In counter.launch, "$(find moosros)"
is used to point to that same directory.

To test the bash script, use rosrun:

$ rosrun moosros_tester counter.sh

A uMS window will open up, press "Connect" to start monitoring variables. You
should see the CounterFromROS variable incrementing. 

At this point, the Bridge is also looking for changes in the "CounterFromMOOS"
variable in the MOOS community. Let's monitor the variable in ROS as we change
it using uMS. Open a new terminal and start rostopic echo:

$ rostopic echo /CounterFromMOOS

In the uMS window, CTRL+RightClick on an empty row. Enter the variable name:
CounterFromMOOS. Choose NUMERIC. Enter value: 42. You should see the rostopic
echo program print out the following:

data: 42
---

Bridge Configuration
========================

The Bridge is configured using an XML file that specifies the names of ROS and
MOOS topics and the associated data types. Look at
/path/to/moos-ros-bridge/moosros/counters.xml for an example. In the XML file,
each message consists of the following members:

moosname - the name of the moos variable

rosname - the name of the ros topic

moostype - the type of the moos variable

rostype - the type of the ros topic

direction - the direction of the data over the bridge (either toMOOS or toROS).


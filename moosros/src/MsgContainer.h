//* The MOOS ROS Bridge
/**
 * 
 * @file
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 *
 * @version 1.0
 *
 * @date May 25th, 2012
 * 
 * @section LICENSE
 * 
 * Georgia Tech Research Institute (GTRI)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * http://www.gnu.org/copyleft/gpl.html
 *
 * @section DESCRIPTION
 *
 * The MOOS ROS Bridge allows for communication between a 
 * MOOS Database and a ROS Core.  
 */

#ifndef _MSG_CONTAINER_H_ 
#define _MSG_CONTAINER_H_

#include "ros/ros.h"
#include <string.h>

using namespace std;

class MsgContainer{
public:
	// from MOOS to ROS
	MsgContainer(ros::Publisher new_pub, string new_moosName, string new_rosName, string new_moosType, string new_rosType){
		pub      = new_pub;
		moosName = new_moosName;
		rosName  = new_rosName;
		moosType = new_moosType;
		rosType  = new_rosType;
	}

	// from ROS to MOOS
	MsgContainer(ros::Subscriber new_sub, string new_moosName, string new_rosName){
		sub      = new_sub;
		moosName = new_moosName;
		rosName  = new_rosName;
	}

	ros::Publisher pub;
	ros::Subscriber sub;

	string moosName;
	string rosName;
	string moosType;
	string rosType;
};

#endif

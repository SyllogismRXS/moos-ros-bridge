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

#ifndef _MOOS_NODE_H_ 
#define _MOOS_NODE_H_

#include <MOOS/libMOOS/App/MOOSApp.h>
#include "ros/ros.h"
#include <vector>
#include "MsgContainer.h"
#include <string.h>

class MOOSNode : public CMOOSApp { 
public:
     //standard construction and destruction
	MOOSNode();
	virtual ~MOOSNode();

	bool toMOOS(std::string moosName, double data);
	bool toMOOS(std::string moosName, std::string myString);
    bool toMOOSBinaryString(std::string moosName, std::string myString);
	void AssignPublisher(vector<MsgContainer> *msgVec);

protected:
	double appTick;
	double commsTick;

	//where we handle new mail
	bool OnNewMail(MOOSMSG_LIST &NewMail);

	//where we do the work
	bool Iterate();

	//called when we connect to the server
	bool OnConnectToServer();

	//called when we are starting up..
	bool OnStartUp();

void DoRegistrations();

private:
	vector<MsgContainer> *msgVec;

};
#endif

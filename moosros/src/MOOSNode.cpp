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

#include "MOOSNode.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "MsgContainer.h"
#include <vector>

//default constructor
MOOSNode::MOOSNode()
{ 
}

void MOOSNode::AssignPublisher(vector<MsgContainer> *new_msgVec){
     msgVec = new_msgVec;
}

//default (virtual) destructor 
MOOSNode::~MOOSNode(){
}

bool MOOSNode::toMOOS(std::string moosName, double data){
     m_Comms.Notify(moosName,data,MOOSTime());
     return true;
}

bool MOOSNode::toMOOS(std::string moosName, std::string myString){
     m_Comms.Notify(moosName,myString,MOOSTime());
     return true;
}

//Support for binary-string from ROS to MOOS
bool MOOSNode::toMOOSBinaryString(std::string moosName, std::string myString){
    m_Comms.Notify(moosName,(void *)myString.c_str(),myString.size(),MOOSTime());
	return true;
}

bool MOOSNode::OnNewMail (MOOSMSG_LIST &NewMail){
     MOOSMSG_LIST::iterator p;
     for( p = NewMail.begin() ; p != NewMail.end() ; p++ ){
    
	  CMOOSMsg & rMsg = *p;

	  //Every time a new type is added, a new IF statement is
	  //required here to handle the conversion

	  vector<MsgContainer>::iterator it;

	  for ( it = msgVec->begin() ; it < msgVec->end() ; it++ ){
	       if(MOOSStrCmp(rMsg.GetKey(), it->moosName)){
		    if(rMsg.IsDouble() && (strcmp(it->moosType.c_str(),"Double") == 0) ){
			 if( it->rosType == "std_msgs/Float64"){
			      std_msgs::Float64 dataFloat64;
			      dataFloat64.data = rMsg.GetDouble();
			      it->pub.publish(dataFloat64);
			 }else if( it->rosType == "std_msgs/Float32"){
			      std_msgs::Float32 dataFloat32;
			      dataFloat32.data = rMsg.GetDouble();
			      it->pub.publish(dataFloat32);
			 }else if( it->rosType == "std_msgs/Int32"){
			      std_msgs::Int32 dataInt32;
			      dataInt32.data = rMsg.GetDouble();
			      it->pub.publish(dataInt32);
			 }else if( it->rosType == "std_msgs/Int64"){
			      std_msgs::Int64 dataInt64;
			      dataInt64.data = rMsg.GetDouble();
			      it->pub.publish(dataInt64);
			 }
		    }

		    if(rMsg.IsString() && (it->moosType  == "String")){
			 std_msgs::String myString;
			 myString.data = rMsg.GetString();
			 it->pub.publish(myString);
		    }

	       }//end if MOOSStrCmp
	  }//end Iterator for
     }//end Mailbox for
     return true;
}
 
/* 
   called by the base class when the application has made contact with 
   the MOOSDB and a channel has been opened . Place code to specify what 
   notifications you want to receive here . 
*/ 
bool MOOSNode::OnConnectToServer()
{
     DoRegistrations();
     return true;
}

/*
  Called by the base class periodically. This is where you place code 
  which does the work of the application 
*/ 
bool MOOSNode::Iterate(){
     return true;
}

/* 
   called by the base class before the first :: Iterate is called . Place 
   startup code here − especially code which reads configuration data from the 
   mission file 
*/ 
bool MOOSNode::OnStartUp()
{
     appTick = 1;
     commsTick = 1;

     if(!m_MissionReader.GetConfigurationParam("AppTick",appTick)){
	  MOOSTrace("Warning, AppTick not set.\n");
     }
  
     if(!m_MissionReader.GetConfigurationParam("CommsTick",commsTick)){
	  MOOSTrace("Warning, CommsTick not set.\n");
     }

     SetAppFreq(appTick);
     SetCommsFreq(commsTick);
  
     DoRegistrations();

     return true;
}

void MOOSNode::DoRegistrations(){
     vector<MsgContainer>::iterator it;
     for ( it = msgVec->begin() ; it < msgVec->end() ; it++ ){
	  m_Comms.Register(it->moosName,0);

	  char buf[512];
	  sprintf(buf,"Subscribing to %s\n",it->moosName.c_str());
	  MOOSTrace(buf);
     } 
}

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

// Basic ROS Include
#include "ros/ros.h"

// Standard Library Headers
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h> 
#include <vector>
//#include <map>
#include <string>

// MOOS Includes
#include <MOOS/libMOOS/App/MOOSApp.h>
#include "MOOSNode.h"
#include "MsgContainer.h"

// BOOST Includes
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

// RapidXML Parser
#include "rapidxml/rapidxml.hpp"
#include "rapidxml/rapidxml_print.hpp"

// ROS TYPES
// Note: Add includes here to handle new data type conversion
#include "std_msgs/Int32.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#define FAIL 1
#define SUCCESS 0

//Declare namespaces in use
using namespace rapidxml;
using namespace std;

vector<MsgContainer> subVec;

/*
 * MOOS to ROS Interface
 */
MOOSNode MOOSApp;
void MOOSInit(const char * sMissionFile){
     const char * sMOOSName = "MOOS_ROS_BRIDGE";
   
     MOOSApp.Run(sMOOSName, sMissionFile);
}

/*
 * ROS to MOOS Interface
 */

/*
//Might have to use this in the future with a boost::bind call during subscribe time.
//http://ros-users.122217.n3.nabble.com/How-to-identify-the-subscriber-or-the-topic-name-related-to-a-callback-td2391327.html
//http://mirror.umd.edu/roswiki/ROS%282f%29TCPROS.html
//http://answers.ros.org/question/9590/how-to-bind-a-callback-to-include-a-messageevent/
//
//Instead, using ros::TransportHints().unreliable()); to force other nodes to transmit "topic" as well
//
template <class T, class C>
void callBack(const ros::MessageEvent<T const>& event, const std::string &topic){
ros::M_string header = event.getConnectionHeader();

ros::M_string::iterator it2;
     
// show content:
int count = 0;
for ( it2=header.begin() ; it2 != header.end(); it2++ ) {
cout << count << ": " << (*it2).first << " => " << (*it2).second << endl;
count++;
}

vector<MsgContainer>::iterator it;
for ( it = subVec.begin() ; it < subVec.end() ; it++ ){     
size_t found = topic.find(it->rosName);
if(found!=string::npos){
const C & msg = event.getMessage();
MOOSApp.toMOOS(it->moosName,msg->data);
}
}
}
*/

template <class T, class C>
void callBack(const ros::MessageEvent<T const>& event){
     ros::M_string header = event.getConnectionHeader();
     std::string topic  = header["topic"];

     // show header content:
     // Uncomment this code and check that the "topic" is actually
     // being transmitted. the .unreliable() code forces the "topic"
     // to be transmitted in ROS Fuerte
     /*
       ros::M_string::iterator it2;
       int count = 0;
       for ( it2=header.begin() ; it2 != header.end(); it2++ ) {
       cout << count << ": " << (*it2).first << " => " << (*it2).second << endl;
       count++;
       }
     */

     vector<MsgContainer>::iterator it;
     for ( it = subVec.begin() ; it < subVec.end() ; it++ ){     
	  size_t found = topic.find(it->rosName);
	  if(found!=string::npos){
	       const C & msg = event.getMessage();
	       MOOSApp.toMOOS(it->moosName,msg->data);
	  }
     }
}


template <class T, class C>
void binaryStringCallBack(const ros::MessageEvent<T const>& event){
     ros::M_string header = event.getConnectionHeader();
     std::string topic  = header["topic"];
     vector<MsgContainer>::iterator it;
     for ( it = subVec.begin() ; it < subVec.end() ; it++ ){
          size_t found = topic.find(it->rosName);
          if(found!=string::npos){
               const C & msg = event.getMessage();
               MOOSApp.toMOOSBinaryString(it->moosName,msg->data);
          }
     }
}


/*
 * Parse the XML file to search for Publishers and Subscribers
 * Create ROS Publishers and Subscribers
 * Also create vectors to hold pointers to publishers and subscribers
 */
int CreateROSPubSub(xml_node<> *node, vector<MsgContainer> *pubVec, vector<MsgContainer> *subVec, ros::NodeHandle *n){

     // MOOS Publishers to ROS Subscribers
     if(strcmp(node->first_node("direction")->value(),"toROS") == SUCCESS){
	  if(strcmp(node->first_node("rostype")->value(),"std_msgs/Int32") == SUCCESS){
	       pubVec->push_back(MsgContainer(n->advertise<std_msgs::Int32>(node->first_node("rosname")->value(),1000),
					      node->first_node("moosname")->value(), 
					      node->first_node("rosname")->value(), 
					      node->first_node("moostype")->value(), 
					      node->first_node("rostype")->value()));
	  }else if(strcmp(node->first_node("rostype")->value(),"std_msgs/Int64") == SUCCESS){
	       pubVec->push_back(MsgContainer(n->advertise<std_msgs::Int64>(node->first_node("rosname")->value(),1000),
					      node->first_node("moosname")->value(), 
					      node->first_node("rosname")->value(), 
					      node->first_node("moostype")->value(), 
					      node->first_node("rostype")->value()));
	  }else if(strcmp(node->first_node("rostype")->value(),"std_msgs/String") == SUCCESS){
	       pubVec->push_back(MsgContainer(n->advertise<std_msgs::String>(node->first_node("rosname")->value(),1000),
					      node->first_node("moosname")->value(), 
					      node->first_node("rosname")->value(), 
					      node->first_node("moostype")->value(), 
					      node->first_node("rostype")->value()));
	  }else if(strcmp(node->first_node("rostype")->value(),"std_msgs/Float32") == SUCCESS){
	       pubVec->push_back(MsgContainer(n->advertise<std_msgs::Float32>(node->first_node("rosname")->value(),1000),
					      node->first_node("moosname")->value(), 
					      node->first_node("rosname")->value(), 
					      node->first_node("moostype")->value(), 
					      node->first_node("rostype")->value()));
	  }else if(strcmp(node->first_node("rostype")->value(),"std_msgs/Float64") == SUCCESS){
	       pubVec->push_back(MsgContainer(n->advertise<std_msgs::Float64>(node->first_node("rosname")->value(),1000),
					      node->first_node("moosname")->value(), 
					      node->first_node("rosname")->value(), 
					      node->first_node("moostype")->value(), 
					      node->first_node("rostype")->value()));
	  }else{
	       ROS_INFO("ERROR PARSING XML CONFIG FILE\n");
	       return FAIL;
	  }

	  // ROS Publishers to MOOS Subscribers
	  // In the current release of ROS, the "topic" name is not required to be transmitted with the data, this makes it difficult
	  // for us to decode which message is from where in the callback.  So we force the "topic" name to be transmitted 
	  // by setting the comm links to be unreliable. However, this could change in future versions of ROS, need to 
	  // keep an eye on this functionality.
     }else if(strcmp(node->first_node("direction")->value(),"toMOOS") == SUCCESS){
	  if(strcmp(node->first_node("rostype")->value(),"std_msgs/Int32") == 0){
               subVec->push_back(MsgContainer(n->subscribe(node->first_node("rosname")->value(),
							   1000,callBack<std_msgs::Int32,std_msgs::Int32ConstPtr>, ros::TransportHints().unreliable()),
					      node->first_node("moosname")->value(),
					      node->first_node("rosname")->value()));
	  }else if(strcmp(node->first_node("rostype")->value(),"std_msgs/Int64") == SUCCESS){
	       subVec->push_back(MsgContainer(n->subscribe(node->first_node("rosname")->value(),
							   1000,callBack<std_msgs::Int64,std_msgs::Int64ConstPtr>, ros::TransportHints().unreliable()),
					      node->first_node("moosname")->value(),
					      node->first_node("rosname")->value()));
	  }else if(strcmp(node->first_node("rostype")->value(),"std_msgs/Float32") == SUCCESS){
	       subVec->push_back(MsgContainer(n->subscribe(node->first_node("rosname")->value(),
							   1000,callBack<std_msgs::Float32,std_msgs::Float32ConstPtr>, ros::TransportHints().unreliable()),
					      node->first_node("moosname")->value(),
					      node->first_node("rosname")->value()));
	  }else if(strcmp(node->first_node("rostype")->value(),"std_msgs/Float64") == SUCCESS){
	       subVec->push_back(MsgContainer(n->subscribe(node->first_node("rosname")->value(),
							   1000,callBack<std_msgs::Float64,std_msgs::Float64ConstPtr>, ros::TransportHints().unreliable()),
					      node->first_node("moosname")->value(),
					      node->first_node("rosname")->value()));
	  }else if(strcmp(node->first_node("rostype")->value(),"std_msgs/String") == SUCCESS){
	       subVec->push_back(MsgContainer(n->subscribe(node->first_node("rosname")->value(),
							   1000,callBack<std_msgs::String,std_msgs::StringConstPtr>, ros::TransportHints().unreliable()),
					      node->first_node("moosname")->value(),
					      node->first_node("rosname")->value()));
	  }else if(strcmp(node->first_node("rostype")->value(),"std_msgs/String/Binary") == SUCCESS){
               // Binary-string support
               subVec->push_back(MsgContainer(n->subscribe(node->first_node("rosname")->value(),
                                                           1000,binaryStringCallBack<std_msgs::String,std_msgs::StringConstPtr>, ros::TransportHints().unreliable()),
                                              node->first_node("moosname")->value(),
                                              node->first_node("rosname")->value()));
          }else{
	       ROS_INFO("ERROR PARSING XML CONFIG FILE\n");
	       return FAIL;
	  }
     }

     return SUCCESS;
}

int main(int argc, char **argv)
{
     if(argc < 3){
          ROS_INFO("Invalid number of parameters\n\n");
          ROS_INFO("argc is %d, but it should be at least 3.\n", argc);
          ROS_INFO("Usage:\n\trosrun moosros Bridge <moosrosconfig.xml> <mission.moos>");
          return 0;
     }

     //Initialize ROS Communications
     ros::init(argc, argv, "MOOS_ROS_Bridge");
     ros::NodeHandle n;

     //Read in complete XML Document
     string str,strTotal;
     ifstream in;

     //Check for existence of config files
     struct stat stFileInfo;
     int intStat;
     intStat = stat(argv[1],&stFileInfo);
     if(intStat != 0) {
          ROS_INFO("\n******\nCONFIG FILE MISSING\n%s does not exist!\n******\n",argv[1]);
          return 0;
     }

     //Open file and read the entire file into the strTotal buffer
     in.open(argv[1]);
     getline(in,str);

     while ( in ) {
          strTotal += str;
          getline(in,str);
     }

     //Convert C++ string to char*
     char * xmlDoc = new char [strTotal.size()+1];
     strcpy (xmlDoc, strTotal.c_str());

     //Parse XML Document
     xml_document<> doc;    // character type defaults to char
     doc.parse<0>(xmlDoc);    // 0 means default parse flags

     vector<MsgContainer> pubVec;

     //Get First Topic/Message
     xml_node<> *node = doc.first_node()->first_node();

     if(CreateROSPubSub(node,&pubVec,&subVec,&n) == FAIL)
          return 0;

     //Process all Topics/Messages
     while( (node = node->next_sibling()) != 0 )
          if(CreateROSPubSub(node,&pubVec,&subVec,&n) == FAIL)
               return 0;

     ros::Rate loop_rate(10);

     //Kick off the MOOS Loop in a separate thread
     //before entering the ROS Loop
     MOOSApp.AssignPublisher(&pubVec);
     boost::thread MOOSThread(MOOSInit, argv[2]);

     int count = 0;
     while (ros::ok())
     {
          ros::spinOnce();
          loop_rate.sleep();
          ++count;
     }

     return 0;
}

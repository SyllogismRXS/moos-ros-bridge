#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include <sstream>

int main(int argc, char **argv)
{
     ros::init(argc, argv, "counter");
     ros::NodeHandle n;
     ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("CounterFromROS", 1000);

     ros::Rate loop_rate(1);

     int count = 0;
     std_msgs::Int32 msg;
     while (ros::ok()) {

	  msg.data = count;

	  chatter_pub.publish(msg);

	  ros::spinOnce();

	  loop_rate.sleep();
	  ++count;
     }


     return 0;
}

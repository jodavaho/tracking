#include <string>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <network/protocol.h>
#include <time.h>

using namespace std;
using namespace ros;
int main(int argc, char** argv){
	init(argc,argv,"faker");
	NodeHandle nh("/");

	Publisher pub = nh.advertise<std_msgs::String>("/network/received",1);

	while(ok()){
		stringstream ss;
		time_t tnow = time(NULL);
		ss<<"dummy[T="<<tnow<<",Dt=30,Xn=-40,Yn=60,Zn=2.0,Zc=1,Rn=.29,x=1,y=1,th=1.7]";
		std_msgs::String omsg;
		omsg.data = ss.str();
		Duration(1.0).sleep();
		pub.publish(omsg);
		spinOnce();
	}

	return EXIT_SUCCESS;
}

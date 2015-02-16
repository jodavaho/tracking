#include <string>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <heartbeat/delimiters.h>
#include <time.h>

using namespace std;
using namespace ros;

int main(int argc, char** argv){
	init(argc,argv,"faker");
	NodeHandle nh("/");

	Publisher pub = nh.advertise<std_msgs::String>("/network/received",1);

	double sensor_sigma = M_PI/12;

	double t1=-30;
	double t2=-120;
	double rx = -52;
	double ry = -60;
	double z = atan2(ry-t2,rx-t1);
	while(ok()){
		stringstream ss;
		time_t tnow = time(NULL);
		ss<<"dummy[T="<<tnow<<",Dt="<<tnow<<",Xn="<<rx<<",Yn="<<ry<<",Zn="<<z<<",Rn="<<sensor_sigma<<",Zc=3,x=1,y=1,th=1.7]";
		std_msgs::String omsg;
		omsg.data = ss.str();
		Duration(1.0).sleep();
		pub.publish(omsg);
		spinOnce();
	}

	return EXIT_SUCCESS;
}

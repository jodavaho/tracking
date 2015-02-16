#include <tagloc/tracking.h>
#include <bearing_measurement/GetMeasurement.h>
#include <tagloc/algorithms.h>
#include <iostream>
#include <Eigen/Dense>
#include <lavant/WaypointNavigation.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>
#include <helpers/geometry_msgs.h>
#include <map>
#include <unistd.h>
#include <fstream>

using namespace std;
bool haveboth;
bool update;
map<string,string> data;

ofstream logfile;

ros::ServiceClient wpclient;
ros::ServiceClient bearingclient;
ros::Subscriber stateIn;
ros::Subscriber hbin;
ros::Publisher hbout;
lavant::WaypointNavigation::Response wpres;
lavant::WaypointNavigation::Request wpreq;

Eigen::MatrixXd r1(2,1);
Eigen::MatrixXd r2(2,1);
Eigen::MatrixXd t(2,1);
Eigen::MatrixXd P(2,2);
Eigen::MatrixXd rend(2,2);

vector<double> px;
vector<double> py;
vector<double> R;
vector<double> Z;

double sensor_sigma=(M_PI/8)*(M_PI/8);

void mapstate(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
		std::stringstream ss;
		ss<<msg->pose.pose.position.x;
		data["_x"]=ss.str();
		ss.str("");
		ss<<msg->pose.pose.position.y;
		data["_y"]=ss.str();
		ss.str("");
		ss<<RSN::getYaw(msg->pose.pose.orientation);
		data["_th"]=ss.str();
		ss.str("");
}

string pdelim=",";
string delim="=";

void nwin(const std_msgs::String::ConstPtr &msg){
	ROS_INFO("NWIN");
	if (!update) return;

	std::string s = msg->data;	
	if (s.find("=")==string::npos){
		ROS_INFO("No values in %s",s.c_str());
		return;
	}
	std::string key,val;
	int dpos,ppos;
	int msgstart = s.find("[")+1;
	int msgend = s.find("]");
	s = s.substr(msgstart,msgend-msgstart);
	printf("Heard: %s\n",msg->data.c_str());
	while( (dpos=s.find(delim))!=std::string::npos ){
		key=s.substr(0,dpos);
		ppos = s.find(pdelim);
		if (((size_t)ppos)==std::string::npos){
			val = s.substr(dpos+1);
			s="";
		}
		else{
			val = s.substr(dpos+1,ppos-dpos-1);
			s = s.substr(ppos+1);
		}
		std::stringstream ss;
		ss<<key;
		ss>>key;
		ss<<val;
		ss>>val;
		ROS_INFO("Found: %s = %s\n",key.c_str(),val.c_str());
		data[key]=val;
	}
	if (data["x"].length()>0 &&
			data["y"].length()>0 &&
			data["th"].length()>0 && 
			data["_x"].length()>0 &&
			data["_y"].length()>0 &&
			data["_th"].length()>0){
		haveboth=true;
	}
}
void setupRos(ros::NodeHandle nh){
	wpclient=nh.serviceClient<lavant::WaypointNavigation>("/control/command");
	bearingclient=nh.serviceClient<bearing_measurement::GetMeasurement>("bearing_measurement/GetMeasurement");
	stateIn = nh.subscribe("/robot/state",1,mapstate);
	hbin = nh.subscribe("/network/received",1,nwin);
	hbout = nh.advertise<std_msgs::String>("/network/out",1);
}
int main(int argc, char** argv){


	if (argc!=7){
		return EXIT_FAILURE;
	}

	t = Eigen::MatrixXd(2,1);
	P = Eigen::MatrixXd(2,2);

	t<<atof(argv[1]),atof(argv[2]);
	P<<atof(argv[3]),atof(argv[4]),atof(argv[5]),atof(argv[6]);

	cout<<t<<endl;
	cout<<P<<endl;

	string fname="";
	ros::init(argc,argv,"tl");
	ros::NodeHandle nh("/");

	setupRos(nh);

	char HOSTNAME[HOST_NAME_MAX];
	gethostname(HOSTNAME,HOST_NAME_MAX);
	ROS_INFO("Onestep. I am %s",HOSTNAME);

	ROS_INFO("Stopping waypoint navigation, if any");
	wpreq.desiredState=2;
	wpreq.blockState=wpreq.NONBLOCKING;
	if (wpclient.exists()){
		wpclient.call(wpreq,wpres);
	} else {
		cerr<<"WP Service doesn't exist"<<endl;
		ROS_WARN("Skipping waypoint service");
	}

	ROS_WARN("DEFAULT VALUES");

	ROS_INFO("Waiting for joint state");
	haveboth=false;
	update = true;

	data.clear();
	while(haveboth==false && ros::ok()){
		ros::spinOnce();
		ros::Duration(1).sleep();
	}
	if (!ros::ok()){
		cerr<<"ROS down. Exiting"<<endl;
		return EXIT_FAILURE;
	}
	update = false;

	ROS_INFO("Got joint state, calculating step");

	r1<<atof(data["_x"].c_str()),atof(data["_y"].c_str());
	r2<<atof(data["x"].c_str()),atof(data["y"].c_str());

	cout<<"________________"<<endl;
	cout<<"Robot positions:"<<endl;
	cout<<"Me:"<<r1.transpose()<<endl;
	cout<<"Other guy:"<<r2.transpose()<<endl;
	cout<<"________________"<<endl;

	RSN::onestep(r1,r2,t,P,.1,3);
	rend = (r1+r2)/2;

	ROS_INFO("Got destination: %0.3f,%0.3f",r1(0),r1(1));
	cout<<r1<<endl;	

	wpreq.destination.x=r1(0);
	wpreq.destination.y=r1(1);
	wpreq.destination.theta=0;

	if (wpclient.exists()){
		wpclient.call(wpreq,wpres);
	} else {
		cerr<<"wpservice problem"<<endl;
		ROS_WARN("Skipping waypoint service");
	}
	
	ROS_INFO("Taking Bearing Measurement");
	bearing_measurement::GetMeasurement::Request breq;
	bearing_measurement::GetMeasurement::Response bres;

	if (bearingclient.exists()){
		bearingclient.call(breq,bres);
	} else {
		ROS_WARN("Assuming fake bearing emasurement and spelling");
		bres.valid.data=true;
		bres.bearing=0;
	}

	stringstream ss;
	r1<<atof(data["_x"].c_str()),atof(data["_y"].c_str());
	px.push_back(r1(0));
	py.push_back(r1(1));
	Z.push_back(bres.bearing);
	R.push_back(sensor_sigma);

	ROS_INFO("Publishing Measurement value over network");
	ss<<"Zn="<<bres.bearing<<",";
	ss<<"Xn="<<r1(0)<<",";
	ss<<"Yn="<<r1(1)<<",";
	ss<<"Rn="<<sensor_sigma<<",";
	ss<<"Zc="<<Z.size();

	std_msgs::String omsg;
	omsg.data = ss.str();

	hbout.publish(omsg);

	ROS_INFO("Going to rendesvous point");

	if (wpclient.exists()){
		wpreq.destination.x = rend(0);
		wpreq.destination.y = rend(1);
		wpreq.destination.theta = 0;
		wpreq.desiredState = 0;
		wpreq.blockState=wpreq.BLOCKING;
		wpclient.call(wpreq,wpres);
	}

	ROS_INFO("Arrived. Waiting for joint state");
	haveboth=false;
	update = true;
	data.clear();

	while(haveboth==false && ros::ok()){
		ros::spinOnce();
		ros::Duration(1).sleep();
	}
	if (!ros::ok()){
		cerr<<"ROS down. Exiting"<<endl;
		return EXIT_FAILURE;
	}
	update = false;

	int theirs = atoi(data["Zc"].c_str());
	if ((size_t)theirs != Z.size()){
		ROS_INFO("Mis-match in measurement count: %Zu (mine) %d (theirs)",Z.size(),theirs);
	}

	px.push_back(atof(data["Xn"].c_str()));
	py.push_back(atof(data["Yn"].c_str()));
	Z.push_back(atof(data["Zn"].c_str()));
	R.push_back(atof(data["Rn"].c_str()));

	ROS_INFO("Got Joint State. Updating hypothesis");

	double t[2];
	double c[4];
	RSN::line_intersection(Z.size(),px,py,Z,t[0],t[1]);
	RSN::BOT::IWLS_2D(Z.size(),px,py,Z,R,t,c,true);
	cout<<t[0]<<","<<t[1]<<endl;
	cout<<c[0]<<" "<<c[1]<<endl;
	cout<<c[2]<<" "<<c[3]<<endl;

	return EXIT_SUCCESS;
}

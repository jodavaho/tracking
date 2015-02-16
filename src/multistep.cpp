#include <tagloc/tracking.h>
#include <bearing_measurement/GetMeasurement.h>
#include <tagloc/algorithms.h>
#include <iostream>
#include <Eigen/Dense>
#include <heartbeat/delimiters.h>
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
bool theyreok;
bool ignore_failures;
map<string,string> data;

ros::ServiceClient wpclient;
ros::ServiceClient bearingclient;
ros::Subscriber stateIn;
ros::Subscriber hbin;
ros::Publisher hbout;

vector<double> px;
vector<double> py;
vector<double> R;
vector<double> Z;

int FREQ;
int their_count;
double sensor_var=.29;

void publish_z(double x, double y, double bearing, double sensor_variance){
	std::stringstream ss;
	std_msgs::String omsg;
	ss<<"Xn="<<x<<",";
	ss<<"Yn="<<y<<",";
	ss<<"Zn="<<bearing<<",";
	ss<<"Rn="<<sensor_variance<<",";
	ss<<"Zc="<<Z.size();
	ROS_INFO("Publishing the %d meas",Z.size());
	omsg.data = ss.str();
	hbout.publish(omsg);
}
bool measure(int f,double &bearing){
	ROS_INFO("Taking Bearing Measurement");
	bearing_measurement::GetMeasurement::Request breq;
	bearing_measurement::GetMeasurement::Response bres;
	breq.freq = f;
	if (bearingclient.exists()){
		bearingclient.call(breq,bres);
		
	} else {
		ROS_WARN("Unable to measure.");
	}
	bearing = bres.bearing;
	ROS_INFO("Got: %0.3f", bres.bearing);
	return bres.valid.data;
}
void clear(){
	std_msgs::String omsg;
	data.clear();
	std::stringstream ss;
	ss.clear();
	ROS_WARN("DEFAULT VALUES");
	ROS_INFO("Clearing old measurements.");
	ss<<"Zc="<<0;
	omsg.data = ss.str();
	hbout.publish(omsg);
}
void sync_robots(int timeout_sec=-1/*wait forever*/);
void move(double x, double y);
//int onestep(Eigen::MatrixXd &t,Eigen::MatrixXd &P);

//Maintenance functions:
void setupRos(ros::NodeHandle nh);
void nwin(const std_msgs::String::ConstPtr &msg);
void mapstate(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

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

void nwin(const std_msgs::String::ConstPtr &msg){
	//if (!update) return;

	char *state,*inner_state,*token;

	ROS_INFO("NWIN: %s",msg->data.c_str());
	std::string s = msg->data;
	int msgstart = s.find("[")+1;
	int msgend = s.find("]");
	char *cs_s = (char*)calloc(s.size()+1,sizeof(char));
	strcpy(cs_s,s.c_str());

	std::string sdata = s.substr(msgstart,msgend-msgstart);
	char *cs_data = (char*)calloc(sdata.size()+1,sizeof(char));
	strcpy(cs_data,sdata.c_str());

	//first get all meta-data from the string
	token = strtok_r(cs_s,p_delim.c_str(),&state);
	while(token!=NULL){
		std::string key;
		std::string val;
		std::stringstream ss;
		ss<<strtok_r(token,kv_delim.c_str(),&inner_state);
		ss>>key;ss.clear();
		ss<<strtok_r(NULL,kv_delim.c_str(),&inner_state);
		ss>>val;ss.clear();
		data[key]=val;
		token = strtok_r(NULL,p_delim.c_str(),&state);
	}
	//now get all msg data from the string
	token = strtok_r(cs_data,p_delim.c_str(),&state);
	while(token!=NULL){
		std::string key;
		std::string val;
		std::stringstream ss;
		ss<<strtok_r(token,kv_delim.c_str(),&inner_state);
		ss>>key;ss.clear();
		ss<<strtok_r(NULL,kv_delim.c_str(),&inner_state);
		ss>>val;ss.clear();
		data[key]=val;
		token = strtok_r(NULL,p_delim.c_str(),&state);
	}
//	map<string,string>::iterator kvi = data.begin();
//	for(;kvi!=data.end();kvi++){
//		cout<<"data["<<kvi->first<<"]="<<kvi->second<<endl;
//	}
	//theyreok = data["OK"].size()>0 && strcmp(data["OK"].c_str(),"true")==0;
	if (data["x"].length()>0 &&
			data["y"].length()>0 &&
			data["th"].length()>0 && 
			data["_x"].length()>0 &&
			data["_y"].length()>0 &&
			data["_th"].length()>0){
		haveboth=true;
	}
	if(data["Zc"].size()>0){
		their_count=atoi(data["Zc"].c_str());
	}

	free(cs_s);
	free(cs_data);
}

void stop(){
	lavant::WaypointNavigation::Response wpres;
	lavant::WaypointNavigation::Request wpreq;
	ROS_INFO("Stopping navigation, if any");
	wpreq.desiredState=2;
	wpreq.blockState=wpreq.BLOCKING;
	if (wpclient.exists()){
		wpclient.call(wpreq,wpres);
	} else {
		cerr<<"WP Service doesn't exist"<<endl;
		ROS_WARN("Skipping waypoint service");
	}
}
void move(double x, double y){
	lavant::WaypointNavigation::Response wpres;
	lavant::WaypointNavigation::Request wpreq;
	ROS_INFO("moving to:%0.3f %0.3f",x,y);
	wpreq.desiredState=0;
	wpreq.destination.x=x;
	wpreq.destination.y=y;
	wpreq.blockState=wpreq.BLOCKING;

	if (wpclient.exists()){
		wpclient.call(wpreq,wpres);
	} else {
		cerr<<"WP Service doesn't exist"<<endl;
		ROS_WARN("Skipping waypoint service");
	}
	ROS_INFO("Move Complete");
}
void sync_robots(int s){
	std_msgs::String omsg;
	data.clear();
	ROS_INFO("Waiting for joint state");
	haveboth=false;
	update = true;
	theyreok=false;

	//let them know we need their status
	omsg.data="OK=false";
	hbout.publish(omsg);

	ROS_WARN("Add duration wait");
	ROS_INFO("Waiting for both robot states");
	//ros::Time::now()+
	//wait until we got their status
	ros::Duration(5).sleep();
	while(!haveboth && ros::ok()){
		//while we wait, the local state is updated, 
		//published over network, and the other robot is assumed to do the same
		ros::spinOnce();
		ros::Duration(1).sleep();
	}
	ros::Duration(5).sleep();
	//assume we have both our state and other guy's state, 
	//as well as any extra data he sent along

	if (!ros::ok()){
		cerr<<"ROS down. Exiting"<<endl;
		return;
	}
	omsg.data="OK=true";
	hbout.publish(omsg);
	ros::Duration(20).sleep();
	
	ROS_INFO("Got joint state.");
}
void setupRos(ros::NodeHandle nh){
	wpclient=nh.serviceClient<lavant::WaypointNavigation>("/control/command");
	bearingclient=nh.serviceClient<bearing_measurement::GetMeasurement>("bearing_measurement/get_measurement");
	stateIn = nh.subscribe("/robot/state",1,mapstate);
	hbin = nh.subscribe("/network/received",1,nwin);
	hbout = nh.advertise<std_msgs::String>("/network/out",1);
}
int onestep(Eigen::MatrixXd &t,Eigen::MatrixXd &P){
	Eigen::MatrixXd r1(2,1);
	Eigen::MatrixXd r2(2,1);
	Eigen::MatrixXd rend(2,2);
	std::stringstream ss;

	cout<<t<<endl;
	cout<<P<<endl;

	clear();
	sync_robots();
	if (!ros::ok()){
		return EXIT_FAILURE;
	}

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

	move(r1(0),r1(1));

	double bearing;
	bool zok = measure(FREQ,bearing);

	if (zok || ignore_failures){
		r1<<atof(data["_x"].c_str()),atof(data["_y"].c_str());
		px.push_back(r1(0));
		py.push_back(r1(1));
		Z.push_back(bearing);
		R.push_back(sensor_var);
		ROS_INFO("Publishing Measurement value over network");
		publish_z(r1(0),r1(1),bearing,sensor_var);
	}
	ROS_INFO("Going to rendesvous point");
	
	move(rend(0),rend(1));
//	clear();
	sync_robots();
	if (!ros::ok()){
		return EXIT_FAILURE;
	}

	ROS_INFO("Examining their data");

	int theirs = atoi(data["Zc"].c_str());
	if ((size_t)theirs == 0){
		ROS_INFO("They have no measurements");
	} else if ((size_t)theirs >= Z.size()){
		ROS_INFO("%Zu (mine) %d (theirs)",Z.size(),theirs);
		ROS_WARN("Not requesting other measurements, just using most current");
		px.push_back(atof(data["Xn"].c_str()));
		py.push_back(atof(data["Yn"].c_str()));
		Z.push_back(atof(data["Zn"].c_str()));
		R.push_back(atof(data["Rn"].c_str()));
	} else if ((size_t) theirs < Z.size()){
		ROS_INFO("They missed a measurement!");
	}

	ROS_INFO("Got Joint State. Updating hypothesis");
	ROS_INFO("Measurement Sequence is:");
	for (unsigned int i=0;i<Z.size();i++){
		cout<<px[i]<<","<<py[i]<<":"<<Z[i]<<endl;
	}

	double tz[2];
	double c[4];
	RSN::lineIntersection(Z.size(),px,py,Z,tz[0],tz[1]);
	RSN::BOT::IWLS_2D(Z.size(),px,py,Z,R,tz,c,true);
	t<<tz[0],tz[1];
	P<<c[0],c[1],c[2],c[3];

	cout<<t<<endl;
	cout<<P<<endl;

	return EXIT_SUCCESS;
}

int main(int argc, char** argv){

	Eigen::MatrixXd r1(2,1);
	Eigen::MatrixXd r2(2,1);
	FREQ = 49711;

	string fname="";
	ros::init(argc,argv,"tl");
	ros::NodeHandle nh("/");

	setupRos(nh);

	char HOSTNAME[HOST_NAME_MAX];
	gethostname(HOSTNAME,HOST_NAME_MAX);
	ROS_INFO("Multi-step. I am %s",HOSTNAME);

	clear();
	Eigen::MatrixXd t(2,1);
	Eigen::MatrixXd P(2,2);
	t<<-30,-120;
	P<<900,0,0,900;

	r1<<-10,-60;
	r2<<-40,-60;
	Eigen::MatrixXd rend(2,1);
	rend<<-25,-60;
	ROS_INFO("Starting, going to -30,0");
	move(-30.0,0);

	std::stringstream ss;

	cout<<t<<endl;
	cout<<P<<endl;


	ROS_INFO("Got destination: %0.3f,%0.3f",r1(0),r1(1));
	cout<<r1<<endl;	

	move(r1(0),r1(1));

	double bearing;
	bool zok = measure(FREQ,bearing);

	if (zok || ignore_failures){
		r1<<atof(data["_x"].c_str()),atof(data["_y"].c_str());
		px.push_back(r1(0));
		py.push_back(r1(1));
		Z.push_back(bearing);
		R.push_back(sensor_var);
		ROS_INFO("Publishing Measurement value over network");
		publish_z(r1(0),r1(1),bearing,sensor_var);
	}
	ROS_INFO("Going to rendesvous point");
	
	move(rend(0),rend(1));
	sync_robots();
	if (!ros::ok()){
		return EXIT_FAILURE;
	}

	ROS_INFO("Examining their data");

	int theirs = atoi(data["Zc"].c_str());
	if ((size_t)theirs == 0){
		ROS_INFO("They have no measurements");
	} else if ((size_t)theirs >= Z.size()){
		ROS_INFO("%Zu (mine) %d (theirs)",Z.size(),theirs);
		ROS_WARN("Not requesting other measurements, just using most current");
		px.push_back(atof(data["Xn"].c_str()));
		py.push_back(atof(data["Yn"].c_str()));
		Z.push_back(atof(data["Zn"].c_str()));
		R.push_back(atof(data["Rn"].c_str()));
	} else if ((size_t) theirs < Z.size()){
		ROS_INFO("They missed a measurement!");
	}

	ROS_INFO("Got Joint State. Updating hypothesis");
	ROS_INFO("Measurement Sequence is:");
	for (unsigned int i=0;i<Z.size();i++){
		cout<<px[i]<<","<<py[i]<<":"<<Z[i]<<endl;
	}

	double tz[2];
	double c[4];
	RSN::lineIntersection(Z.size(),px,py,Z,tz[0],tz[1]);
	RSN::BOT::IWLS_2D(Z.size(),px,py,Z,R,tz,c,true);
	Eigen::MatrixXd tt = t;
	t<<tz[0],tz[1];
	P<<c[0],c[1],c[2],c[3];

	t = (t + tt)/2.0;

	r1 = rend;
	r2 = rend;
	RSN::onestep(r1,r2,t,P,.1,3);
	rend = (r1+r2)/2;

	ROS_INFO("Got destination: %0.3f,%0.3f",r1(0),r1(1));
	cout<<r1<<endl;	

	move(r1(0),r1(1));

	zok = measure(FREQ,bearing);

	if (zok || ignore_failures){
		r1<<atof(data["_x"].c_str()),atof(data["_y"].c_str());
		px.push_back(r1(0));
		py.push_back(r1(1));
		Z.push_back(bearing);
		R.push_back(sensor_var);
		ROS_INFO("Publishing Measurement value over network");
		publish_z(r1(0),r1(1),bearing,sensor_var);
	}
	ROS_INFO("Going to rendesvous point");
	
	move(rend(0),rend(1));
	sync_robots();
	if (!ros::ok()){
		return EXIT_FAILURE;
	}

	ROS_INFO("Examining their data");

	theirs = atoi(data["Zc"].c_str());
	if ((size_t)theirs == 0){
		ROS_INFO("They have no measurements");
	} else if ((size_t)theirs >= Z.size()){
		ROS_INFO("%Zu (mine) %d (theirs)",Z.size(),theirs);
		ROS_WARN("Not requesting other measurements, just using most current");
		px.push_back(atof(data["Xn"].c_str()));
		py.push_back(atof(data["Yn"].c_str()));
		Z.push_back(atof(data["Zn"].c_str()));
		R.push_back(atof(data["Rn"].c_str()));
	} else if ((size_t) theirs < Z.size()){
		ROS_INFO("They missed a measurement!");
	}

	ROS_INFO("Got Joint State. Updating hypothesis");
	ROS_INFO("Measurement Sequence is:");
	for (unsigned int i=0;i<Z.size();i++){
		cout<<px[i]<<","<<py[i]<<":"<<Z[i]<<endl;
	}

	RSN::lineIntersection(Z.size(),px,py,Z,tz[0],tz[1]);
	RSN::BOT::IWLS_2D(Z.size(),px,py,Z,R,tz,c,true);
	t<<tz[0],tz[1];
	P<<c[0],c[1],c[2],c[3];

	cout<<t<<endl;
	cout<<P<<endl;
	move(10,-120);
	return 0;
}

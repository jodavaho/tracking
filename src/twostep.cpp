#include <tagloc/tracking.h>
#include <bearing_measurement/GetMeasurement.h>
#include <tagloc/algorithms.h>
#include <iostream>
#include <Eigen/Dense>
#include <networking/protocol.h>
#include <control/WaypointNavigation.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>
#include <helpers/geometry_msgs.h>
#include <map>
#include <unistd.h>
#include <fstream>

using std::vector;
using std::map;
using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::stringstream;
using Eigen::MatrixXd;
using ros::ServiceClient;
using ros::Subscriber;
using ros::Publisher;

const string msg_start = SimpleProtocol::msg_start_str;
const string msg_stop = SimpleProtocol::msg_stop_str;
const string kv_delim = SimpleProtocol::key_value_delim_str;
const string p_delim = SimpleProtocol::pair_delim_str;

bool simulate = true;
map<string,string> data;

ServiceClient wpclient;
ServiceClient bearingclient;
Subscriber stateIn;
Subscriber hbin;
Publisher hbout;
Publisher status_out;

vector<double> px;
vector<double> py;
vector<double> R;
vector<double> Z;

vector<double> opx;
vector<double> opy;
vector<double> oR;
vector<double> oZ;

vector<double> apx;
vector<double> apy;
vector<double> aR;
vector<double> aZ;


int DESIRED_ZC=0;
int HAVE_ZC=0;
int FREQ;
int their_count=-1;
long last_time=-1;
long returned_time=999999;
double sensor_var=pow((M_PI/16),2.0);;
double cx,cy;

double get_radius(MatrixXd P){
	assert(P.rows()==P.cols());
	Eigen::JacobiSVD<MatrixXd> _svdA;
	_svdA.compute(P);
	return sqrt(_svdA.singularValues()(0));
}
void sync_robots_at(double x, double y);
void move(double x, double y, bool block=true,bool print = true);
void publish_z(double x, double y, double bearing, double sensor_variance);
void publish_all_z(vector<double> px, vector<double> y, vector<double> bearing, vector<double> sensor_variance);
bool measure(int f,double &bearing);
void setupRos(ros::NodeHandle nh);
void nwin(const std_msgs::String::ConstPtr &msg);
void mapstate(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
void get_all_meausrements(vector<double>&  px,vector<double>&  py,vector<double>&  Z,vector<double>&  R,vector<double>& opx,vector<double>& opy,vector<double>& oZ,vector<double>& oR,vector<double>& apx,vector<double>& apy,vector<double>& aZ,vector<double>& aR);
void get_last_measurement(vector<double>& opx,vector<double>& opy,vector<double>& oZ,vector<double>& oR);

////////////////////////////////////////////////////////////////////////////////
void mapstate(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
	stringstream ss;
	ss<<msg->pose.pose.position.x;
	data["_x"]=ss.str();
	ss.str("");
	ss<<msg->pose.pose.position.y;
	data["_y"]=ss.str();
	ss.str("");
	ss<<RSN::getYaw(msg->pose.pose.orientation);
	data["_th"]=ss.str();
	ss.str("");
	cx = atof(data["_x"].c_str());
	cy = atof(data["_y"].c_str());
}

void publish_z(double x, double y, double bearing, double sensor_variance){
	stringstream ss;
	std_msgs::String omsg;
	ss<<"Xn="<<x<<",";
	ss<<"Yn="<<y<<",";
	ss<<"Zn="<<bearing<<",";
	ss<<"Rn="<<sensor_variance<<",";
	ss<<"Zc="<<Z.size();
	omsg.data = ss.str();
	ROS_INFO("Publishing %Zu = %s",Z.size(),omsg.data.c_str());
	hbout.publish(omsg);
}

void publish_all_z(vector<double> xs, vector<double> ys, vector<double> zs, vector<double> rs){
	stringstream ss;
	std_msgs::String omsg;
	ss<<"Zc="<<Z.size();
	for (unsigned int i=0;i<zs.size();i++){
		ss<<",X"<<i<<"="<<xs[i];
		ss<<",Y"<<i<<"="<<ys[i];
		ss<<",Z"<<i<<"="<<zs[i];
		ss<<",R"<<i<<"="<<rs[i];
	}
	omsg.data = ss.str();
	ROS_INFO("Publishing %s",omsg.data.c_str());
	hbout.publish(omsg);
}

/** Please don't laugh, I'm in a hurry -jdvh **/
void get_all_meausrements(vector<double>&  px,vector<double>&  py,vector<double>&  Z,vector<double>&  R,vector<double>& opx,vector<double>& opy,vector<double>& oZ,vector<double>& oR,vector<double>& apx,vector<double>& apy,vector<double>& aZ,vector<double>& aR){
		int theirs = atoi(data["Zc"].c_str());
		
		opx.clear();
		opy.clear();
		oZ.clear();
		oR.clear();

		apx.clear();
		apy.clear();
		aZ.clear();
		aR.clear();

		for (int i=0;i<theirs;i++){
			stringstream ss;
			ss<<"X"<<i;
			opx.push_back(atof(data[ss.str()].c_str()));
		}
		for (int i=0;i<theirs;i++){
			stringstream ss;
			ss<<"Y"<<i;
			opy.push_back(atof(data[ss.str()].c_str()));
		}
		for (int i=0;i<theirs;i++){
			stringstream ss;
			ss<<"Z"<<i;
			oZ.push_back(atof(data[ss.str()].c_str()));
		}
		for (int i=0;i<theirs;i++){
			stringstream ss;
			ss<<"R"<<i;
			oR.push_back(atof(data[ss.str()].c_str()));
		}
		apx.insert(apx.end(),px.begin(),px.end());
		apx.insert(apx.end(),opx.begin(),opx.end());
		apy.insert(apy.end(),py.begin(),py.end());
		apy.insert(apy.end(),opy.begin(),opy.end());
		aZ.insert(aZ.end(),Z.begin(),Z.end());
		aZ.insert(aZ.end(),oZ.begin(),oZ.end());
		aR.insert(aR.end(),R.begin(),R.end());
		aR.insert(aR.end(),oR.begin(),oR.end());
}
void get_last_measurement(vector<double>& opx,vector<double>& opy,vector<double>& oZ,vector<double>& oR){
	opx.push_back(atof(data["Xn"].c_str()));
	opy.push_back(atof(data["Yn"].c_str()));
	oZ.push_back(atof(data["Zn"].c_str()));
	oR.push_back(atof(data["Rn"].c_str()));
}
bool measure(int f,double &bearing){
	if (simulate){
		ROS_WARN("SIMULATING MEASUREMENT!");
		return true;
	}
	ROS_INFO("Taking Bearing Measurement");
	bearing_measurement::GetMeasurement::Request breq;
	bearing_measurement::GetMeasurement::Response bres;
	breq.freq = f;
	if (bearingclient.exists()){
		bearingclient.call(breq,bres);
	} else {
		ROS_WARN("Unable to measure.");
	}
	bearing = ((bres.bearing)/180.0)*M_PI;
	ROS_INFO("Got: %0.3f", bres.bearing);

	return bres.valid.data;
}
void nwin(const std_msgs::String::ConstPtr &msg){

	char *state,*inner_state,*token;

	//ROS_INFO("NWIN: %s",msg->data.c_str());
	string s = msg->data;
	int msgstart = s.find(msg_start)+1;
	int msgend = s.find(msg_stop);
	char *cs_s = (char*)calloc(s.size()+1,sizeof(char));
	strcpy(cs_s,s.c_str());

	string sdata = s.substr(msgstart,msgend-msgstart);
	char *cs_data = (char*)calloc(sdata.size()+1,sizeof(char));
	strcpy(cs_data,sdata.c_str());

	//first get all meta-data from the string
	token = strtok_r(cs_s,p_delim.c_str(),&state);
	while(token!=NULL){
		string key;
		string val;
		stringstream ss;
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
		string key;
		string val;
		stringstream ss;
		ss<<strtok_r(token,kv_delim.c_str(),&inner_state);
		ss>>key;ss.clear();
		ss<<strtok_r(NULL,kv_delim.c_str(),&inner_state);
		ss>>val;ss.clear();
		data[key]=val;
		token = strtok_r(NULL,p_delim.c_str(),&state);
	}
	if(data["Zc"].size()>0){
		their_count = atoi(data["Zc"].c_str());
	}
	if (data["T"].size()>0){
		last_time = atol(data["T"].c_str());
	}
	if (data["Dt"].size()>0){
		returned_time = atol(data["Dt"].c_str());
	}
	if (data["HaveC"].size()>0){
		HAVE_ZC= atoi(data["HaveC"].c_str());
	}
	std_msgs::String omsg;
	stringstream ss;
	ss<<"Dt="<<last_time<<",";
	ss<<"RendC="<<DESIRED_ZC<<",";
	ss<<"HaveC="<<atoi(data["RendC"].c_str());
	omsg.data = ss.str();
	hbout.publish(omsg);
	free(cs_s);
	free(cs_data);
}

void stop(){
	control::WaypointNavigation::Response wpres;
	control::WaypointNavigation::Request wpreq;
	ROS_INFO("Stopping navigation, if any");
	wpreq.desiredState=2;
	wpreq.blockState=wpreq.BLOCKING;
	if (wpclient.exists()){
		wpclient.call(wpreq,wpres);
	} else {
		ROS_WARN("Skipping waypoint service: Doesn't exist");
	}
}
void move(double x, double y, bool block, bool print){
	control::WaypointNavigation::Response wpres;
	control::WaypointNavigation::Request wpreq;
	if (print)
		ROS_INFO("moving to:%0.3f %0.3f",x,y);

	wpreq.desiredState=0;
	wpreq.destination.x=x;
	wpreq.destination.y=y;

	if (block){
		wpreq.blockState=wpreq.BLOCKING;
	}	else {
		wpreq.blockState=wpreq.NONBLOCKING;
	}

	if (wpclient.exists()){
		wpclient.call(wpreq,wpres);
	} else {
		ROS_WARN("Skipping waypoint service: Doesn't exist");
	}
	if (print)
		ROS_INFO("Move Complete");
}
void sync_robots_at(double x, double y){
	DESIRED_ZC++;
	ROS_INFO("Waiting for joint state at %f, %f",x,y);

	returned_time=0;
	ros::Duration(5).sleep();

	time_t tnow = time(NULL);
	long delta_t=tnow-last_time;

	int their_dc=-1;
	ROS_INFO("It's been %ld since I heard from them",delta_t);

	while( (their_dc!=DESIRED_ZC || HAVE_ZC!=DESIRED_ZC) /*|| delta_t>5 || t_delta_t > 5)*/ && ros::ok()) {
		ros::Duration(.1).sleep();
		move(x,y,false,false);
		their_dc=atoi(data["RendC"].c_str());
	}

	ROS_INFO("Seems we sync'd. Waiting a bit...");
	stop();
	ros::Duration(3).sleep();
	move(x,y,false);
	//assume we have both our state and other guy's state, 
	//as well as any extra data he sent along

	if (!ros::ok()){
		cerr<<"ROS down. Exiting"<<endl;
		return;
	}

	ROS_INFO("Got joint state.");
}

void setupRos(ros::NodeHandle nh){
  wpclient=nh.serviceClient<control::WaypointNavigation>("/control/command");
	bearingclient=nh.serviceClient<bearing_measurement::GetMeasurement>("/bearing_measurement/get_measurement");
	stateIn = nh.subscribe("/robot/state",10,mapstate);
	hbin = nh.subscribe("/network/received",30,nwin);
	hbout = nh.advertise<std_msgs::String>("/network/out",0);
	status_out = nh.advertise<std_msgs::String>("/loc/out",0);
}

int main(int argc, char** argv){

	cx = 0.0;
	cy = 0.0;
	bool test_sync=false;
	bool userend = true;
	bool assume_stationkeep = true;
	simulate=false;

	MatrixXd firstrend(2,1);
	firstrend<<-30,-20;
	MatrixXd r1(2,1);
	MatrixXd r2(2,1);
	FREQ = 49691;

	string fname="";
	ros::init(argc,argv,"tl");
	ros::NodeHandle nh("tagloc");

	setupRos(nh);
	ros::Duration(5).sleep();
	ros::AsyncSpinner spinner(1);

	FILE* log_file = fopen("tagloc_log","a");
	if (log_file==NULL){
		ROS_ERROR("Cannot open log file!");
		return -1;
	}

	double Dei=100.0;

	nh.getParam("test_sync",test_sync);
	ROS_INFO("test_sync=%s ... true will only go, sync, and return.",test_sync?"true":"false");

	nh.getParam("userend",userend);
	ROS_INFO("userend=%s ... true implies we assume we're at the measurement location.",userend?"true":"false");

	nh.getParam("sensor_var",sensor_var);
	ROS_INFO("sensor_var=%f ",sensor_var);

	nh.getParam("dei",Dei);
	ROS_INFO("Dei: %0.3f ... desired information = 1/sigma^2 ",Dei);

	nh.getParam("simulate",simulate);
	ROS_INFO("simulate: %s",simulate?"true":"false");

	nh.getParam("assume_stationkeep",assume_stationkeep);
	ROS_INFO("simulate: %s",simulate?"true":"false");

	int max_measurement = 3;
	nh.getParam("max_measurement",max_measurement);
	ROS_INFO("max_measurement: %d ... max times to call two-step",max_measurement);

	int max_local_measurement= 1;
	nh.getParam("max_local_measurement",max_local_measurement);
	ROS_INFO("max_local_measurement: %d ... max number of measurements at each loc",max_local_measurement);
	if (max_local_measurement>1){
		ROS_WARN("This is a problem: Not ready for many local measurements");
		//return -1;
	}

	if (simulate){
		ROS_WARN("IGNORING FAILURES. SIMULATING");
	}

	char HOSTNAME[HOST_NAME_MAX];
	gethostname(HOSTNAME,HOST_NAME_MAX);
	ROS_INFO("Multi-step. I am %s",HOSTNAME);
#ifdef MAKE_RECIP
	ROS_INFO("I am the second robot!");
	fprintf(log_file,"%s\n","* robot 2");
#else
	ROS_INFO("I am the first robot!");
	fprintf(log_file,"%s\n","* robot 1");
#endif

	if (test_sync && ! simulate){
		ROS_WARN("TESTING SYNC");
		sync_robots_at(20,-80);
		exit(0);
	}

	/*************Main**********/

	spinner.start();

	MatrixXd t(2,1);
	MatrixXd ti(2,1);
	MatrixXd t0(2,1);

	MatrixXd P(2,2);P = MatrixXd::Identity(2,2);
	MatrixXd Pi(2,2); Pi = MatrixXd::Identity(2,2);
	MatrixXd P0(2,2); P0 = MatrixXd::Identity(2,2);

	MatrixXd y(2,1);
	MatrixXd rend(2,1);

	t0<<-10,-150;
	P0<<2500,0,0,2500;
	cout<<t0<<endl;
	cout<<P0<<endl;

	fprintf(log_file,"P: %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f\n",t0(0),t0(1),P0(0),P0(1),P0(2),P0(3));

	ROS_INFO("STARTING RAD:%f",get_radius(P0));
	move(firstrend(0),firstrend(1));

	r1 = firstrend;
	r2 = firstrend;

	P = P0;
	t = t0;

	for (int measurement=0;measurement<max_measurement;measurement++){
		ROS_INFO("**Measurement %d",measurement);
		if (userend){
			r1 = rend;
			r2 = rend;
		}
		RSN::onestep_circle(r1,r2,t,get_radius(P),sensor_var,Dei);
		rend = (r1+r2)/2;
#ifdef MAKE_RECIP
		y = r2;
		r2 = r1;
		r1 = y;
#else
#endif

		ROS_INFO("Got destination: %0.3f,%0.3f. Other guy:%0.3f,%0.3f",r1(0),r1(1),r2(0),r2(1));

		for (int local_measurement = 0;local_measurement<max_local_measurement;local_measurement++){

			fprintf(log_file,"M: %4.4f %4.4f %4.4f %4.4f\n",r1(0),r1(1),r2(0),r2(1));
			move(r1(0),r1(1),true);

			double bearing;
			bool zok = false;
			if (simulate){
				stringstream ss;
				ss<<r1(0);
				data["_x"]=ss.str();ss.str("");
				ss<<r1(1);
				data["_y"]=ss.str();ss.str("");
				bearing=atan2(t0(1)-r1(1),t0(0)-r1(0));
				zok = true;
			} else{
				zok = measure(FREQ,bearing);
			}

			if (zok){
				//r1<<atof(data["_x"].c_str()),atof(data["_y"].c_str());
				Z.push_back(bearing);
				R.push_back(sqrt(sensor_var));
				if (assume_stationkeep){
					px.push_back(r1(0));
					py.push_back(r1(1));
					//publish_z(r1(0),r1(1),bearing,sqrt(sensor_var));
				}	else{
					px.push_back(cx);
					py.push_back(cy);
					//publish_z(cx,cy,bearing,sqrt(sensor_var));
				}
				fprintf(log_file,"Z: %4.4f %4.4f %4.4f %4.4f\n",r1(0),r1(1),bearing,sensor_var);
				fprintf(log_file,"Y: %4.4f %4.4f %4.4f %4.4f\n",cx,cy,bearing,sensor_var);
			} else {
				ROS_INFO("Bad measurement");
			}
		}

		ROS_INFO("Publishing Measurement value over network");
		publish_all_z(px,py,Z,R);
		ROS_INFO("Going to rendesvous point");

		move(rend(0),rend(1),true);
		fprintf(log_file,"M: %4.4f %4.4f \n",rend(0),rend(1));
		if (!simulate || test_sync){
			sync_robots_at(rend(0),rend(1));
		}
		if (!ros::ok()){
			return EXIT_FAILURE;
		}

		ROS_INFO("Examining their data");
		get_all_meausrements(px,py,Z,R,opx,opy,oZ,oR,apx,apy,aZ,aR);

		//int theirs = atoi(data["Zc"].c_str());
		/*
		if (max_local_measurement>1){
			//send ours to be joined with theirs into all
			// this is safe to do if they failed
			get_all_meausrements(px,py,Z,R,opx,opy,oZ,oR,apx,apy,aZ,aR);
		}else{
			//insert our most recent
			apx.insert(apx.end(),px[px.size()-1]);
			apy.insert(apy.end(),py[py.size()-1]);
			aZ.insert(aZ.end(),Z[Z.size()-1]);
			aR.insert(aR.end(),R[R.size()-1]);
			//record their most recent
			get_last_measurement(opx,opy,oZ,oR);
			//insert their most recent
			apx.insert(apx.end(),opx[opx.size()-1]);
			apy.insert(apy.end(),opy[opy.size()-1]);
			aZ.insert(aZ.end(),oZ[oZ.size()-1]);
			aR.insert(aR.end(),oR[oR.size()-1]);
		}
		if ((size_t)theirs == 0){
			ROS_INFO("They have no measurements");
		} else if ((size_t)theirs >= Z.size()){
			ROS_INFO("%Zu (mine) %d (theirs)",Z.size(),theirs);
			ROS_INFO("Looks good.");
		} else if ((size_t) theirs < Z.size()){
			ROS_INFO("They missed a measurement!");
		}*/

		ROS_INFO("Got Joint State. Updating hypothesis with %Zu total measurements",aZ.size());
		ROS_INFO("Measurement Sequence is:");
		for (unsigned int i=0;i<aZ.size();i++){
			cout<<apx[i]<<" "<<apy[i]<<" "<<aZ[i]<<" "<<aR[i]<<endl;
		}

		double tz[2];
		double c[4];

		RSN::line_intersection(aZ.size(),apx,apy,aZ,tz[0],tz[1]);
		RSN::BOT::IWLS_2D(aZ.size(),apx,apy,aZ,aR,tz,c,true);

		ti<<tz[0],tz[1];
		Pi<<c[0],c[1],c[2],c[3];

		ROS_INFO("Z gives:");
		cout<<ti<<endl;
		cout<<Pi<<endl;

		P = (Pi.inverse()+P0.inverse()).inverse();
		t =  P * (Pi.inverse() * ti + P0.inverse() * t0);

		cout<<"With Prior:"<<endl;
		cout<<t<<endl;
		cout<<P<<endl;

	}

	fprintf(log_file,"* \n");
	fclose(log_file);
	move(0,-20,true);
	spinner.stop();
	ros::Duration(3).sleep();
	return 0;
}

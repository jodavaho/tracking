//SYS
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <assert.h>

//ROS
#include <paramon/paramon.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tagloc/helpers.h>
#include <husky/Constants.h>
#include <tagloc/Tag.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <tagloc/AddTag.h>
#include <tagloc/SaveTarget.h>
#include <tagloc/LoadTarget.h>
#include <tagloc/GetTag.h>
#include <tagloc/CheckTag.h>
#include <tagloc/TrigCommand.h>
#include <tagloc/TrigManyCommand.h>
#include <tagloc/tagloc.h>
#include <tagloc/helpers.h>
#include <tagloc/UpdateStep.h>
#include <tagloc/GreedyStep.h>
#include <triangulation/Target_State.h>
#include <triangulation/GetMeasurement.h>
#include <tagloc/Measurement.h>
#include <husky/Goto.h>
#include <husky/HuskyState.h>

#define R (M_PI/8)
#define Cd .1 
#define SIG_B .6425

namespace tagloc{
}

std::string itoa(int i){
	std::stringstream ss;
	ss<<i;
	std::string ret;
	return ss.str();
}

//internal:
std::map<int,tagloc::Tag> hypothesis;
std::map<int,tagloc::Tag> initialHypothesis;
std::map<int,int> numZ;
double ConstantReduction;
double SigB2;
double bias;
bool trustHusky;
bool simulation;
int currentTarget;
std::vector<int> toTrack;
pthread_t tracker;
bool verbose;
bool returnatstart;
bool alwaysInitialize;

bool done;
double R2;

husky::HuskyState lastState;
cv::Mat_<double> Eigs = cv::Mat::eye(2,2,CV_64F);

//ros:
ros::Publisher waypointout;
ros::Subscriber waypointdone;
ros::Subscriber currentState;
ros::ServiceServer taglocService;
ros::ServiceServer taglocServiceMultiple;
ros::ServiceServer stop;
ros::ServiceServer wait;
ros::ServiceServer addHypothesis;
ros::ServiceServer initHypothesis;
ros::ServiceServer get_Hypothesis;
ros::ServiceServer checkHypothesis;
ros::Publisher target_State;
ros::ServiceServer updateTarget;
ros::ServiceServer saveTarget_H;
ros::ServiceServer loadTarget_H;
ros::ServiceServer greedyStepService;
ros::ServiceServer releaseAll;
ros::ServiceServer measureService;
ros::Subscriber paramonListener;

//----------------------------------------_Functions
//

//server functions:
	void setTargetInfo(int freq, cv::Mat_<double> pose, cv::Mat_<double> cv);
	void stopChassis();
	bool trigStart(tagloc::TrigCommand::Request &req, tagloc::TrigCommand::Response &res);
	bool trigStartMultiple(tagloc::TrigManyCommand::Request &req, tagloc::TrigManyCommand::Response &res);
	cv::Mat_<double> getCurrentState();
	bool checkTarget_Service(tagloc::CheckTag::Request& req, tagloc::CheckTag::Response& res);
	triangulation::GetMeasurementResponse getMaxSS(int f);
	bool getMeasurementService(tagloc::Measurement::Request& req, tagloc::Measurement::Response& res);
	bool loadTarget_Service(tagloc::LoadTarget::Request&req,tagloc::LoadTarget::Response&res);
	bool saveTarget_Service(tagloc::SaveTarget::Request&req,tagloc::SaveTarget::Response&res);
	bool getMeasurementService(tagloc::Measurement::Request& req, tagloc::Measurement::Response& res);
	cv::Mat_<double> move(cv::Mat_<double>);
	void publishTarget_State(int currentTarget, cv::Mat_<double> xr);
	void publishTarget_State(int currentTarget, cv::Mat_<double> xr, triangulation::GetMeasurementResponse z);
//

	/*this should be a class. But it isn't. It'd be handy to link in this code so we dont' *need* to use ros*/
//class Tagloc{
	void getTargetInfo(int freq, cv::Mat_<double>& pose, cv::Mat_<double>& cv);
	bool trigAddHypo(tagloc::AddTag::Request &req, tagloc::AddTag::Response &res);
	cv::Mat_<double> doGreedyStep(int f,cv::Mat_<double> cur);
	bool checkTargetGoal(int f);
	void initTarget_State(int f);
	void getInitTargetInfo(int freq, cv::Mat_<double>& pose, cv::Mat_<double>& cv);
	void updateTarget_State(int f,cv::Mat_<double> xr, triangulation::GetMeasurementResponse z);

//};
//------------------------------------_TRACKER
void * trackAFewTargets(void *args){
	ROS_INFO("Multi-Tracker started");
	stopChassis();

	cv::Mat_<double> location = cv::Mat::zeros(2,1,CV_64F);
	if (returnatstart){
		ROS_INFO("Moving back to home");
		location = move(cv::Mat::zeros(2,1,CV_64F));
	} else {
		location = getCurrentState();
		ROS_INFO("Not moving back to home");
	}
	ROS_INFO("triangulation started at [%0.3f, %0.3f]",location(0),location(1));

	bool targetdone[toTrack.size()];
	for (int i=0;i<(int)toTrack.size();i++){
		currentTarget = toTrack[i];	
		targetdone[i] = checkTargetGoal(currentTarget);
		done = done&&targetdone[i];
	}
	if (done){
		ROS_INFO("Target(s) already triangulated");
		return NULL;
	}
	if (alwaysInitialize){
		for (int i=0;i<(int)toTrack.size();i++){
			currentTarget = toTrack[i];	
			ROS_INFO("Initializing hypothesis");
			initTarget_State(currentTarget);
		}
	} else {
		ROS_INFO("Using pre-defined initial state");
	}
	for (int i=0;i<(int)toTrack.size();i++){
		currentTarget = toTrack[i];	
		publishTarget_State(currentTarget,getCurrentState());
	}
	while(!done){
		location = getCurrentState();
		double rangeStep[toTrack.size()];
		int imax=-1;double vmax=DBL_MAX;
		bool mdone = true;
		for (int i=0;i<(int)toTrack.size();i++){
			mdone = mdone && targetdone[i];
			if (targetdone[i]){continue;}
			//else:
			currentTarget = toTrack[i];	
			cv::Mat_<double> dest = doGreedyStep(currentTarget,location);
			rangeStep[i]=cv::norm(dest,location.rowRange(0,2));
			ROS_INFO("Range to %d's Greedy measurement: %0.3f m",currentTarget,rangeStep[i]);
			if (vmax>rangeStep[i]){imax=i;vmax = rangeStep[i];}
		}
		done=mdone;
		if (!done) {
			currentTarget=toTrack[imax];
			cv::Mat tloc = doGreedyStep(currentTarget,location);
			location = move(tloc);
			stopChassis();
		}
		triangulation::GetMeasurementResponse b;
		if (!done) {
			b = getMaxSS(currentTarget);
		}

		if (b.valid.data){
			if (b.bearing!=b.bearing){
				ROS_ERROR("Received %f which I think is not a number. Re-measuring",b.bearing);
				b = getMaxSS(currentTarget);
				ROS_INFO("This time got: %f\n",b.bearing);
				if (b.bearing!=b.bearing){
					ROS_ERROR("NAN again: Cannot use this rubbish... Exiting tracker thread");
					done = true;
					return NULL;
				}
			}
			if (!done){
				location = getCurrentState();
				updateTarget_State(currentTarget,location,b);
				publishTarget_State(currentTarget,location,b);
				targetdone[imax] = checkTargetGoal(currentTarget);
			}
		} else {
			ROS_WARN("Either trig failed or we are done. Either way I'm exiting the thread");
			done = true;
		}
	}
	ROS_INFO("Tracking thread done");
	return NULL;
}
void * trackTarget(void *threadid){
	ROS_INFO("Tracker started");
	stopChassis();

	cv::Mat_<double> location = cv::Mat::zeros(2,1,CV_64F);
	if (returnatstart){
		ROS_INFO("Moving back to home");
		location = move(cv::Mat::zeros(2,1,CV_64F));
	} else {
		location = getCurrentState();
		ROS_INFO("Not moving back to home");
	}
	ROS_INFO("triangulation started at [%0.3f, %0.3f]",location(0),location(1));
	

	done = checkTargetGoal(currentTarget);
	if (done){
		ROS_INFO("Target already triangulated");
		return NULL;
	}
	if (alwaysInitialize){
		ROS_INFO("Initializing hypothesis");
		initTarget_State(currentTarget);
	} else {
		ROS_INFO("Using pre-defined initial state");
	}
	publishTarget_State(currentTarget,getCurrentState());
	while(!done){
		location = getCurrentState();
		cv::Mat_<double> dest = doGreedyStep(currentTarget,location);
		if (!done) {
			location = move(dest);
			stopChassis();
		}
		triangulation::GetMeasurementResponse b;
		if (!done) {
			b = getMaxSS(currentTarget);
		}

		if (b.valid.data){
			if (b.bearing!=b.bearing){
				ROS_ERROR("Received %f which I think is not a number. Re-measuring",b.bearing);
				b = getMaxSS(currentTarget);
				ROS_INFO("This time got: %f\n",b.bearing);
				if (b.bearing!=b.bearing){
					ROS_ERROR("NAN again: Cannot use this rubbish... Exiting tracker thread");
					done = true;
					return NULL;
				}
			}
			if (!done){
				location = getCurrentState();
				updateTarget_State(currentTarget,location,b);
				publishTarget_State(currentTarget,location,b);
				done = checkTargetGoal(currentTarget);
			}
		} else {
			ROS_WARN("Invalid Measurement from triangulation. Resetting target state");
			done = true;
		}

	}
	ROS_INFO("Tracking thread done");
	return NULL;
}
//----------------------------------------_END

cv::Mat_<double> getCurrentState(){
	cv::Mat_<double> Xr = cv::Mat::zeros(3,1,CV_64F);
	Xr(0) = lastState.pose.x;
	Xr(1) = lastState.pose.y;
	Xr(2) = lastState.pose.theta;
	return Xr;
}
void stopChassis(){
	ROS_INFO("Ordering chassis stop");
	std_srvs::Empty mt;
	if (simulation){return;}
	bool ok = ros::service::call(HUSKY_CLEAR_NAVIGATION,mt.request,mt.response);
	if (!ok){
		ROS_ERROR("Could not stop chassis!?");
	}
}

bool checkTarget_Service(tagloc::CheckTag::Request& req, tagloc::CheckTag::Response& res){
	ROS_INFO("Heard request for status of %d",req.freq);
	int f = req.freq;
	bool tdone = checkTargetGoal(f);
	res.done = tdone;
	return true;
}

void getInitTargetInfo(int freq, cv::Mat_<double>& pose, cv::Mat_<double>& cov){
	ROS_INFO("Getting Init Hypothesis info for: %d",freq);
	tagloc::Tag t = initialHypothesis[freq];
	pose(0) = t.target_pose.x;
	pose(1) = t.target_pose.y;
	cov(0,0) = t.target_cov[0];
	cov(0,1) = t.target_cov[1];
	cov(1,0) = t.target_cov[2];
	cov(1,1) = t.target_cov[3];
}
void getTargetInfo(int freq, cv::Mat_<double>& pose, cv::Mat_<double>& cov){
	ROS_INFO("Getting target info for: %d",freq);
	tagloc::Tag t = hypothesis[freq];
	ROS_INFO("OK");
	pose(0) = t.target_pose.x;
	pose(1) = t.target_pose.y;
	cov(0,0) = t.target_cov[0];
	cov(0,1) = t.target_cov[1];
	cov(1,0) = t.target_cov[2];
	cov(1,1) = t.target_cov[3];
}
void saveTargetInfo(int freq){
	cv::Mat_<double> pose = cv::Mat::zeros(2,1,CV_64F);
	cv::Mat_<double> cov = cv::Mat::zeros(2,2,CV_64F);
	getTargetInfo(freq,pose,cov);
	ROS_INFO("Got");
	ros::NodeHandle nh("/taginfo");
	std::string s;
	s= "taginfo/"+itoa(freq);
	ROS_INFO("Saving to: %s",s.c_str());
	s = "taginfo/"+itoa(freq)+"/x"	 ;
	nh.setParam(s.c_str(),pose(0));
	s = "taginfo/"+itoa(freq)+"/y";
	nh.setParam(s.c_str(),pose(1));
	s = "taginfo/"+itoa(freq)+"/c1";
	nh.setParam(s.c_str(),(cov(0,0)));
	s = "taginfo/"+itoa(freq)+"/c2";
	nh.setParam(s.c_str(),(cov(0,1)));
	s = "taginfo/"+itoa(freq)+"/c3";
	nh.setParam(s.c_str(),(cov(1,0)));
	s = "taginfo/"+itoa(freq)+"/c4";
	nh.setParam(s.c_str(),(cov(1,1)));
}
void loadTargetInfo(int freq){
	cv::Mat_<double> pose = cv::Mat::zeros(2,1,CV_64F);
	cv::Mat_<double> cov = cv::Mat::zeros(2,2,CV_64F);
	ros::NodeHandle nh("taginfo");
	std::string s;

	double iparm;
	s = "taginfo/"+itoa(freq)+"/x";
	nh.getParam(s,iparm);
	pose(0)=iparm;

	s = "taginfo/"+itoa(freq)+"/y";
	nh.getParam(s,iparm);
	pose(1)=iparm;

	s = "taginfo/"+itoa(freq)+"/c1";
	nh.getParam(s,iparm);
	cov(0,0)=iparm;

	s = "taginfo/"+itoa(freq)+"/c2";
	nh.getParam(s,iparm);
	cov(0,1)=iparm;
	
	s = "taginfo/"+itoa(freq)+"/c3";
	nh.getParam(s,iparm);
	cov(1,0)=iparm;
	
	s = "taginfo/"+itoa(freq)+"/c4";
	nh.getParam(s,iparm);
	cov(1,1)=iparm;

	setTargetInfo(freq,pose,cov);
}
void setTargetInfo(int freq, cv::Mat_<double> pose, cv::Mat_<double> cv){
	ROS_INFO("Setting target info for: %d to [%0.3f,%0.3f], etc",freq,pose(0),pose(1));
	
	tagloc::Tag t = hypothesis[freq];
	t.target_pose.x = pose(0);
	t.target_pose.y = pose(1);
	t.target_cov[0] = cv(0,0);
	t.target_cov[1] = cv(0,1);
	t.target_cov[2] = cv(1,0);
	t.target_cov[3] = cv(1,1);
	hypothesis[freq] = t;

}
bool greedyService(tagloc::GreedyStep::Request& req, tagloc::GreedyStep::Response& res){
	int f = req.frequency;
	cv::Mat_<double> next = cv::Mat::zeros(2,1,CV_64F);
	next = doGreedyStep(f,next);
	res.gx=next(0);
	res.gy=next(1);
	return true;
}
bool updateService(tagloc::UpdateStep::Request& req, tagloc::UpdateStep::Response& res){
	int f = req.frequency;
	double z = req.Zlocal;
	double fx = req.fromX;
	double fy = req.fromY;
	double th = req.fromTH;
	cv::Mat_<double> xr = cv::Mat::zeros(2,1,CV_64F);
	triangulation::GetMeasurementResponse bz;
	bz.bearing = z;
	std_msgs::Bool isValid;
	isValid.data =1;
	bz.valid = isValid;
	xr(0) = fx;
	xr(1) = fy;
	xr(2) = th;
	updateTarget_State(f,xr,bz);
	return true;
}

void publishTarget_State(int currentTarget, cv::Mat_<double> xr){
	cv::Mat_<double> xp = cv::Mat::zeros(2,1,CV_64F);
	cv::Mat_<double> Pp = cv::Mat::zeros(2,2,CV_64F);
	getTargetInfo(currentTarget,xp,Pp);

	triangulation::Target_State ts;
	ts.frequency = currentTarget;
	ts.robot_pose.x = xr(0);
	ts.robot_pose.y = xr(1);
	ts.robot_pose.theta = xr(2);
	ts.target_pose.x = xp(0);
	ts.target_pose.y = xp(1);
	ts.target_cov[0] = Pp(0,0);
	ts.target_cov[1] = Pp(0,1);
	ts.target_cov[2] = Pp(1,0);
	ts.target_cov[3] = Pp(1,1);

	target_State.publish(ts);
	ROS_INFO("Stored Target State: f:%d r:(%0.3f,%0.3f) t:(%0.3f,%0.3f) c:[%0.3f,%0.3f,%0.3f,%0.3f]",
			currentTarget,
			xr(0),xr(1),
			xp(0),xp(1),
			Pp(0,0),Pp(0,1),
			Pp(1,0),Pp(1,1)
			);
}
void publishTarget_State(int currentTarget, cv::Mat_<double> xr, triangulation::GetMeasurementResponse z){
	cv::Mat_<double> xp = cv::Mat::zeros(2,1,CV_64F);
	cv::Mat_<double> Pp = cv::Mat::zeros(2,2,CV_64F);
	getTargetInfo(currentTarget,xp,Pp);
	
	double globalZ = xr(2)+tagloc::degToRad(z.bearing);
	globalZ = tagloc::normalizeAngle(globalZ);

	triangulation::Target_State ts;
	ts.bearing = globalZ;
	ts.frequency = currentTarget;
	ts.robot_pose.x = xr(0);
	ts.robot_pose.y = xr(1);
	ts.robot_pose.theta = xr(2);
	ts.target_pose.x = xp(0);
	ts.target_pose.y = xp(1);
	ts.target_cov[0] = Pp(0,0);
	ts.target_cov[1] = Pp(0,1);
	ts.target_cov[2] = Pp(1,0);
	ts.target_cov[3] = Pp(1,1);

	target_State.publish(ts);
	ROS_INFO("Target State: f:%d r:(%0.3f,%0.3f) t:(%0.3f,%0.3f) c:[%0.3f,%0.3f,%0.3f,%0.3f] z:%0.3f",
			currentTarget,
			xr(0),xr(1),
			xp(0),xp(1),
			Pp(0,0),Pp(0,1),
			Pp(1,0),Pp(1,1),
			globalZ
			);
}
/**
 * Please, for the love of god pass in bearing in degrees in local frame, and xr in radians in map frame.
 */
void updateTarget_State(int currentTarget, cv::Mat_<double> xr, triangulation::GetMeasurementResponse z){
	ROS_INFO("Update target state for %d",currentTarget);
	if (!z.valid.data){
		ROS_INFO("Invalid measurement!, resetting");
		done = true;
		return;
	}
	cv::Mat_<double> tp = cv::Mat::zeros(2,1,CV_64F);
	cv::Mat_<double> P = cv::Mat::zeros(2,2,CV_64F);
	getTargetInfo(currentTarget,tp,P);
	double globalZ = xr(2)+tagloc::degToRad(z.bearing);
	globalZ = tagloc::normalizeAngle(globalZ);

	ROS_INFO("Updating: freq=%d \n\t at [%0.3f,%0.3f] \n\t with z=[%0.3f+%0.3f=%0.3f]\n\t x_robot = [%0.3f,%0.3f]",currentTarget,tp(0),tp(1),z.bearing,xr(2),globalZ,xr(0),xr(1));
	cv::Mat_<double> H = cv::Mat::zeros(1,2,CV_64F);

	double dx = xr(0)-tp(0);double dy = xr(1)-tp(1);
	double d = sqrt(dx*dx+dy*dy);
	double dd=d*d;
	double h1 = (tp(1)-xr(1));
	double h2 = (xr(0)-tp(0));
	ROS_INFO("*(dd: %0.3f, h1,2=[%0.3f,%0.3f])",dd,h1,h2);
	if (dd>0){
		H(0) = h1/dd;
		H(1) = h2/dd;
	}
	ROS_INFO("*H:[%0.3f %0.3f]",H(0),H(1));
	cv::Mat_<double> S=H*P*H.t()+(R2);
	double zhypoth = atan2(tp(1)-xr(1),tp(0)-xr(0));
	double innov = tagloc::diff(zhypoth,globalZ,-M_PI,M_PI);

	innov=tagloc::normalizeAngle(innov);

	if (fabs(innov)>M_PI/2){
		innov = tagloc::normalizeAngle(innov+M_PI);
	}

	cv::Mat_<double>K=P*H.t()*S.inv();
	cv::Mat_<double> xx = K*innov;
	cv::Mat_<double> xp = tp+xx;
	cv::Mat_<double> Pp=(cv::Mat::eye(2,2,CV_64F)-K*H)*P;

	ROS_INFO("*(innov: %0.3f, z_hat %0.3f, S: %0.3f)",innov,zhypoth,S(0));
	setTargetInfo(currentTarget,xp,Pp);
	return;
}

bool checkTargetGoal(int mcurrentTarget){
	ROS_INFO("Checking %d ... ",currentTarget);
	bool goalstate=false;

	//current
	cv::Mat_<double> Pc = cv::Mat::zeros(2,2,CV_64F);
	cv::Mat_<double> tpc = cv::Mat::zeros(2,1,CV_64F);
	getTargetInfo(mcurrentTarget,tpc,Pc);
	cv::SVD svdc = cv::SVD(Pc);
	cv::Mat_<double> currentConstants= svdc.w;

	//initial
	cv::Mat_<double> Pi = cv::Mat::zeros(2,2,CV_64F);
	cv::Mat_<double> tpi = cv::Mat::zeros(2,1,CV_64F);
	getInitTargetInfo(mcurrentTarget,tpi,Pi);
	cv::SVD svdi= cv::SVD(Pi);
	cv::Mat_<double> initialConstants = svdi.w;

	double cc1;
	double cc2;
	if (initialConstants(0)==0 || initialConstants(1) ==0){
		cc1 = 1; cc2 = 1;
	}else {
		cc1 = currentConstants(0)/initialConstants(0);
		cc2 = currentConstants(1)/initialConstants(1);
	}

	ROS_INFO("Current constants vs Init %d: %0.3f, %0.3f, desired: %0.3f",mcurrentTarget,cc1,cc2,ConstantReduction);
	if (cc1<=ConstantReduction && cc2<= ConstantReduction){
		goalstate = true;
		ROS_INFO("Finished! ");
	} else {
		ROS_INFO("Not finished ");
	}
	return goalstate;
}

void launch_track_thread(){
	done = false;
	pthread_create(&tracker,NULL,trackTarget, (void *)NULL);
}

void launch_multi_track_thread(std::vector<int> todo){
	toTrack = todo;
	done = false;
	pthread_create(&tracker,NULL,trackAFewTargets,(void*)NULL);
}
cv::Mat_<double> move(cv::Mat_<double> dest){
	ROS_INFO("Moving to: %0.3f %0.3f\n",dest(0),dest(1));
	husky::Goto gt;
	gt.request.desired.x = dest(0);
	gt.request.desired.y = dest(1);
//if (simulation){
//	ROS_WARN("Simulating movement!");
//	ros::Duration(1.0).sleep();
//	lastState.pose.x = dest(0);
//	lastState.pose.y = dest(1);
//	lastState.pose.theta = 0;
//	dest(2)=0;
//	return dest;
	//}else	
	if (ros::service::call(HUSKY_GOTO_SERVICE,gt.request,gt.response)){
		ROS_INFO("Response received");
		dest(0)=gt.response.achieved.x;
		dest(1)=gt.response.achieved.y;
		dest(2)=gt.response.achieved.theta;
		ROS_INFO("Response: %0.3f %0.3f %0.3f",dest(0),dest(1),dest(2));
	} else {
		ROS_ERROR("failure from husky. Resetting");
		done = true;
	}
	return dest;
}

void initTarget_State(int f){
	triangulation::GetMeasurementResponse mr = getMaxSS(f);
	if (mr.valid.data){
		if (mr.bearing != mr.bearing){ 
			ROS_ERROR("NAN"); 
			mr = getMaxSS(f);
		}
		if (mr.bearing != mr.bearing){
			ROS_ERROR("NAN again! defaulting to zero."); 
			mr.bearing=0;
		}
		double globalZ = lastState.pose.theta+tagloc::degToRad(mr.bearing);
		globalZ = tagloc::normalizeAngle(globalZ);
		ROS_INFO("z: %0.3f-->world_z%0.3f",mr.bearing,globalZ);

		cv::Mat_<double> Rot = cv::Mat::zeros(2,2,CV_64F);
		double th = globalZ;
		Rot(0,0)=cos(th);
		Rot(0,1)=-sin(th);
		Rot(1,1)=cos(th);
		Rot(1,0)=sin(th);
		cv::Mat_<double> CC = Rot*Eigs*(Rot.t());
		cv::Mat_<double> XR = cv::Mat::zeros(2,1,CV_64F);
		tagloc::AddTag::Request ad;
		tagloc::AddTag::Response res;
		ad.toAdd.frequency = f;
		ad.toAdd.target_pose.x = lastState.pose.x;
		ad.toAdd.target_pose.y = lastState.pose.y;
		XR(0)=lastState.pose.x;
		XR(1)=lastState.pose.y;
		ad.toAdd.target_cov[0]=CC(0,0);
		ad.toAdd.target_cov[1]=CC(0,1);
		ad.toAdd.target_cov[2]=CC(1,0);
		ad.toAdd.target_cov[3]=CC(1,1);
		trigAddHypo(ad,res);
		initialHypothesis[f] = ad.toAdd;
		cv::Mat_<double> xr = cv::Mat::zeros(3,1,CV_64F);
	} else {
		ROS_INFO("Invalid initialization");
	}
}
triangulation::GetMeasurement::Response getMaxSS(int f){
	ROS_INFO("Getting bearing measure for %d",f);
	triangulation::GetMeasurement gm;
	gm.request.freq = f;
	if (simulation){
		ros::Duration(1.0).sleep();
		gm.response.bearing = 0;
		gm.response.valid.data=1;
	} else	if (!ros::service::call("/triangulation/get_measurement",gm)||!gm.response.valid.data){
		ROS_INFO("failure from triang. Resetting");
		done = true;
	} else {
		ROS_INFO("Received local measurement: %0.3f",gm.response.bearing);
	}
	gm.response.bearing = gm.response.bearing - bias;
	return gm.response;
}
cv::Mat_<double> doGreedyStep(int mcurrentTarget,cv::Mat_<double> cur){
	ROS_INFO("Calculating Greedy Step");
	cv::Mat_<double> Ret = cv::Mat::zeros(2,1,CV_64F);
	cv::Mat_<double> P = cv::Mat::zeros(2,2,CV_64F);
	cv::Mat_<double> tp = cv::Mat::zeros(2,1,CV_64F);
	getTargetInfo(mcurrentTarget,tp,P);
	cv::SVD svd = cv::SVD(P);
	cv::Mat_<double> A = svd.u;
	cv::Mat_<double> B = svd.w;
	cv::Mat_<double> Rot = cv::Mat::zeros(2,2,CV_64F);
	double th = atan2(A(1,0),A(0,0));
	double r = sqrt(B(0)/(SigB2-R2));
	cv::Mat_<double> pp1 = cv::Mat::zeros(2,1,CV_64F);
	cv::Mat_<double> pp2 = cv::Mat::zeros(2,1,CV_64F);
	double p2 = M_PI/2;
	pp1(0) = cos(th+p2)*r;
	pp1(1) = sin(th+p2)*r;
	pp2(0) = cos(th-p2)*r;
	pp2(1) = sin(th-p2)*r;
	pp1 = pp1 + tp;
	pp2 = pp2 + tp;
	ROS_INFO("th: %0.3f, r: %0.3f",th,r);
	ROS_INFO("cur: [%0.3f,%0.3f] to pp1: [%0.3f %0.3f] pp2: [%0.3f,%0.3f]",cur(0),cur(1),pp1(0),pp1(1),pp2(0),pp2(1));
	if (cur.size()!=pp1.size()){
		cur = cur(cv::Range(0,2),cv::Range::all());
	}
	if (norm(pp1-cur)<norm(pp2-cur)){
		Ret=pp1;
	} else{
		Ret=pp2;
	}
	return Ret;
}
bool trigwait(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	ROS_INFO("waiting until trig is done");
	while (!done){
		ros::Duration(.1).sleep();
	}
	return true;
}
bool trigstop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	ROS_INFO("Heard trig stop\n");
	done = true;
	return true;
}
bool trigStartMultiple(tagloc::TrigManyCommand::Request &req, tagloc::TrigManyCommand::Response &res){
	ROS_INFO("Heard trig start\n");
	if (!done){
		res.ACK=0;
		ROS_ERROR("Cannot, previous triang not done.\n");
		return false;
	}

	std::vector<int> targetlist = req.freqs;
	currentTarget = -1;//unfortunate...
	if (req.C>0){
		ConstantReduction = req.C;
	}
	ROS_INFO("Target set to :%Zu frequencies",targetlist.size());

	for (int i =0;i<(int)targetlist.size();i++){
		hypothesis[targetlist[i]]=initialHypothesis[targetlist[i]];
	}

	launch_multi_track_thread(targetlist);

	res.ACK=1;
	return true;
}
bool trigStart(tagloc::TrigCommand::Request &req, tagloc::TrigCommand::Response &res){
	ROS_INFO("Heard trig start\n");
	if (!done){
		res.ACK=0;
		ROS_ERROR("Cannot, previous triang not done.\n");
		return false;
	}

	int toTrack = req.freq;
	currentTarget = toTrack;
	if (req.C>0){
		ConstantReduction = req.C;
	}
	ROS_INFO("Target set to :%d",toTrack);

	hypothesis[toTrack]=initialHypothesis[toTrack];
	launch_track_thread();

	res.ACK=1;
	return true;
}
bool loadTarget_Service(tagloc::LoadTarget::Request&req,tagloc::LoadTarget::Response&res){
	ROS_INFO("Trying to load");
	int ff = req.freq;
	loadTargetInfo(ff);
	res.ack=1;
	return true;
}
bool saveTarget_Service(tagloc::SaveTarget::Request&req,tagloc::SaveTarget::Response&res){
	ROS_INFO("Trying to save");
	int ff = req.freq;
	saveTargetInfo(ff);
	res.ack=1;
	return true;
}
bool trigAddHypo(tagloc::AddTag::Request &req, tagloc::AddTag::Response &res){
	int f = req.toAdd.frequency;
	initialHypothesis[f] = req.toAdd;
	hypothesis[f] = req.toAdd;
	ROS_INFO("adding (new sz=%d): %d (%0.3f, %0.3f)\n",(int)hypothesis.size(),f,req.toAdd.target_pose.x,req.toAdd.target_pose.y);
	res.ACK=1;
	return true;
}
bool trigGetHypo(tagloc::GetTag::Request &req, tagloc::GetTag::Response &res){
	int f = req.freq;
	tagloc::Tag t = hypothesis[f];
	ROS_INFO("Getting hypothesis (sz=%d) %d (%0.3f, %0.3f) \n",(int)hypothesis.size(),t.frequency,t.target_pose.x,t.target_pose.y);
	res.current = t;
	return true;
}
void huskyStateHandler(const husky::HuskyState::ConstPtr & msg){
	lastState = *msg;
}
void loadparams(ros::NodeHandle nh){
	double dparm;

	if (nh.getParam("Cd",dparm)){
		ConstantReduction = dparm;
	}else{
		ConstantReduction = Cd;
	}
	ROS_INFO("Cd: %0.3f",ConstantReduction);

	if(nh.getParam("eigs1",dparm)){
		Eigs(0,0)=dparm*dparm;
	} else {
		Eigs(0,0)=50*50;
	}

	if(nh.getParam("eigs2",dparm)){
		Eigs(1,1)=dparm*dparm;
	} else {
		Eigs(1,1)=30*30;
	}
	ROS_INFO("init eigs (eigs1, eigs2)=(%f,%f)",Eigs(0,0),Eigs(1,1));

	if (nh.getParam("sigs",dparm)){
		R2 = dparm;
	}else{
		R2 = R*R;
	}
	ROS_INFO("sigs^2: %0.3f",R2);

	if (nh.getParam("sigb",dparm)){
		SigB2 = dparm;
	}else{
		SigB2 = SIG_B;
	}
	ROS_INFO("sigb^2: %0.3f",SigB2);
	bool bparm;
	if (nh.getParam("alwaysInitialize",bparm)){
		alwaysInitialize = bparm;
	}
	ROS_INFO("alwaysInitialize set to %s",alwaysInitialize?"true":"false");
	if (nh.getParam("return",bparm)){
		returnatstart = bparm;
	}
	nh.getParam("verbose",verbose);
	ROS_INFO("Returning: %d",(int)returnatstart);
	if (nh.getParam("trust_husky",bparm)){
		trustHusky = bparm;
	}
	ROS_INFO("Trust husky? %d",(int)trustHusky);
	if (nh.getParam("simulate",bparm)){
		simulation = bparm;
	} else {
		simulation = false;
	}
	if (simulation){
		ROS_WARN("Tagloc.server is SIMULATING");
	}
	if (nh.getParam("bias",dparm)){
		ROS_INFO("Subtracting the following bias (in degrees) from received measurements: %0.3f",dparm);
		bias = dparm;
	} else {
		bias = 0.0;
		ROS_INFO("assuming measurements are unbiased");
	}
}
bool trigInitHypo(tagloc::Measurement::Request& req, tagloc::Measurement::Response& res){
	ROS_INFO("initializing: %d",req.frequency);
	initTarget_State(req.frequency);
	publishTarget_State(req.frequency,getCurrentState());
	return true;
}
bool getMeasurementService(tagloc::Measurement::Request& req, tagloc::Measurement::Response& res){
	ROS_INFO("Heard measurement request for %d",req.frequency);
	triangulation::GetMeasurement::Response ss = getMaxSS(req.frequency);
	res.valid = ss.valid.data;
	res.bearing_local = tagloc::normalizeAngle(tagloc::degToRad(ss.bearing)+lastState.pose.theta);
	return true;
}

void paramonIn(const std_msgs::String::ConstPtr &msg){
	std::string my_ns = ros::this_node::getNamespace();
	if (msg->data.find(my_ns)!=std::string::npos){
		ROS_INFO("Heard one of my params change... %s",msg->data.c_str());
		ros::NodeHandle nh("tagloc");
		loadparams(nh);
	}
}
void setup(ros::NodeHandle nh){
	returnatstart = false;
	trustHusky = false;
	done = true;
	ROS_INFO("synchronized ...\n");
	lastState.pose.x=0;
	lastState.pose.y=0;
	lastState.pose.theta=0;

	taglocService = nh.advertiseService(START_SERVICE,trigStart);
	taglocServiceMultiple = nh.advertiseService(START_SERVICE_MULTIPLE,trigStartMultiple);
	stop = nh.advertiseService(STOP_SERVICE,trigstop);
	wait = nh.advertiseService(WAIT_SERVICE,trigwait);
	initHypothesis = nh.advertiseService(INIT_HYPOTHESIS,trigInitHypo);
	addHypothesis = nh.advertiseService(ADD_HYPOTHESIS,trigAddHypo);
	get_Hypothesis = nh.advertiseService(GET_HYPOTHESIS,trigGetHypo);
	checkHypothesis = nh.advertiseService(CHECK_HYPOTHESIS,checkTarget_Service);
	target_State = nh.advertise<triangulation::Target_State>(TL_TARGET_STATE,5);
	updateTarget = nh.advertiseService(TL_UPDATE_TARGET,updateService);
	saveTarget_H = nh.advertiseService(SAVE_TARGET,saveTarget_Service);
	loadTarget_H = nh.advertiseService(LOAD_TARGET,loadTarget_Service);
	greedyStepService = nh.advertiseService(TL_GET_GREEDY_STEP,greedyService);
	currentState = nh.subscribe<husky::HuskyState>(HUSKY_STATE, 10, &huskyStateHandler);
	measureService = nh.advertiseService(TL_GET_MEASUREMENT,getMeasurementService);
	paramonListener = nh.subscribe<std_msgs::String>("/paramon/changed",10,&paramonIn);
	ROS_INFO("topics and services up ...\n");

}

void addwatches(ros::NodeHandle nh){
	std::string my_ns = "tagloc";
	paramon::addWatch(my_ns+"/Cd");
	paramon::addWatch(my_ns+"/eigs1");
	paramon::addWatch(my_ns+"/eigs2");
	paramon::addWatch(my_ns+"/sigb");
	paramon::addWatch(my_ns+"/sigs");
	paramon::addWatch(my_ns+"/verbose");
	ROS_INFO("Monitoring my parameters!");	
}
void destroy(){
	pthread_cancel(tracker);
}

int main(int argc, char** argv){
	verbose = false;

	alwaysInitialize = false;
	ROS_INFO("Server starting up... \n");

	tagloc::parseArgs(argc,argv);
	if (!ros::isInitialized()){
		ros::init(argc,argv,"tagloc");
	}

	ros::NodeHandle nh("tagloc");
	
	if (argc<2){
		ROS_INFO("Not saving state or loading state");
	}	else {
		std::string ff = std::string(argv[1])+"/tt";
		/*
		if (loadState(ff.c_str())){
			ROS_INFO("Loaded");
		} else {
			return EXIT_FAILURE;
		}*/
	}
	loadparams(nh);
	addwatches(nh);
	setup(nh);

	ros::MultiThreadedSpinner spinner(8);
	spinner.spin();

	ROS_INFO("Server shutting down\n");
	done = true;
	destroy();
	pthread_exit(NULL);
	return 0;
}

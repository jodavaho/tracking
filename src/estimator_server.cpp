#include <tagloc/tracking.h>
#include <std_msgs/Float64MultiArray.h>
#include <tagloc/Estimation.h>
#include <stdio.h>
#include <ros/ros.h>
#include <helpers/std_msgs.h>

bool linest(tagloc::Estimation::Request &req, tagloc::Estimation::Response &res){
	int n = (int) req.global_pose.size();
	std::vector<double> px;
	std::vector<double> py;
	std::vector<double> Z;
	for (int i=0;i<n;i++){
		px.push_back(req.global_pose[i].pose.position.x);
		py.push_back(req.global_pose[i].pose.position.y);
		Z.push_back(req.global_measurements[i].data);
	}
	double t[2];

	res.global_estimate.pose.position.x=t[0];
	res.global_estimate.pose.position.y=t[1];
	for (int i=0;i<36;i++){
		res.global_estimate.covariance[i]=0;
	}
	return true;
}
bool ekfest(tagloc::Estimation::Request &req, tagloc::Estimation::Response &res){
	int n = (int) req.global_pose.size();
	std::vector<double> px;
	std::vector<double> py;
	std::vector<double> Z;
	std::vector<double> R;
	for (int i=0;i<n;i++){
		px.push_back(req.global_pose[i].pose.position.x);
		py.push_back(req.global_pose[i].pose.position.y);
		Z.push_back(req.global_measurements[i].data);
		R.push_back(req.sensor_variance[i].data);
	}
	double t[2];
	double c[4];
	if (req.use_prior.data){
		t[0] = req.optional_prior.pose.position.x;
		t[1] = req.optional_prior.pose.position.y;
		c[0] = req.optional_prior.covariance[0];
		c[1] = req.optional_prior.covariance[1];
		c[2] = req.optional_prior.covariance[6];
		c[3] = req.optional_prior.covariance[7];
	} else {
		RSN::line_intersection(n,px,py,Z,t[0],t[1],c);
	}
	RSN::BOT::IWLS_2D(n,px,py,Z,R,t,c,req.ambiguous.data);
	for (int i=0;i<n;i++){
		RSN::BOT::EKF_UP_2D(px[i],py[i], Z[i], R[i], t, c, req.ambiguous.data);
	}
	res.global_estimate.pose.position.x=t[0];
	res.global_estimate.pose.position.y=t[1];
	for (int i=0;i<36;i++){
		res.global_estimate.covariance[i]=0;
	}
	res.global_estimate.covariance[0]=c[0];
	res.global_estimate.covariance[1]=c[1];
	res.global_estimate.covariance[6]=c[2];
	res.global_estimate.covariance[7]=c[3];
	return true;
}
bool iekfest(tagloc::Estimation::Request &req, tagloc::Estimation::Response &res){
	int n = (int) req.global_pose.size();
	std::vector<double> px;
	std::vector<double> py;
	std::vector<double> Z;
	std::vector<double> R;
	for (int i=0;i<n;i++){
		px.push_back(req.global_pose[i].pose.position.x);
		py.push_back(req.global_pose[i].pose.position.y);
		Z.push_back(req.global_measurements[i].data);
		R.push_back(req.sensor_variance[i].data);
	}
	double t[2];
	double c[4];
	if (req.use_prior.data){
		t[0] = req.optional_prior.pose.position.x;
		t[1] = req.optional_prior.pose.position.y;
		c[0] = req.optional_prior.covariance[0];
		c[1] = req.optional_prior.covariance[1];
		c[2] = req.optional_prior.covariance[6];
		c[3] = req.optional_prior.covariance[7];
	} else {
		RSN::line_intersection(n,px,py,Z,t[0],t[1],c);
	}
	for (int i=0;i<n;i++){
		RSN::BOT::IEKF_2D(px[i],py[i],Z[i],R[i],t,c,req.ambiguous.data);
	}

	res.global_estimate.pose.position.x=t[0];
	res.global_estimate.pose.position.y=t[1];
	for (int i=0;i<36;i++){
		res.global_estimate.covariance[i]=0;
	}
	res.global_estimate.covariance[0]=c[0];
	res.global_estimate.covariance[1]=c[1];
	res.global_estimate.covariance[6]=c[2];
	res.global_estimate.covariance[7]=c[3];
	return true;
}
bool iwlsest(tagloc::Estimation::Request &req, tagloc::Estimation::Response &res){
	int n = (int) req.global_pose.size();
	std::vector<double> px;
	std::vector<double> py;
	std::vector<double> Z;
	std::vector<double> R;
	for (int i=0;i<n;i++){
		px.push_back(req.global_pose[i].pose.position.x);
		py.push_back(req.global_pose[i].pose.position.y);
		Z.push_back(req.global_measurements[i].data);
		R.push_back(req.sensor_variance[i].data);
	}
	double t[2];
	double c[4];
	if (req.use_prior.data){
		t[0] = req.optional_prior.pose.position.x;
		t[1] = req.optional_prior.pose.position.y;
		c[0] = req.optional_prior.covariance[0];
		c[1] = req.optional_prior.covariance[1];
		c[2] = req.optional_prior.covariance[6];
		c[3] = req.optional_prior.covariance[7];
	} else {
		RSN::line_intersection(n,px,py,Z,t[0],t[1],c);
	}
	RSN::BOT::IWLS_2D(n,px,py,Z,R,t,c,req.ambiguous.data);

	res.global_estimate.pose.position.x=t[0];
	res.global_estimate.pose.position.y=t[1];
	for (int i=0;i<36;i++){
		res.global_estimate.covariance[i]=0;
	}
	res.global_estimate.covariance[0]=c[0];
	res.global_estimate.covariance[1]=c[1];
	res.global_estimate.covariance[6]=c[2];
	res.global_estimate.covariance[7]=c[3];
	return true;
}
bool batchest(tagloc::Estimation::Request &req, tagloc::Estimation::Response &res){
	int n = (int) req.global_pose.size();
	std::vector<double> px;
	std::vector<double> py;
	std::vector<double> Z;
	std::vector<double> R;
	for (int i=0;i<n;i++){
		px.push_back(req.global_pose[i].pose.position.x);
		py.push_back(req.global_pose[i].pose.position.y);
		Z.push_back(req.global_measurements[i].data);
		R.push_back(req.sensor_variance[i].data);
	}
	double t[2];
	double c[4];
	if (req.use_prior.data){
		t[0] = req.optional_prior.pose.position.x;
		t[1] = req.optional_prior.pose.position.y;
		c[0] = req.optional_prior.covariance[0];
		c[1] = req.optional_prior.covariance[1];
		c[2] = req.optional_prior.covariance[6];
		c[3] = req.optional_prior.covariance[7];
	} else {
		RSN::line_intersection(n,px,py,Z,t[0],t[1],c);
	}
	RSN::BOT::Ml_Grad_Asc_2D(n,px,py,Z,R,t,c,req.ambiguous.data,10000);

	res.global_estimate.pose.position.x=t[0];
	res.global_estimate.pose.position.y=t[1];
	for (int i=0;i<36;i++){
		res.global_estimate.covariance[i]=0;
	}
	res.global_estimate.covariance[0]=c[0];
	res.global_estimate.covariance[1]=c[1];
	res.global_estimate.covariance[6]=c[2];
	res.global_estimate.covariance[7]=c[3];
	return true;
}

int main(int argc, char** argv){
	ros::init(argc,argv,"estimator");
	ros::NodeHandle nh("estimator");

	ros::ServiceServer line_estimator = nh.advertiseService("/estimator/line_int",linest);
	ros::ServiceServer ekf_estimator = nh.advertiseService("/estimator/ekf",ekfest);
	ros::ServiceServer iwls_estimator = nh.advertiseService("/estimator/iekf",iekfest);
	ros::ServiceServer iekf_estimator = nh.advertiseService("/estimator/iwls",iwlsest);
	ros::ServiceServer batch_estimator = nh.advertiseService("/estimator/grad",batchest);

	while(ros::ok()){
		ros::spinOnce();
		ros::Duration(.1).sleep();
	}

	return EXIT_SUCCESS;
}

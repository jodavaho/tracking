#include <ros/ros.h>
#include <tagloc/Estimation.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <math.h>
#define USE_MATH_DEFINES

int main(int argc, char** argv){

	ros::init(argc,argv,"tagloc_test");
	ros::NodeHandle nh("estimator");

	tagloc::Estimation::Request req;
	req.global_pose = std::vector<geometry_msgs::PoseWithCovariance>(4);
	req.global_measurements = std::vector<std_msgs::Float64>(4);
	req.sensor_variance = std::vector<std_msgs::Float64>(4);

	tagloc::Estimation::Response res;

	req.global_pose[0].pose.position.x = 2;
	req.global_pose[0].pose.position.y = 0;
	req.global_measurements[0].data = M_PI+.1;
	req.sensor_variance[0].data=.1;

	req.global_pose[1].pose.position.x = 0;
	req.global_pose[1].pose.position.y = 1;
	req.global_measurements[1].data = -M_PI/4+.1;
	req.sensor_variance[1].data=.1;

	req.global_pose[2].pose.position.x = 2;
	req.global_pose[2].pose.position.y = 1;
	req.global_measurements[2].data = -3*M_PI/4+.1;
	req.sensor_variance[2].data=.1;

	req.global_pose[3].pose.position.x = 0;
	req.global_pose[3].pose.position.y = 0;
	req.global_measurements[3].data=0;
	req.sensor_variance[3].data=.1;

	bool ok=false;
	req.use_prior.data = false;	
	ROS_INFO("=======================================================================");
	ROS_INFO("No prior: Expecting approximately [1,0]");
	ok = ros::service::call("/estimator/iwls",req,res);
	ROS_INFO("Test iwls: [%0.3f,%0.3f]",res.global_estimate.pose.position.x,res.global_estimate.pose.position.y);
	ok = ros::service::call("/estimator/grad",req,res);
	ROS_INFO("Test batch: [%0.3f,%0.3f]",res.global_estimate.pose.position.x,res.global_estimate.pose.position.y);
	ok = ros::service::call("/estimator/ekf",req,res);
	ROS_INFO("Test ekf: [%0.3f,%0.3f]",res.global_estimate.pose.position.x,res.global_estimate.pose.position.y);
	ok = ros::service::call("/estimator/iekf",req,res);
	ROS_INFO("Test iekf: [%0.3f,%0.3f]",res.global_estimate.pose.position.x,res.global_estimate.pose.position.y);
	req.use_prior.data = true;	
	req.optional_prior.pose.position.x=1.5;
	req.optional_prior.pose.position.y=1;
	req.optional_prior.covariance[0]=10;
	req.optional_prior.covariance[7]=10;
	ROS_INFO("=======================================================================");
	ROS_INFO("bad prior: Expecting approximately [1,0], but prior will mess things up");
	ok = ros::service::call("/estimator/iwls",req,res);
	ROS_INFO("Test iwls: [%0.3f,%0.3f]",res.global_estimate.pose.position.x,res.global_estimate.pose.position.y);
	ok = ros::service::call("/estimator/grad",req,res);
	ROS_INFO("Test batch: [%0.3f,%0.3f]",res.global_estimate.pose.position.x,res.global_estimate.pose.position.y);
	ok = ros::service::call("/estimator/ekf",req,res);
	ROS_INFO("Test ekf: [%0.3f,%0.3f]",res.global_estimate.pose.position.x,res.global_estimate.pose.position.y);
	ok = ros::service::call("/estimator/iekf",req,res);
	ROS_INFO("Test iekf: [%0.3f,%0.3f]",res.global_estimate.pose.position.x,res.global_estimate.pose.position.y);
	ROS_INFO("=======================================================================");
	req.global_pose[0].pose.position.x = 0;
	req.global_pose[0].pose.position.y = 0;
	req.global_measurements[0].data = 0;
	req.sensor_variance[0].data=.1;

	req.global_pose[1].pose.position.x = 0;
	req.global_pose[1].pose.position.y = 1;
	req.global_measurements[1].data = -M_PI/10;
	req.sensor_variance[1].data=.1;

	req.global_pose[2].pose.position.x = 0;
	req.global_pose[2].pose.position.y = 2;
	req.global_measurements[2].data = -M_PI/5;
	req.sensor_variance[2].data=.1;

	req.global_pose[3].pose.position.x = 0;
	req.global_pose[3].pose.position.y = 0;
	req.global_measurements[3].data=M_PI/20;
	req.sensor_variance[3].data=.1;
	ROS_INFO("prior, longer range");
	ok = ros::service::call("/estimator/iwls",req,res);
	ROS_INFO("Test iwls: [%0.3f,%0.3f]",res.global_estimate.pose.position.x,res.global_estimate.pose.position.y);
	ok = ros::service::call("/estimator/grad",req,res);
	ROS_INFO("Test batch: [%0.3f,%0.3f]",res.global_estimate.pose.position.x,res.global_estimate.pose.position.y);
	ok = ros::service::call("/estimator/ekf",req,res);
	ROS_INFO("Test ekf: [%0.3f,%0.3f]",res.global_estimate.pose.position.x,res.global_estimate.pose.position.y);
	ok = ros::service::call("/estimator/iekf",req,res);
	ROS_INFO("Test iekf: [%0.3f,%0.3f]",res.global_estimate.pose.position.x,res.global_estimate.pose.position.y);
	return (int) !ok; 
}

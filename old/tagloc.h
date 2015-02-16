#ifndef TAGLOC_H
#define TAGLOC_H

#define SIM_SERVICE "/tagloc/set_sim_val"
#define WAIT_SERVICE "/triangulation/wait"
#define START_SERVICE "/triangulation/start"
#define START_SERVICE_MULTIPLE "/triangulation/start_multiple"
#define STOP_SERVICE "/triangulation/stop"

#define ADD_HYPOTHESIS "/triangulation/add_hypothesis"
#define GET_HYPOTHESIS "/triangulation/get_hypothesis"
#define CHECK_HYPOTHESIS "/triangulation/check_hypothesis"
#define INIT_HYPOTHESIS "/triangulation/init"
#define LOAD_TARGET "/tagloc/loadtarget"
#define SAVE_TARGET "/tagloc/savetarget"

#define TL_TARGET_STATE "/triangulation/target_state"
#define TL_GET_MEASUREMENT "/tagloc/get_measurement"
#define TL_UPDATE_TARGET "/triangulation/update_step"
#define TL_GET_GREEDY_STEP "/triangulation/greedy_step"
#define SET_PAN_TOPIC "/motor_cmd/pan"

// DEPRECATED now using a single init call and param to choose
//#define INIT_HYBRID "/initialization/hybrid"
//#define INIT_5ROUND "/initialization/round"
//#define INIT_SEARCH "/initialization/search"
//#define INIT_BATCH "/initialization/batch"

#define START_INIT_SERVICE	"/initialization/start"

#include <stdlib.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <tagloc/helpers.h>
#include <husky/Constants.h>
#include <tagloc/Tag.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <tagloc/AddTag.h>
#include <tagloc/GetTag.h>
#include <tagloc/TrigCommand.h>
#include <tagloc/TrigManyCommand.h>
#include <tagloc/UpdateStep.h>
#include <tagloc/GreedyStep.h>
#include <tagloc/tagloc.h>
#include <tagloc/Measurement.h>
#include <tagloc/CheckTag.h>
#include <std_srvs/Empty.h>

namespace tagloc{

	void addH(ros::NodeHandle nh, int f, float* pose, float* cov);
	void start(ros::NodeHandle nh, int f, int C);
	void stop(ros::NodeHandle nh);
	void printTagObject(tagloc::Tag t);
	void printH(ros::NodeHandle nh, int f);
	void measure(ros::NodeHandle nh, int f);
	void init(ros::NodeHandle nh, int f);
	void wait(ros::NodeHandle nh);
	void check(ros::NodeHandle nh, int f);
//void init_hybrid(ros::NodeHandle nh, int f);
//void init_batch(ros::NodeHandle nh, int f);
//void init_search(ros::NodeHandle nh, int f);

	void check(ros::NodeHandle nh, int f){
		tagloc::CheckTag gt;
		gt.request.freq = f;
		if (ros::service::call(CHECK_HYPOTHESIS,gt.request,gt.response)){
			if (gt.response.done){
				ROS_INFO("Tag: %d appears done.\n",f);
			}
			else {
				ROS_INFO("Tag: %d not done.\n",f);
			}
		} else { 
			ROS_INFO("Failure in checking %d\n",f);
		}

	}

	void wait(ros::NodeHandle nh){
		std_srvs::Empty n;
		if (ros::service::call(WAIT_SERVICE,n.request,n.response)){
			ROS_INFO("Done Waiting\n");
		} else {
			ROS_INFO("Wait failed!");
		}
	}

	void measure(ros::NodeHandle nh, int f){
		tagloc::Measurement command;
		command.request.frequency = f;
		if (ros::service::call(TL_GET_MEASUREMENT,command.request,command.response) && command.response.valid){
			ROS_INFO("Bearing found %0.3f, for freq %d",command.response.bearing_local,f);
		} else {
			ROS_INFO("Bearing failed for freq %d",f);
		}
	}

	tagloc::Measurement::Response getMeasure(ros::NodeHandle nh, int f){
		tagloc::Measurement command;
		command.request.frequency = f;
		if (ros::service::call(TL_GET_MEASUREMENT,command.request,command.response) && command.response.valid){
			ROS_INFO("Bearing found %0.3f, for freq %d",command.response.bearing_local,f);
		} else {
			ROS_INFO("Bearing failed for freq %d",f);
		}
		return command.response;
	}
	void testZ(ros::NodeHandle nh, int f, double z, double rth){
		ros::ServiceClient taglocService = nh.serviceClient<tagloc::UpdateStep>(TL_UPDATE_TARGET);
		tagloc::UpdateStep command;
		command.request.Zlocal= z;
		command.request.frequency=f;
		command.request.fromX = 0;
		command.request.fromY = 0;
		command.request.fromTH = rth;
		if (!taglocService.call(command)){
			ROS_INFO("Failure from tagloc server!\n");
		} else {
			printH(nh,f);
		}
	}

	void start(ros::NodeHandle nh, std::vector<int> todo){
		ros::ServiceClient taglocService = nh.serviceClient<tagloc::TrigManyCommand>(START_SERVICE_MULTIPLE);
		tagloc::TrigManyCommand command;
		command.request.freqs = todo;
		if (!taglocService.call(command)){
			ROS_INFO("Failure from tagloc server: Ack: %d\n",command.response.ACK);
		} else {
			for (int i=0;i<(int)command.request.freqs.size();i++){
				int f = command.request.freqs[i];
				ROS_INFO("Result for tag: %d:\n",f);
				printH(nh,f);
			}
		}
	}
	void start(ros::NodeHandle nh, int f,int Cd){
		ros::ServiceClient taglocService = nh.serviceClient<tagloc::TrigCommand>(START_SERVICE);
		tagloc::TrigCommand command;
		command.request.C = Cd;
		command.request.freq = f;
		if (!taglocService.call(command)){
			ROS_INFO("Failure from tagloc server: Ack: %d\n",command.response.ACK);
		} else {
			ROS_INFO("Result for tag: %d:\n",f);
			printH(nh,f);
		}
	}
	void stop(ros::NodeHandle nh){
		ros::ServiceClient stop = nh.serviceClient<std_srvs::Empty>(STOP_SERVICE);
		std_srvs::Empty asdf;
		stop.call(asdf);
		ros::spinOnce();
	}
	void step(ros::NodeHandle nh, int f){
		ros::ServiceClient	gstep = nh.serviceClient<tagloc::GreedyStep>(TL_GET_GREEDY_STEP);
		tagloc::GreedyStep command;
		command.request.frequency = f;
		gstep.call(command);
		ROS_INFO("Got: %0.3f %0.3f for frequency: %d\n", command.response.gx,command.response.gy,f);
	}
	void addH(ros::NodeHandle nh, int f, float* pose, float* cov){
		ros::ServiceClient	addHypothesis = nh.serviceClient<tagloc::AddTag>(ADD_HYPOTHESIS);
		tagloc::AddTag ad;
		ad.request.toAdd.frequency=f;
		ad.request.toAdd.target_pose.x=pose[0];
		ad.request.toAdd.target_pose.y=pose[1];
		ad.request.toAdd.target_cov.at(0)=cov[0];
		ad.request.toAdd.target_cov.at(1)=cov[1];
		ad.request.toAdd.target_cov.at(2)=cov[2];
		ad.request.toAdd.target_cov.at(3)=cov[3];
		addHypothesis.call(ad);
	}
	void printTagObject(tagloc::Tag t){
		ROS_INFO("f:%d x_t: (%0.3f,%0.3f) c:[%0.3f %0.3f %0.3f %0.3f]",
				t.frequency,
				t.target_pose.x,
				t.target_pose.y,
				t.target_cov.at(0),
				t.target_cov.at(1),
				t.target_cov.at(2),
				t.target_cov.at(3)
				);
	}
	void init(ros::NodeHandle nh, int f){
		tagloc::Measurement command;
		command.request.frequency = f;
		ros::service::call(INIT_HYPOTHESIS,command.request,command.response);
	}
	void printH(ros::NodeHandle nh, int f){
		ros::ServiceClient get_Hypothesis = nh.serviceClient<tagloc::GetTag>(GET_HYPOTHESIS);
		tagloc::GetTag gt;
		gt.request.freq=f;
		get_Hypothesis.call(gt);
		tagloc::Tag t;
		t = gt.response.current;
		printTagObject(t);
	}
}
#endif

#ifndef RSN_TRACKING_ALGORITHMS_H
#define RSN_TRACKING_ALGORITHMS_H

#include <tagloc/tracking.h>
#include <Eigen/Dense>

namespace RSN{
	void onestep_circle(Eigen::MatrixXd &robot_1_world, Eigen::MatrixXd &robot_2_world,Eigen::MatrixXd target_world, double circle_radius, double sensor_variance, double desired_min_info, double scalar=1);

	void onestep(Eigen::MatrixXd &robot_1_world, Eigen::MatrixXd &robot_2_world,Eigen::MatrixXd target_world, Eigen::MatrixXd covariance_matrix, double sensor_variance, double desired_min_info, double scalar=1);
}

#endif

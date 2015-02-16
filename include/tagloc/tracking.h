#ifndef TAGLOC_BEARING
#define TAGLOC_BEARING

#define EIGEN2_SUPPORT
#include <math.h>
#include <helpers/math.h>
#include <Eigen/Dense>
#include <vector>

#define RSN_BEARING (1)
#define RSN_RANGE (2)
#define RSN_RANGE_PLUS_BEARING (3)

namespace RSN{

/*
		void IWLS_2D(const int n, std::vector<std::vector<double> > sensors, const double bearing_global_radians[], const double sensor_sigma[], double t[2], double target_cov[4], const bool ambig=false, const double min_step=.01, const int max_iters=100);
	void IWLS_2D(const int n, const double sensors[][2], const double measurements[], const int measurement_type[], const double sensor_sigma[], double t[2], double target_cov[4], const bool ambig=false, const double min_step=.01, const int max_iters=100);
	void Ml_Grad_Asc_2D(const int n, const double sensors[][2], const double measurements[], const int measurement_type[], const double sensor_sigma[],double t[2], double target_cov[4], const bool ambig=false, const int max_iters=10000);
	void IEKF_2D(const double sensors[2], const double measurements[], const int measurement_type[], const double sensor_sigma, double t[2], double target_cov[4], const bool ambig=false, const double min_step=.01, const int max_iters=10);
	void EKF_UP_2D(const double sensor[2], const double bearing_global_radians, const int measurement_type, const double sensor_sigma, double t[2], double target_cov[4], const bool ambig=false);
	Eigen::MatrixXd get_S(const double sensor[2],const int sensor_type, const double target[2],const double cov[4],const double sensor_sigma);
	Eigen::MatrixXd get_H(const Eigen::MatrixXd sensor,const Eigen::MatrixXd sensor_type, const Eigen::MatrixXd target);
	Eigen::MatrixXd get_H(const double sensor[2], const int sensor_type, const double target[2]);
	Eigen::MatrixXd get_FIM(const Eigen::MatrixXd col_wise_sensors, const Eigen::MatixXd vector_of_types, const Eigen::MatrixXd vector_of_sigma, const Eigen::MatrixXd target);
	*/

	bool in_ellipse(const Eigen::MatrixXd center_pt, const Eigen::MatrixXd Ellipse_Matrix, const Eigen::MatrixXd query_pt, const double scalar=1);
	double distance_to_ellipse(const Eigen::MatrixXd center_pt, const Eigen::MatrixXd Ellipse_Matrix, const Eigen::MatrixXd query_pt, const double scalar=1);
	Eigen::MatrixXd closest_pt_ellipse(const Eigen::MatrixXd center_pt, const Eigen::MatrixXd Cov_Matrix, const Eigen::MatrixXd query_pt, const double scalar=1);
	void circumcircle(const size_t N, double px[], double py[], double &centerx, double &centery, double& radius);
	void circumcircle(const size_t N, const std::vector<std::vector<double> >p, double &centerx, double &centery, double& radius);
	void circumcircle(const size_t N, const std::vector<double> px, std::vector<double> py, double &centerx, double &centery, double& radius);
	void line_intersection(const size_t N, double p[][2] ,double th[], double &px, double &py,double c[4]=NULL);
	void line_intersection(const size_t N, const std::vector<std::vector<double> >p, std::vector<double> th, double &px, double &py,double c[4]=NULL);
	void line_intersection(const size_t N, const std::vector<double> px, std::vector<double> py, std::vector<double> th, double &opx, double &opy,double c[4]=NULL);

	Eigen::MatrixXd getCovMatrix(const double covariance[4]);

	/**
	 * BEARING-ONLY Methods
	 */
	namespace BOT{
		void Ml_Grad_Asc_2D(const int n, const std::vector<double> sx, const std::vector<double> sy, const std::vector<double> bearing_global_radians, const std::vector<double> sensor_sigma,double t[2], double target_cov[4], const bool ambig=false, const int max_iters=10000);
		void Ml_Grad_Asc_2D(const int n, const double sensors[][2], const double bearing_global_radians[], const double sensor_sigma[],double t[2], double target_cov[4], const bool ambig=false, const int max_iters=10000);
		void Ml_Grad_Asc_2D(const int n, const std::vector<std::vector<double> >sensors, const std::vector<double> bearing_global_radians, const std::vector<double> sensor_sigma,double t[2], double target_cov[4], const bool ambig=false, const int max_iters=10000);

		void IWLS_2D(const int n, const std::vector<double>sx, const std::vector<double> sy, const std::vector<double> bearing_global_radians, const std::vector<double> sensor_sigma,double target[2], double target_cov[4], const bool ambig=false, double min_step=.01, int max_iters=20);
		void IWLS_2D(const int n, const std::vector<std::vector<double> >sensors, const std::vector<double> bearing_global_radians, const std::vector<double> sensor_sigma,double target[2], double target_cov[4], const bool ambig=false, double min_step=.01, int max_iters=20);
		void IWLS_2D(const int n, const double sensors[][2], const double bearing_global_radians[], const double sensor_sigma[], double t[2], double target_cov[4], const bool ambig=false, const double min_step=.01, const int max_iters=20);

		void IEKF_2D(const double sensors[2], const double bearing_global_radians, const double sensor_sigma, double t[2], double target_cov[4], const bool ambig=false, const double min_step=.01, const int max_iters=10);
		void IEKF_2D(const double sensors_x, const double sensor_y, const double bearing_global_radians, const double sensor_sigma, double t[2], double target_cov[4], const bool ambig=false, const double min_step=.01, const int max_iters=10);
		void EKF_UP_2D(const double sensor[2], const double bearing_global_radians, const double sensor_sigma, double t[2], double target_cov[4], const bool ambig=false);
		void EKF_UP_2D(const double sensor_x, const double sensor_y, const double bearing_global_radians, const double sensor_sigma, double t[2], double target_cov[4], const bool ambig=false);
		Eigen::MatrixXd get_S(const double sensor[2],const double target[2],const double cov[4],const double sensor_sigma);
		Eigen::MatrixXd get_S(const Eigen::MatrixXd sensor,const Eigen::MatrixXd target, const Eigen::MatrixXd cov,const double sensor_sigma);
		Eigen::MatrixXd get_H(const double sensor[2],const double target[2]);
		Eigen::MatrixXd get_H(const Eigen::MatrixXd sensor,const Eigen::MatrixXd target);
		Eigen::MatrixXd get_FIM(const Eigen::MatrixXd col_wise_sensors, const Eigen::MatrixXd vector_of_sigma, const Eigen::MatrixXd target);
	}


	/**
	 * RANGE-ONLY Methods
	 */

	/*
	namespace ROT{
		void IWLS_2D(const int n, const double sensors[][2], const double range[], const double sensor_sigma[], double t[2], double target_cov[4], const bool ambig=false, const double min_step=.01, const int max_iters=100);
	}*/


}

#endif

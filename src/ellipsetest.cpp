#include <tagloc/tracking.h>
#include <Eigen/Dense>
#define USE_MATH_DEFINES
#include <math.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv){
	Eigen::MatrixXd t = Eigen::MatrixXd::Zero(2,1);	
	Eigen::MatrixXd r = Eigen::MatrixXd::Zero(2,1);	
	r<<10,10;
	double th = M_PI/4;
	Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2,2);
	R<<cos(th),-sin(th),sin(th),cos(th);
	Eigen::MatrixXd P = Eigen::MatrixXd::Zero(2,2);	
	P<<4,0,0,1;
	P = R*P*R.transpose();
	cout<<P<<endl;

	Eigen::MatrixXd cp;
	
	cout<<"---"<<endl;	
	cp = RSN::closest_pt_ellipse(t,P,r,1);
	cout<<"Expect something along [1,1]"<<endl;	
	cout<<cp<<endl;
	
	cout<<"---"<<endl;	
	r<<-10,-10;
	cp = RSN::closest_pt_ellipse(t,P,r,1);
	cout<<"Expect reflection of above"<<endl;	
	cout<<cp<<endl;


	cout<<"---"<<endl;	
	r<<-10,-10;
	cout<<"x:"<<endl;
	cout<<r<<endl;
	th = 0;
	R<<cos(th),-sin(th),sin(th),cos(th);
	P<<4,0,0,1;
	P = R*P*R.transpose();
	cout<<P<<endl;
	cout<<"cp:"<<endl;
	cp = RSN::closest_pt_ellipse(t,P,r,1);
	cout<<cp<<endl;
	
	cout<<"---"<<endl;	
	cout<<"Same as above, but with target shifted to [20,0]"<<endl;	
	t<<20,0;
	r<<-10,-10;
	cout<<"x:"<<endl;
	cout<<r<<endl;
	th = 0;
	R<<cos(th),-sin(th),sin(th),cos(th);
	P<<4,0,0,1;
	P = R*P*R.transpose();
	cout<<P<<endl;
	cout<<"cp:"<<endl;
	cp = RSN::closest_pt_ellipse(t,P,r,1);
	cout<<cp<<endl;
	return 0;
}

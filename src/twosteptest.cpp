#include <ros/ros.h>
#include <iostream>
#include <tagloc/algorithms.h>
#include <tagloc/tracking.h>
#include <Eigen/Dense>
#include <math.h>
#include <Eigen/Geometry>
#include <vector>

double Dei = 100;
double sensor_var=pow((M_PI/12),2);
using namespace std;
double zof(Eigen::MatrixXd T, Eigen::MatrixXd R){
	double nz = (rand()%100-50)/(100.0) * M_PI/(16);
	return atan2(T(1)-R(1),T(0)-R(0))+nz;
}
//useful stuff
double get_radius(Eigen::MatrixXd P){
	assert(P.rows()==P.cols());
	Eigen::JacobiSVD<Eigen::MatrixXd> _svdA;
	_svdA.compute(P);
	return sqrt(_svdA.singularValues()(0));
}
int main(int argc, char** argv){
	if (argc>1){
		Dei = atof(argv[1]);
		sensor_var = atof(argv[2]);
	}
	std::vector<double> px;
	std::vector<double> py;
	std::vector<double> Z;
	std::vector<double> R;
	Eigen::MatrixXd T(2,1);
	Eigen::MatrixXd TT(2,1);
	T<< 0,0;
	TT<< 0,0;
	Eigen::MatrixXd P = Eigen::MatrixXd::Identity(2,2) * 900.0;
	Eigen::MatrixXd r1(2,1),r2(2,1);
	r1 << 200,20;
	r2 << 200,-20;
	int i=0;
	while(get_radius(P)>sqrt(Dei)){
		cout<<"\t\t\ti:"<<i++<<endl;
		px.push_back(r1(0));
		px.push_back(r2(0));
		py.push_back(r1(1));
		py.push_back(r2(1));
		R.push_back((sensor_var));
		R.push_back((sensor_var));
		Z.push_back(zof(TT,r1));
		Z.push_back(zof(TT,r2));
		double ti[2];
		double c[4];
		RSN::BOT::IWLS_2D(px.size(),px,py,Z,R,ti,c,false);
		T<<ti[0],ti[1];
		P<<c[0],c[1],c[2],c[3];
		//P=Eigen::MatrixXd::Identity(2,2)* 900.0;
		double rad = get_radius(P);
		cout<<"--------------------"<<endl;
		cout<<T<<endl;
		cout<<P<<endl;
		cout<<"e1:"<<rad<<endl;
		cout<<"--------------------"<<endl;
		RSN::onestep_circle(r1,r2,T,rad,(sensor_var),Dei);
		Eigen::MatrixXd rot(2,2);
		double th =0; //M_PI/8;
		rot(0,0)=cos(th);
		rot(0,1)=-sin(th);
		rot(1,0)=sin(th);
		rot(1,1)=cos(th);
		r1 = rot * r1;
		r2 = rot * r2;
	}
	cout<<"Got there in "<<i<<endl;
	cout<<T<<endl;
	
	return EXIT_SUCCESS;
}

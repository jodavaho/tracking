#include <tagloc/algorithms.h>
#include <tagloc/tracking.h>
#include <fstream>
#include <iostream>
#include <tagloc/tracking.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#define USE_MATH_DEFINES

using namespace std;
using namespace Eigen;
using namespace RSN;

#define MAX 1024
double getGauss(double variance){
	double t = rand()/((double)RAND_MAX);
	double u = rand()/((double)RAND_MAX);
	//double R = sqrt(-2.0*log(u))*cos(2*M_PI*t) * sqrt(variance);
	double R = rand()/((double)RAND_MAX) * (variance);
	//double R = sqrt(-2.0*log(u))*cos(2*M_PI*t);
	//cout<<t<<"\t"<<u<<"\t"<<R<<endl;
	return R;
}
double getZ(MatrixXd s, MatrixXd t,double var){
	cout<<"get Z:"<<endl;
	cout<<s<<endl;
	cout<<t<<endl;
	double Z = atan2(t(1)-s(1),t(0)-s(0));
	Z = Z+getGauss(var);
	Z = atan2(sin(Z),cos(Z));
	cout<<Z<<endl;
	cout<<"--"<<endl;
	return Z;
}
double getRand(double ub){
	double dr = (rand()/( (double) RAND_MAX ) )*ub;
	return dr;	
}
int main(int argc, char** argv){
	srand(time(NULL));
	ofstream log;
	ofstream log2;
	log.open("./opt-n-conn");
	log2.open("./opt-n-conn-res");

	double variance = .29;
	MatrixXd r1(2,1),r2(2,1),T(2,1);
	double xmax = 1000;
	double xmin = 0;
	double ymax = 1000;
	double ymin = 0;

	double tm = 30;

	T<<xmax/2,ymax/2;
	r1<<getRand(xmax),getRand(ymax);
	r2 = r1;

	cout<<"----------------------------"<<endl;
	cout<<r1<<endl;
	cout<<"----------------------------"<<endl;
	
	vector<double> Z;
	vector<double> R;
	vector<double> px;
	vector<double> py;
	vector<double> tx;
	vector<double> ty;
	vector<double> As;

	MatrixXd zp(2,1);
	zp<<r1(0)+tm/sqrt(2),r1(1)+tm/sqrt(2);
	px.push_back(zp(0));
	py.push_back(zp(1));
	Z.push_back(getZ(zp,T,variance));
	R.push_back(sqrt(variance));
	
	zp<<r1(0)-tm/sqrt(2),r1(1)+tm/sqrt(2);
	px.push_back(zp(0));
	py.push_back(zp(1));
	Z.push_back(getZ(zp,T,variance));
	R.push_back(sqrt(variance));

	zp<<r1(0)-tm/sqrt(2),r1(1)-tm/sqrt(2);
	px.push_back(zp(0));
	py.push_back(zp(1));
	Z.push_back(getZ(zp,T,variance));
	R.push_back(sqrt(variance));

	zp<<r1(0)+tm/sqrt(2),r1(1)-tm/sqrt(2);
	px.push_back(zp(0));
	py.push_back(zp(1));
	Z.push_back(getZ(zp,T,variance));
	R.push_back(sqrt(variance));

	double DA = 50;
	double A = 999999.9;
	double t[2];
	double c[4];
	while(A>DA){
		
		memset(&t,0,2*sizeof(double));	
		memset(&c,0,4*sizeof(double));	

		RSN::line_intersection(px.size(),px,py,Z,t[0],t[1],NULL);
		cout<<"line intersection: "<<t[0]<<"\t"<<t[1]<<endl;
		RSN::BOT::IWLS_2D(px.size(),px,py,Z,R,t,c,true);
		cout<<"IWLS : "<<t[0]<<"\t"<<t[1]<<endl;
		MatrixXd Cov_Matrix(2,2);
		Cov_Matrix<<c[0],c[1],c[2],c[3];
		Eigen::SVD<Eigen::MatrixXd> svd(Cov_Matrix);
		A = (sqrt(svd.singularValues()(0)));
		As.push_back(A);
		tx.push_back(t[0]);
		ty.push_back(t[1]);
		
		cout<<"seq:"<<endl;
		for (int j=0;j<px.size();j++){
			cout<<px[j]<<"\t";
		}
		cout<<endl;

		for (int j=0;j<px.size();j++){
			cout<<py[j]<<"\t";
		}
		cout<<endl;

		for (int j=0;j<px.size();j++){
			cout<<Z[j]<<"\t";
		}
		cout<<endl;
		
		for (int j=0;j<px.size();j++){
			cout<<R[j]<<"\t";
		}
		cout<<endl;

		cout<<"-----------------"<<endl;
		cout<<"Est: "<<endl;
		cout<<t[0]<<"\t"<<t[1]<<endl;
		cout<<"True: "<<endl;
		cout<<T.transpose()<<endl;
		cout<<"Cov: "<<endl;
		cout<<Cov_Matrix<<endl;
		cout<<"|A|:"<<A<<endl;
		cout<<"-----------------"<<endl;

		usleep(1e6);
		MatrixXd mt(2,1);
		mt<<t[0],t[1];
		RSN::onestep_circle(r1,r2,mt,sqrt(A),(variance),DA*DA,3);

		cout<<"spacing: "<<(r1-r2).norm()<<endl;
		px.push_back(r1(0));
		py.push_back(r1(1));
		Z.push_back(getZ(r1,T,variance));
		R.push_back(sqrt(variance));

		px.push_back(r2(0));
		py.push_back(r2(1));
		Z.push_back(getZ(r2,T,variance));
		R.push_back(sqrt(variance));

		//rendezvous

	}
	log2<<T(0)<<","<<T(1)<<","<<t[0]<<","<<t[1];
	int j;
	for (j=0;j<4;j++){
		cout<<","<<c[j];
	}
	cout<<endl;
	for (j=0;j<px.size()-1;j++){
		log<<px[j]<<",";
	}
	log<<px[j]<<endl;

	for (j=0;j<px.size()-1;j++){
		log<<py[j]<<",";
	}
	log<<py[j]<<endl;
	for (j=0;j<px.size()-1;j++){
		log<<Z[j]<<",";
	}
	log<<Z[j]<<endl;
	for (j=0;j<px.size()-1;j++){
		log<<R[j]<<",";
	}
	log<<R[j]<<endl;
	for (j=0;j<As.size()-1;j++){
		log<<As[j]<<",";
	}
	log<<As[j]<<endl;
	for (j=0;j<px.size()-1;j++){
		log<<tx[j]<<",";
	}
	log<<tx[j]<<endl;
	for (j=0;j<px.size()-1;j++){
		log<<ty[j]<<",";
	}
	log<<ty[j]<<endl;
	log.close();
	log2.close();

}

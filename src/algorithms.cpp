#include <tagloc/algorithms.h>
#include <stdio.h>
#include <iostream>
using namespace std;

void RSN::onestep_circle(Eigen::MatrixXd &r1, Eigen::MatrixXd &r2,Eigen::MatrixXd t, double radius, double R, double Dei, double scalar){
	Eigen::MatrixXd cn = (r1+r2)/2.0;

	if ( (cn-t).norm() < radius*scalar ){
#ifdef __TL_VERBOSE
		cout<<"Centroid in"<<endl;
#endif
		return;
	}
	if ( (r2-t).norm() < radius*scalar ){
#ifdef __TL_VERBOSE
		cout<<"r2 in"<<endl;
#endif
		return;
	}
	if ( (r1-t).norm() < radius*scalar ){
#ifdef __TL_VERBOSE
		cout<<"r1 in"<<endl;
#endif
		return;
	}

	Eigen::MatrixXd v = t-cn;
	double dd = v.norm();

	Eigen::MatrixXd cp = (v/dd)*(dd-(radius*scalar))+cn;
#ifdef __TL_VERBOSE
	cout<<__LINE__<<":"<<__FILE__<<endl;
	cout<<__LINE__<<"v:"<<v<<endl;
	cout<<__LINE__<<"cp:"<<cp<<endl;
#endif
	double rr = 20;
	rr = sqrt(Dei/(2*R));
#ifdef __TL_VERBOSE
	cout<<__LINE__<<"RR:"<<rr<<endl;
#endif

	double th = atan2(v(1),v(0));

	Eigen::MatrixXd u1(2,1); u1<<-sin(th),cos(th);
	Eigen::MatrixXd c1 = cp+u1*rr;
	Eigen::MatrixXd c2 = cp-u1*rr;
	Eigen::MatrixXd D = Eigen::MatrixXd::Zero(2,2);
	D(0,0) = (c1-r1).norm();
	D(0,1) = (c1-r2).norm();
	D(1,0) = (c2-r1).norm();
	D(1,1) = (c2-r2).norm();
#ifdef __TL_VERBOSE
	cout<<"Assignment Matrix:"<<endl;
	cout<<D<<endl;
#endif
	Eigen::MatrixXd ur1(2,1);
	Eigen::MatrixXd ur2(2,1);

	if (D.determinant()>0){
		ur1 = (c2-r1);
		ur2 = (c1-r2);
	} else {
		ur1 = (c1-r1);
		ur2 = (c2-r2);
	}

	if (ur1.norm()>rr){
		r1 = r1 + (ur1/ur1.norm()) * (ur1.norm() - rr);
	}
	if (ur2.norm()>rr){
		r2 = r2 + (ur2/ur2.norm()) * (ur2.norm() - rr);
	}
}

void RSN::onestep(Eigen::MatrixXd &r1, Eigen::MatrixXd &r2,Eigen::MatrixXd t, Eigen::MatrixXd P, double R, double Dei, double scalar){
	//check if inside;

	Eigen::MatrixXd cn = (r1 + r2)/2.0;

	if (in_ellipse(t,P,cn,scalar)){
#ifdef __TL_VERBOSE
		cout<<"Centroid in"<<endl;
#endif
		return;
	}
	if (in_ellipse(t,P,r2,scalar)){
#ifdef __TL_VERBOSE
		cout<<"r2 in"<<endl;
#endif
		return;
	}
	if (in_ellipse(t,P,r1,scalar)){
#ifdef __TL_VERBOSE
		cout<<"r1 in"<<endl;
#endif
		return;
	}

	cerr<<"WARN: USING TEST VALUES"<<endl;
	Eigen::MatrixXd cp = closest_pt_ellipse(t,P,cn,scalar);

#ifdef __TL_VERBOSE
	cout<<cp<<endl;
#endif
	double rr = 20;

	Eigen::MatrixXd v = cp-cn;
	double th = atan2(v(1),v(0));

	Eigen::MatrixXd u1(2,1); u1<<-sin(th),cos(th);
	Eigen::MatrixXd c1 = cp+u1*rr;
	Eigen::MatrixXd c2 = cp-u1*rr;
	Eigen::MatrixXd D = Eigen::MatrixXd::Zero(2,2);
	D(0,0) = (c1-r1).norm();
	D(0,1) = (c1-r2).norm();
	D(1,0) = (c2-r1).norm();
	D(1,1) = (c2-r2).norm();
#ifdef __TL_VERBOSE
	cout<<"Assignment Matrix:"<<endl;
	cout<<D<<endl;
#endif
	Eigen::MatrixXd ur1(2,1);
	Eigen::MatrixXd ur2(2,1);

	if (D.determinant()>0){
		ur1 = (c2-r1);
		ur2 = (c1-r2);
	} else {
		ur1 = (c1-r1);
		ur2 = (c2-r2);
	}

	if (ur1.norm()>rr){
		r1 = r1 + (ur1/ur1.norm()) * (ur1.norm() - rr);
	}
	if (ur2.norm()>rr){
		r2 = r2 + (ur2/ur2.norm()) * (ur2.norm() - rr);
	}

}

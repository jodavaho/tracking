#include <tagloc/algorithms.h>
#include <tagloc/tracking.h>
#include <fstream>
#include <iostream>
#include <tagloc/tracking.h>
#include <math.h>
#include <stdio.h>
#define USE_MATH_DEFINES

using namespace std;
using namespace Eigen;
using namespace RSN;

#define MAX 1024
double getGauss(double variance){
	double t = rand()/((double)RAND_MAX);
	double u = rand()/((double)RAND_MAX);
	double R = sqrt(-2.0*log(u))*cos(2*M_PI*t) * sqrt(variance);
	//cout<<t<<","<<u<<","<<R<<endl;
	return R;
}
void getZ(int N, double Z[], double s[][2], double t[], double var){
	for (int i=0;i<N;i++){
		Z[i] = atan2(s[i][1]-t[1],s[i][0]-t[0]);
		Z[i] = Z[i]+getGauss(var);
		Z[i]=atan2(sin(Z[i]),cos(Z[i]));
	}
}
int main(int argc, char** argv){

	int max_trials = 1000;
	stringstream ss;
	ss<<"~/icraout/";
	ofstream fo_iwls,fo_s;
	fo_s.open("sensors_A");
	fo_iwls.open("targets_iwls_A");
	srand(time(NULL));
	double var = .2;

	double xmax = 100;
	double ymax = 100;
	double ymin = 0;
	double xmin = 0;
	double TT[2];

	TT[0]=(xmax-xmin)/2;
	TT[1]=(ymax-ymin)/2;

	int N_s = 10;

	MatrixXd Es(2,MAX);
	MatrixXd Er(MAX,1);
	MatrixXd Et(2,1);
	double R[MAX];
	for (int i=0;i<MAX;i++){
		R[i]=var;
		Er(i)=sqrt(var);
	}
	for (int trial=0;trial<max_trials;trial++){

		Et<<TT[0],TT[1];

		double s[MAX][2];
		double z[MAX];
		for (int si=0;si<N_s;si++){
			s[si][0]=((double)rand()/((double)RAND_MAX))*(xmax-xmin)+xmin;
			s[si][1]=((double)rand()/((double)RAND_MAX))*(ymax-ymin)+ymin;
		}
		getZ(N_s,z,s,TT,var);
		fo_iwls<<trial<<",";
		for (int zi=0;zi<N_s;zi++){
			fo_iwls<<zi<<",";
			fo_s<<s[zi][0]<<","<<s[zi][1]<<","<<z[zi]<<endl;
			Es(0,zi)=s[zi][0];
			Es(1,zi)=s[zi][1];
			double t[2];
			double tp[2];
			double cov[4];
			memset(&cov,0,4*sizeof(double));

			MatrixXd et(2,1);
			MatrixXd fim(2,2);

			RSN::line_intersection(N_s, s, z, tp[0],tp[1]);

			//////////////////?IWLS
			fo_iwls<<TT[0]<<","<<TT[1]<<","<<tp[0]<<","<<tp[1]<<",";

			t[0]=tp[0];
			t[1]=tp[1];

			RSN::BOT::IWLS_2D(N_s, s, z, R, t ,cov,true);
			fo_iwls<<t[0]<<","<<t[1];

			for (int i=0;i<4;i++){
				fo_iwls<<","<<cov[i];
			}

			Et(0) = TT[0];
			Et(1) = TT[1];
			fim = RSN::BOT::get_FIM(Es,Er,Et);
			fim = fim.inverse();
			fo_iwls<<","<<fim(0,0);
			fo_iwls<<","<<fim(0,1);
			fo_iwls<<","<<fim(1,0);
			fo_iwls<<","<<fim(1,1);
			Et(0) = t[0];
			Et(1) = t[1];
			fim = RSN::BOT::get_FIM(Es,Er,Et);
			fim = fim.inverse();
			fo_iwls<<","<<fim(0,0);
			fo_iwls<<","<<fim(0,1);
			fo_iwls<<","<<fim(1,0);
			fo_iwls<<","<<fim(1,1);
			fo_iwls<<endl;
		}
	}
	fo_iwls.close();
	fo_s.close();

}

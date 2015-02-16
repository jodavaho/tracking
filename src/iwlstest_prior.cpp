

#include <tagloc/tracking.h>
#include <math.h>
#include <stdio.h>
#include <Eigen/Dense>
#include <iostream>
using std::cout;
using std::endl;
using Eigen::MatrixXd;

#define USE_MATH_DEFINES
int main(int argc, char** argv){

	std::vector<double> px,py,z,R;
	double t[2],c[4];
	double tp[2],cp[4];

	int i=1;
	tp[0] = atof(argv[i++]);	
	tp[1] = atof(argv[i++]);	
	cp[0] = atof(argv[i++]);	
	cp[1] = atof(argv[i++]);	
	cp[2] = atof(argv[i++]);	
	cp[3] = atof(argv[i++]);

	for(;i<argc-1;i+=3){
		px.push_back(atof(argv[i]));
		py.push_back(atof(argv[i+1]));
		z.push_back(atof(argv[i+2]));
		R.push_back(Rval);
	}

	size_t N = R.size();

	RSN::line_intersection(N, px, py, z, &t[0], &t[1], c);
	RSN::BOT::IWLS_2D(N, px, py, z, R, t, c, true);
	printf("iwls: %0.3f, %0.3f [%0.3f, %0.3f, %0.3f, %0.3f]\n",t[0],t[1],c[0],c[1],c[2],c[3]);

	MatrixXd x0(2,1);
	MatrixXd x1(2,1);
	MatrixXd p0(2,2);
	MatrixXd p1(2,2);
	for (int i=0;i<2;i++){
		x0(i) = tp[i];
		x1(i) = t[i];
		for (int j=0;j<2;j++){
			p0(i,j) = cp[i*2+j];
			p1(i,j) = c[i*2+j];
		}
	}
	cout<<x0<<endl<<endl;
	cout<<x1<<endl<<endl;
	cout<<p0<<endl<<endl;
	cout<<p1<<endl<<endl;

	cout<< (p0.inverse()+p1.inverse()).inverse() * (p0.inverse()*x0 + p1.inverse()*x1) <<endl;
	//printf("iwls+prior: %0.3f, %0.3f [%0.3f, %0.3f, %0.3f, %0.3f]\n",t[0],t[1],cov[0],cov[1],cov[2],cov[3]);

	return 0;
}

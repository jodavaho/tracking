

#include <tagloc/tracking.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


#define _USE_MATH_DEFINES
using std::cout;
using std::endl;
using Eigen::MatrixXd;

#define USE_MATH_DEFINES

int main(int argc, char** argv){
	if (argc==1){
		printf(" <prior: x y> <prior cov: (2x2)> [measurement x y z r] \n");
		printf(" need 2 or more measurements! \n");
	}
	FILE* ofile = fopen("tag_loc","w");
	if (ofile==NULL)
		perror("File Open");

	std::vector<double> px,py,z,R;
	double t[2],c[4];
	double tp[2],cp[4];

	int i=1;
	{
		char* cptr = argv[i];
		double fval = strtod(argv[i],&cptr);
		while (fval == 0.0 && cptr==argv[i]){
			printf("Ignoring: %s\n",argv[i]);
			i++;
			fval = strtod(argv[i],&cptr);
		}
	}
	tp[0] = atof(argv[i++]);	
	tp[1] = atof(argv[i++]);	
	cp[0] = atof(argv[i++]);	
	cp[1] = atof(argv[i++]);	
	cp[2] = atof(argv[i++]);	
	cp[3] = atof(argv[i++]);

	for(;i<argc-1;){
		char* cptr = argv[i];
		double fval = strtod(argv[i],&cptr);
		while (fval == 0.0 && cptr==argv[i]){
			printf("Ignoring: %s\n",argv[i]);
			i++;
			fval = strtod(argv[i],&cptr);
		}
		px.push_back(fval);
		py.push_back(atof(argv[i+1]));
		z.push_back(atof(argv[i+2]));
		R.push_back(atof(argv[i+3]));
		i = i+4;
		//R.push_back(pow(M_PI/6,2));
	}

	int N = (int) R.size();

	printf("prior: %0.3f, %0.3f [%0.3f, %0.3f, %0.3f, %0.3f]\n",tp[0],tp[1],cp[0],cp[1],cp[2],cp[3]);

	//fprintf(ofile,"%d %0.3f %0.3f %0.3f ",0,0,0,0); //first measurement loc
	//fprintf(ofile,"%0.3f %0.3f ", tp[0], tp[1]);
	//fprintf(ofile,"%0.3f %0.3f %0.3f %0.3f %0.3f\n",
			//cp[0],cp[1],cp[2],cp[3]);


	for (i=0;i<N;i++){
		printf(" Z %d: (%f,%f) = %f +/- %f \n",i,px[i],py[i],z[i],sqrt(R[i]));
	}
	
	RSN::line_intersection(N, px, py, z, t[0], t[1]);
	printf("LI: %0.3f, %0.3f\n",t[0],t[1]);

	printf("==========\n");
	for (i=1;i<N;i++){
		printf(" Up to %d",i);
		RSN::line_intersection(i+1, px, py, z, t[0], t[1]);
		RSN::BOT::IWLS_2D(i+1, px, py, z, R, t, c, true);
		printf(" gives: %0.3f, %0.3f [%0.3f, %0.3f, %0.3f, %0.3f]\n",t[0],t[1],c[0],c[1],c[2],c[3]);

		//fprintf(ofile,"%d %0.3f %0.3f %0.3f ",i+1,0,0,0); //first measurement loc
		//fprintf(ofile,"%0.3f %0.3f ", tp[0], tp[1]);
		//fprintf(ofile,"%0.3f %0.3f %0.3f %0.3f %0.3f\n",
				//cp[0],cp[1],cp[2],cp[3]);
	}

	RSN::line_intersection(N, px, py, z, t[0], t[1]);
	RSN::BOT::IWLS_2D(N, px, py, z, R, t, c, true);

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

	MatrixXd S = (p0+p1);
	MatrixXd y = x1 - x0;
	MatrixXd tf = x0 + p0*S.inverse()*y;
	MatrixXd Pf = (MatrixXd::Identity(2,2) - p0*S.inverse())*p0;
	printf("final+prior: %0.3f, %0.3f [%0.3f, %0.3f, %0.3f, %0.3f]\n",tf(0),tf(1),Pf(0,0),Pf(0,1),Pf(1,0),Pf(1,1));

	fclose(ofile);
	return 0;
}

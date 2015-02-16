

#include <tagloc/tracking.h>
#include <math.h>
#include <stdio.h>
#define USE_MATH_DEFINES
int main(int argc, char** argv){
	double s[3][2];
	s[0][0]=0;
	s[0][1]=0;
	s[1][0]=0;
	s[1][1]=10;
	s[2][0]=12;
	s[2][1]=10;
	double z[3];
	z[0]=0;
	z[1]=-M_PI/4;
	z[2]=-M_PI/2;
	double r[3];
	r[0] = .1;
	r[1] = .1;
	r[2] = .1;
	double t[2];
	t[0] = 10; t[1]=0;
	double cov[4];
	cov[0] = 10;
	cov[1] = 0;
	cov[2] = 0;
	cov[3] = 10;

	for (int i=0; i< 3;i++)
	{
		printf("prior: %0.3f, %0.3f [%0.3f, %0.3f, %0.3f, %0.3f]\n",t[0],t[1],cov[0],cov[1],cov[2],cov[3]);
		RSN::BOT::IEKF_2D(s[i], z[i], r[i], t ,cov);
		printf("post: %0.3f, %0.3f [%0.3f, %0.3f, %0.3f, %0.3f]\n",t[0],t[1],cov[0],cov[1],cov[2],cov[3]);
	}
	return 0;
}

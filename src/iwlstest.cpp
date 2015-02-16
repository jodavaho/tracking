

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
	s[2][0]=10;
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

	printf("----------------------------------------------_TEST1\n");
printf("prior: %0.3f, %0.3f [%0.3f, %0.3f, %0.3f, %0.3f]\n",t[0],t[1],cov[0],cov[1],cov[2],cov[3]);
RSN::BOT::IWLS_2D(3, s, z, r, t ,cov);
printf("post: %0.3f, %0.3f [%0.3f, %0.3f, %0.3f, %0.3f]\n",t[0],t[1],cov[0],cov[1],cov[2],cov[3]);

s[0][0]=0;
s[0][1]=0;
s[1][0]=0;
s[1][1]=10;
s[2][0]=10;
s[2][1]=10;

z[0]=0;
z[1]=-M_PI/4;
z[2]=-M_PI/2;

r[0] = .1;
r[1] = .1;
r[2] = .1;

t[0] = 10; t[1]=0;

cov[0] = 10;
cov[1] = 0;
cov[2] = 0;
cov[3] = 10;


	printf("----------------------------------------------_TEST2\n");
RSN::line_intersection(3, s, z, t[0],t[1]);
printf("prior: %0.3f, %0.3f [%0.3f, %0.3f, %0.3f, %0.3f]\n",t[0],t[1],cov[0],cov[1],cov[2],cov[3]);
RSN::BOT::IWLS_2D(3, s, z, r, t ,cov);
printf("post: %0.3f, %0.3f [%0.3f, %0.3f, %0.3f, %0.3f]\n",t[0],t[1],cov[0],cov[1],cov[2],cov[3]);

s[0][0]=0;
s[0][1]=0;
s[1][0]=0;
s[1][1]=10;
s[2][0]=10;
s[2][1]=12;

z[0]=M_PI/4;
z[1]=3*M_PI/4;
z[2]=M_PI/4;

r[0] = .1;
r[1] = .1;
r[2] = .1;

t[0] = 10; t[1]=0;

cov[0] = 10;
cov[1] = 0;
cov[2] = 0;
cov[3] = 10;

	printf("----------------------------------------------_TEST3\n");
RSN::line_intersection(3, s, z, t[0],t[1]);
printf("prior: %0.3f, %0.3f [%0.3f, %0.3f, %0.3f, %0.3f]\n",t[0],t[1],cov[0],cov[1],cov[2],cov[3]);
RSN::BOT::IWLS_2D(3, s, z, r, t ,cov,false);
printf("post(not ambig): %0.3f, %0.3f [%0.3f, %0.3f, %0.3f, %0.3f]\n",t[0],t[1],cov[0],cov[1],cov[2],cov[3]);


	printf("----------------------------------------------_TEST4=_TEST3+ambig\n");
RSN::line_intersection(3, s, z, t[0],t[1]);
printf("prior: %0.3f, %0.3f [%0.3f, %0.3f, %0.3f, %0.3f]\n",t[0],t[1],cov[0],cov[1],cov[2],cov[3]);
RSN::BOT::IWLS_2D(3, s, z, r, t ,cov,true);
printf("post(ambig): %0.3f, %0.3f [%0.3f, %0.3f, %0.3f, %0.3f]\n",t[0],t[1],cov[0],cov[1],cov[2],cov[3]);

	s[0][0]=0;
	s[0][1]=0;
	s[1][0]=0;
	s[1][1]=10;
	s[2][0]=0;
	s[2][1]=-10;
	
	z[0]=0;
	z[1]=-M_PI/4;
	z[2]=M_PI/4;
	
	r[0] = .1;
	r[1] = .1;
	r[2] = .1;
	
	t[0] = 5; t[1]=0;
	
	cov[0] = 10;
	cov[1] = 0;
	cov[2] = 0;
	cov[3] = 10;

	printf("----------------------------------------------_TEST5\n");
  //RSN::line_intersection(3, s, z, t[0],t[1]);
	printf("prior: %0.3f, %0.3f [%0.3f, %0.3f, %0.3f, %0.3f]\n",t[0],t[1],cov[0],cov[1],cov[2],cov[3]);
	RSN::BOT::IWLS_2D(3, s, z, r, t ,cov,false);
	printf("post(not ambig): %0.3f, %0.3f [%0.3f, %0.3f, %0.3f, %0.3f]\n",t[0],t[1],cov[0],cov[1],cov[2],cov[3]);

	return 0;
}

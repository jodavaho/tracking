#include <tagloc/tracking.h>
#include <math.h>
#include <stdio.h>
#include <iostream>
#define USE_MATH_DEFINES
int main(){
	double s[4][2];
	s[0][0] = 0;
	s[0][1] = 0;
	s[1][0] = 1;
	s[1][1] = 1;
	s[2][0] = 3;
	s[2][1] = 0;
	s[3][0] = 1;
	s[3][1] = -4;

	double th[4];
	th[0]=0;
	th[1]=-M_PI/2;
	th[2]=0;
	th[3]=-M_PI/2;

	double px,py;
	RSN::line_intersection(4,s,th,px,py);
	std::cout<<"Expect: 1,0. Got: "<<px<<" "<<py<<std::endl;

	for (int i=0; i<4;i++){
		th[i]=-th[i];
	}
	RSN::line_intersection(4,s,th,px,py);
	std::cout<<"Expect: 1,0. Got: "<<px<<" "<<py<<std::endl;

	for (int i=0; i<4;i++){
		th[i]=-th[i];
	}
	th[2]=.1;
	s[3][1]=.1;
	RSN::line_intersection(4,s,th,px,py);
	std::cout<<"Expect: 1.xx,0.xx (added noise). Got: "<<px<<" "<<py<<std::endl;
return 0;
}

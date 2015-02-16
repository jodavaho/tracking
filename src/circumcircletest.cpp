#include <tagloc/tracking.h>
#include <stdio.h>
using namespace RSN;

int main(){
	printf("\n\tExpect c=1.5 1.5, r= root 2 or something\n");
	double px[] = {1, 1, 2, 2, 1};
	double py[] = {1, 2, 2, 1, 1};
	double cx,cy;
	double radius;
	RSN::circumcircle(5, px,py,cx,cy,radius);
	printf("\t[%f,%f] , [%f]\n",cx,cy,radius);
	double px2[] = {1, 1, 2.1, 2};
	double py2[] = {1, 2, 2, 1};
	printf("\n\tExpect something similar to above (added noise)\n");
	RSN::circumcircle(4, px2,py2,cx,cy,radius);
	printf("\t[%f,%f] , [%f]\n",cx,cy,radius);
	printf("\n\tcollinear case: Expect 0,0.5 \n");
	double px3[] = {0, 0};
	double py3[] = {0, 1};
	circumcircle(2, px3,py3,cx,cy,radius);
	printf("\t[%f,%f] , [%f]\n",cx,cy,radius);
}

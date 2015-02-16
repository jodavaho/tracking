#include <tagloc/algorithms.h>
#include <stdio.h>
#include <iostream>

using namespace std;
int main(int argc, char** argv){
	Eigen::MatrixXd r1(2,1),r2(2,1),t(2,1),P(2,2);
	r1<<-20,0;
	r2<<-20,0;
	t<<-30,-120;
	P<<900,0,0,900;

//	t<<0,0;

//	P<<4,0,0,4;

//	r1<<1,10;
//	r2<<-1,10;
//	RSN::onestep(r1,r2,t,P,1,.1);
//	cout<<"Expect symmetric about y"<<endl;
//	cout<<r1<<endl;
//	cout<<r2<<endl;

//	r1<<10,1;
//	r2<<10,-1;
	RSN::onestep_circle(r1,r2,t,30,.35,50,1);
	cout<<r1.transpose()<<endl;
	cout<<r2.transpose()<<endl;
	cout<<"dist:"<<(r1-r2).norm()<<endl;
	RSN::onestep_circle(r1,r2,t,30,.35,50,1);
	cout<<r1.transpose()<<endl;
	cout<<r2.transpose()<<endl;
	cout<<"dist:"<<(r1-r2).norm()<<endl;
	
//	cout<<"Expect same, but on X"<<endl;

//	r1<<10,0;
//	r2<<10,0;
//	RSN::onestep(r1,r2,t,P,1,.1);
//	cout<<"Expect same as last"<<endl;
//	cout<<r1<<endl;
//	cout<<r2<<endl;
	return EXIT_SUCCESS;
}

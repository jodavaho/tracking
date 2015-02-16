#include <tagloc/tracking.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

using namespace std;
using namespace RSN;
namespace po = boost::program_options;
#define DEF_R .1

int main(int argc, char** argv){

	double R;
	po::options_description desc("estimator options");
	desc.add_options()
		("help,h","produce help message")
		("iwls,w","(Default) Use Iterated Weighted Least Squares estimator. The line-intersection of all bearings is used as an initial estimate")
		("ekf,e","Use Extended Kalman Filter. The first two measurements are used to form the initial estimate")
		("iekf,i","Use Iterated Extended Kalman Filter. The first two measurements are used to form the initial estimate, and the update equations are re-linearized and iterated until convergence")
		("grad,g","Use gradient ascent. The line intersection is the initial estimate")
		("line,l","Use simple intersection of lines. R is ignored")
		("ambig,a","use ambiguous measurements")
		("degrees,d","measurements given in degrees")
		("noise,R", po::value<double>()->default_value(DEF_R), "set sensor noise in radians")
		("sequence,s",po::value<vector<double> > ()->multitoken(), "A full sequence of the form x y z x2 y2 z2 ... xn yn zn")
		;

	try{
		po::variables_map vm;
		po::positional_options_description p;
		p.add("sequence",-1);
		//po::store(po::parse_command_line(argc, argv,desc),vm);
		po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(),vm);
		po::notify(vm);    

		if (vm.count("help") || argc == 1) {
			throw -1;
		}

		R=vm["noise"].as<double>();
		vector<double> sequence = vm["sequence"].as<vector<double> >();
		if (((int)sequence.size()) % 3 !=0 ){
			cerr<<"Incorrect number of args: "<<sequence.size()<<endl;
			throw -2;
		}
		bool degrees = (bool) vm.count("degrees");
		vector<double> px;
		vector<double> py;
		vector<double> r;
		vector<double> z;
		double t[2];
		double c[4];
		for (unsigned int i=0;i<sequence.size();){
			px.push_back(sequence[i++]);
			py.push_back(sequence[i++]);
			if (degrees){
				z.push_back((sequence[i++]/180.0)*M_PI);
			} else {
				z.push_back(sequence[i++]);
			}
			r.push_back(R);
		}
		int n = sequence.size()/3;

		bool ambig = (bool) vm.count("ambig");
		bool done=false;
		if (vm.count("grad") ) {
			RSN::line_intersection(n,px,py,z,t[0],t[1],c);
			RSN::BOT::Ml_Grad_Asc_2D(n,px,py,z,r,t,c,ambig);
			cout<<"grad: (";
			cout<<t[0]<<","<<t[1]<<")";
			cout<<" (";
			cout<<c[0]<<",";
			cout<<c[1]<<",";
			cout<<c[2]<<",";
			cout<<c[3]<<")";
			cout<<endl;
			done = true;
		}
		if (vm.count("line") ) {
			RSN::line_intersection(n,px,py,z,t[0],t[1],c);
			cout<<"line: (";
			cout<<t[0]<<","<<t[1]<<")";
			cout<<" (";
			cout<<c[0]<<",";
			cout<<c[1]<<",";
			cout<<c[2]<<",";
			cout<<c[3]<<")";
			cout<<endl;
			done = true;
		}
		if (vm.count("ekf") ) {
			vector<double> pxp(2);
			vector<double> pyp(2);
			for (int i=0;i<2;i++){
				pxp[i] = px[i];
				pyp[i] = py[i];
			}
			RSN::line_intersection(n,pxp,pyp,z,t[0],t[1],c);
			for (int i=2;i<n;i++){
				RSN::BOT::EKF_UP_2D(px[i],py[i],z[i],r[i],t,c,ambig);
			}
			cout<<"ekf: (";
			cout<<t[0]<<","<<t[1]<<")";
			cout<<" (";
			cout<<c[0]<<",";
			cout<<c[1]<<",";
			cout<<c[2]<<",";
			cout<<c[3]<<")";
			cout<<endl;
			done = true;
		}
		if (vm.count("iekf") ) {
			vector<double> pxp(2);
			vector<double> pyp(2);
			for (int i=0;i<2;i++){
				pxp[i] = px[i];
				pyp[i] = py[i];
			}
			RSN::line_intersection(n,pxp,pyp,z,t[0],t[1],c);
			for (int i=2;i<n;i++){
				RSN::BOT::IEKF_2D(px[i],py[i],z[i],r[i],t,c,ambig);
			}
			cout<<"iekf: (";
			cout<<t[0]<<","<<t[1]<<")";
			cout<<" (";
			cout<<c[0]<<",";
			cout<<c[1]<<",";
			cout<<c[2]<<",";
			cout<<c[3]<<")";
			cout<<endl;
			done = true;
		}
		if (vm.count("iwls") || !done) {
			RSN::line_intersection(n,px,py,z,t[0],t[1],c);
			RSN::BOT::IWLS_2D(n,px,py,z,r,t,c,ambig);
			cout<<"iwls: (";
			cout<<t[0]<<","<<t[1]<<")";
			cout<<" (";
			cout<<c[0]<<",";
			cout<<c[1]<<",";
			cout<<c[2]<<",";
			cout<<c[3]<<")";
			cout<<endl;
		}
	} catch (int err){
		switch(err){
			case -2:{cerr<<"Bad arg count"<<endl;}
		}
		cerr<<desc<<endl;
		return EXIT_FAILURE;
	} catch (...){
		cerr<<"Error from boost?"<<endl;
		cerr<<desc<<endl;
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

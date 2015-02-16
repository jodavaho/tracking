#include <tagloc/tagloc.h>

int main(int argc, char** argv){
	printf("Tagloc Client...\n");
	tagloc::parseArgs(argc,argv);
	if (!ros::isInitialized()){
		ros::init(argc,argv,"tagloc_client");
	}

	ros::NodeHandle nh("tagloc_client");
	if (argc<2){
		printf("enter command!\n get <f>\n set <f>\n start <f>\n stop\n test <f> <Z> <x,y>\n step <f>\n init<f>\n");
		return -1;
	}else if (strcmp(argv[1],"get")==0 && argc>2){
		int f = atoi(argv[2]);
		tagloc::printH(nh,f);
	} else if (strcmp(argv[1],"set")==0 && argc>2){
		int f = atoi(argv[2]);

		printf("setting: %d---\n\n",f);
		if (argc!=9){
			printf("need: \"set <f> <x> <y> <cxx> <cxy> <cyx> <cyy>\"");
		} else {
			float p[2];
			p[0] = atof(argv[3]);
			p[1] = atof(argv[4]);
			float c[4];
			c[0] = atof(argv[5]);
			c[1] = atof(argv[6]);
			c[2] = atof(argv[7]);
			c[3] = atof(argv[8]);
			tagloc::addH(nh,f,p,c);
		}

	} else if (strcmp(argv[1],"start")==0 && argc>2){
		std::vector<int> todo;
		for (int i=2;i<argc;i++){
			todo.push_back(atoi(argv[i]));
		}
		tagloc::start(nh,todo);
	} else if (strcmp(argv[1],"stop")==0){
		tagloc::stop(nh);
	} else if (strcmp(argv[1],"test")==0 && argc>3){
		int f = atoi(argv[2]);
		double zz = atof(argv[3]);
		double rth = 0;
		if (argc>4){
			rth = atof(argv[4]);
		}
		tagloc::testZ(nh,f,zz,rth);
	} else if (strcmp(argv[1],"step")==0 && argc>2){
		int f = atoi(argv[2]);
		tagloc::step(nh,f);
	} else if (strcmp(argv[1],"measure")==0 && argc>2){
		int f = atoi(argv[2]);
		tagloc::measure(nh,f);
	} else if (strcmp(argv[1],"init")==0 && argc>2){
		int f = atoi(argv[2]);
		tagloc::init(nh,f);
	} else if (strcmp(argv[1],"check")==0 && argc>2){
		int f = atoi(argv[2]);
		tagloc::check(nh,f);
	} else if (strcmp(argv[1],"wait")==0){
		tagloc::wait(nh);
	}
	return 0;
}

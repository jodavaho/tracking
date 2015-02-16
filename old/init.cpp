#include <tagloc/tagloc.h>
#include <tagloc/helpers.h>
#include <dcc/History.h>
#include <husky/HuskyState.h>
#include <tagloc/Measurement.h>
#include <tagloc/UpdateStep.h>
#include <husky/Goto.h>
#include <geometry_msgs/PoseArray.h>
#include <tagloc/Init.h>
#include <paramon/paramon.h>
#include <robostix/MotorCommands.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8.h>
#include <dcc/FreqChange.h>

bool singleSearchInit(tagloc::Init::Request& req,tagloc::Init::Response &res);
bool singleHybridInit(tagloc::Init::Request& req,tagloc::Init::Response &res);
void doLineSearch(std::vector<int> freqs, double direction, int sign,
		std::vector<std::pair<double,double> >& startLocations, std::vector<std::pair<double,double> >& endLocations,std::vector<int>& foundFreqs);
double checkBoundary(double testx, double testy);
bool hybridinit(tagloc::Init::Request& req, tagloc::Init::Response &res);
void loadparams(ros::NodeHandle nh);
bool setPan(double angle);
void addwatches(ros::NodeHandle nh);
void setFrequency(std::vector<int> freq);
void sampleThese(std::vector<int>& freqs,int counts, bool setFreq);

// Globals

using namespace std;

bool ok;
bool storeHistory;
double step_size;
dcc::History history;
std::map<int, int> nz;
std::map<int, int> cnt;
double minDistToShore;
double homeLat,homeLon;
int min_pings,initStyle,max_steps;
husky::HuskyState lastState;
ros::Publisher panPublisher;

void historyIn(const dcc::History::ConstPtr& msg){
	if (storeHistory){
		history=*msg;
		for (int i =0;i<(int)history.frequencies.size();i++){
			int cf = history.frequencies[i];
			int cnz = history.nonzeros[i];
			int ccnt = history.counts[i];
			nz[cf]=cnz;
			cnt[cf]=ccnt;
		}
	}
}

/**
 * Some of my parameter got updated. Check and update local variable
 */
void paramonIn(const std_msgs::String::ConstPtr &msg){
	std::string my_ns = ros::this_node::getNamespace();
	if (msg->data.find(my_ns)!=std::string::npos){
		ROS_INFO("Heard one of my params change... %s",msg->data.c_str());
		ros::NodeHandle nh("init");
		loadparams(nh);
	}
}

void huskyIn(const husky::HuskyState::ConstPtr& msg){
	lastState = *msg;
}

/**
 * Checks to see if the given point is in the boundary
 * Actually the testing is done by boundary_checker package
 * This function makes a function call and returns a 
 * signed distance of test point to the shore
 * @param testx x coordinate of test point (in cartesian)
 * @param testy y coordinate of test point (in cartesian)
 * @return distToShore signed (minimum) dist. of test point to shore
 */
double checkBoundary(double testx, double testy) {
//double distToShore=1e10;
//boundary_checker::CheckBoundary::Request req;
//boundary_checker::CheckBoundary::Response res;

//req.home.position.x = homeLat;
//req.home.position.y = homeLon;

//geometry_msgs::Pose test;
//test.position.x = testx; 
//test.position.y = testy;
//req.list_to_check.poses.push_back(test);
//if(!ros::service::call("/boundary_checker/check_boundary_command",req,res)) {
//	ROS_WARN("Could not check the boundary. I'm assuming its safe.");
//} else {
//	distToShore = res.dist_to_points[0].data;
//	printf("Distance to shore: %f\n",distToShore);
//}
return 0.0;
//return distToShore;
}

/** I HATE writing methods that use global variables like this, 
 * but I don't see an easy alternative **/
void sampleThese(std::vector<int>& freqs,int counts, bool setFreq){

	if(setFreq) {
		setFrequency(freqs);
	}

	ROS_INFO("Sampling %Zu frequencies",freqs.size());
	std_srvs::Empty MT;
	ros::service::call("/dcc/clear_counts",MT.request,MT.response);
	nz.clear();
	cnt.clear();

	// Listen to min_pings at three pan directions
	map<int,int> prevCnts;
	std::vector<int>::iterator iter;
	for(iter=freqs.begin();iter!=freqs.end(); iter++) {
		prevCnts[(*iter)] = 0; //initialize all previous counts to 0
	}
	int T=3; //assert(T>1);
	for(int t=0; t<T; t++) {
		setPan(-90+t*180.0/(T-1));
		ros::Duration(180.0/(T-1)/30.0); //wait for some time for pan to catch up
		//To get fresh counts, see how many were heard previously
		for(iter=freqs.begin();iter!=freqs.end(); iter++) {
			prevCnts[(*iter)] = cnt[(*iter)];
		}
		storeHistory = true;
		bool notdone = true;
		while(notdone && ok){
			notdone=false;
			ros::Duration(0.1).sleep();
			ros::spinOnce();
			for (int i =0;i<(int)freqs.size();i++){
				//	ROS_INFO("%d has %d",freqs[i],cnt[freqs[i]]);
				if (cnt[freqs[i]]-prevCnts[freqs[i]]<(t+1)*counts){//if anyone notdone, keep trying
					notdone=true;
					break;
				}
			}
		}
		storeHistory=false;
	}
}
void moveChassis(double x, double y){
	husky::Goto::Request req;
	req.desired.x = x;
	req.desired.y = y;
	husky::Goto::Response res;
	ROS_INFO("Moving to: [%f,%f]",x,y);
	ros::service::call(HUSKY_GOTO_SERVICE,req,res);
	lastState.pose = res.achieved;
}

/**
 * Programs dcc to set corresponding frequency list
 */
void setFrequency(std::vector<int> freq) {

	if(freq.size()==0) {
		ROS_WARN("Tried to program an empty list.");
	}
	ROS_INFO("Request to change frequencies.");
	dcc::FreqChange fc;
	std::vector<int>::iterator iter;
	for(iter=freq.begin();iter!=freq.end();iter++) {
		fc.request.desired_list.push_back((*iter));//fill in the frequency
	}
	if(ros::service::call("/dcc/freq_change",fc.request,fc.response)) {
		ROS_INFO("Desired list programmed");
	} else {
		ROS_ERROR("Could not program frequency list. Will attempt to continue.");
	}
}

/**
 * Multi-target init (4x4) search strategy
 * Take first frequency, detect its circle
 * Go to the center of the circle
 * Do another 4 rounds with rest of frequencies
 */
bool multiSearch4x4Init(tagloc::Init::Request& req, tagloc::Init::Response &res){

	//Get the first circle first
	tagloc::Init::Request reqCopy = req; //Because singleSearchInit culls req
	ROS_INFO("Welcome to multi-target search (4x4) initialization");
	ROS_INFO("Starting single-target search first.\n");
	if(!singleSearchInit(reqCopy,res)) {
		ROS_ERROR("Problem initializing first target. Quitting.");
		return false;
	}

	//Goto center of first target's circle
	double startx = res.results[0].target_pose.x;
	double starty = res.results[0].target_pose.y;
	moveChassis(startx,starty);

	// Detect aggregation here
	reqCopy.freqs.clear(); //store aggregation here
	req.freqs.erase(req.freqs.begin(),req.freqs.begin()+1); //remove first element

	sampleThese(req.freqs,min_pings,true);
	for (int j=0;j<(int) req.freqs.size();j++){
		ROS_INFO("%d has %d non-zeros",req.freqs[j],nz[req.freqs[j]]);
		if (nz[req.freqs[j]]>0){
			reqCopy.freqs.push_back(req.freqs[j]);
		}
	}
	req.freqs = reqCopy.freqs; //copy back the aggregation found
	int numAggregate = req.freqs.size();

	//if only one fish in aggregation, do single strategy
	if(numAggregate == 1){
		return singleSearchInit(req,res);//this guy will push back the response
	} else if(numAggregate > 1){//search along four directions
		double direction = req.extras[0]; //Same dir as first round
		map<int,std::vector<double> > px;
		map<int,std::vector<double> > py;
		map<int,bool > prevLeg;
		std::vector<std::pair<double,double> > startLocations,endLocations;
		std::vector<int> foundFreqs;
		std::vector<int>::iterator iter;
		if (ok){
			doLineSearch(req.freqs,tagloc::normalizeAngle(direction+M_PI/4),1,startLocations,endLocations,foundFreqs);
			// Update all frequencies found
			for(iter=foundFreqs.begin();iter<foundFreqs.end();iter++) {
				px[*iter].push_back(endLocations.at(iter-foundFreqs.begin()).first);
				py[*iter].push_back(endLocations.at(iter-foundFreqs.begin()).second);
				px[*iter].push_back(startLocations.at(iter-foundFreqs.begin()).first);
				py[*iter].push_back(startLocations.at(iter-foundFreqs.begin()).second);
				prevLeg[*iter] = true;
			}
			moveChassis(startx,starty);
		}
		if (ok){
			doLineSearch(req.freqs,tagloc::normalizeAngle(direction+M_PI/4),-1,startLocations,endLocations,foundFreqs);
			// Update all frequencies found
			for(iter=foundFreqs.begin();iter<foundFreqs.end();iter++) {
				if(prevLeg.count(*iter)>0) {//something was found on previous leg, remove start point and update it
					px[*iter].pop_back();
					py[*iter].pop_back();
					px[*iter].push_back(endLocations.at(iter-foundFreqs.begin()).first);
					py[*iter].push_back(endLocations.at(iter-foundFreqs.begin()).second);
				} else { //just add the new start and end points
					px[*iter].push_back(endLocations.at(iter-foundFreqs.begin()).first);
					py[*iter].push_back(endLocations.at(iter-foundFreqs.begin()).second);
					px[*iter].push_back(startLocations.at(iter-foundFreqs.begin()).first);
					py[*iter].push_back(startLocations.at(iter-foundFreqs.begin()).second);
				}
			}
			moveChassis(startx,starty);
		}
		prevLeg.clear(); //start new leg
		if (ok){
			doLineSearch(req.freqs,tagloc::normalizeAngle(direction-M_PI/4),1,startLocations,endLocations,foundFreqs);
			// Update all frequencies found
			for(iter=foundFreqs.begin();iter<foundFreqs.end();iter++) {
				px[*iter].push_back(endLocations.at(iter-foundFreqs.begin()).first);
				py[*iter].push_back(endLocations.at(iter-foundFreqs.begin()).second);
				px[*iter].push_back(startLocations.at(iter-foundFreqs.begin()).first);
				py[*iter].push_back(startLocations.at(iter-foundFreqs.begin()).second);
				prevLeg[*iter] = true;
			}
			moveChassis(startx,starty);
		}
		if (ok){
			doLineSearch(req.freqs,tagloc::normalizeAngle(direction-M_PI/4),-1,startLocations,endLocations,foundFreqs);
			// Update all frequencies found
			for(iter=foundFreqs.begin();iter<foundFreqs.end();iter++) {
				if(prevLeg.count(*iter)>0) {//something was found on previous leg, remove start point and update it
					px[*iter].pop_back();
					py[*iter].pop_back();
					px[*iter].push_back(endLocations.at(iter-foundFreqs.begin()).first);
					py[*iter].push_back(endLocations.at(iter-foundFreqs.begin()).second);
				} else { //just add the new start and end points
					px[*iter].push_back(endLocations.at(iter-foundFreqs.begin()).first);
					py[*iter].push_back(endLocations.at(iter-foundFreqs.begin()).second);
					px[*iter].push_back(startLocations.at(iter-foundFreqs.begin()).first);
					py[*iter].push_back(startLocations.at(iter-foundFreqs.begin()).second);
				}
			}
		}

		if (ok){
			//for all found
			map<int,std::vector<double> >::iterator itx,ity;
			for(itx=px.begin(),ity=py.begin(); itx!=px.end() && ity!=py.end(); itx++,ity++) {
				double cx,cy,r;
				if((*itx).second.size()<3) {
					ROS_ERROR("Incorrect number of points for %d.",(*itx).first);
					continue;
				}
				double x[(*itx).second.size()],y[(*ity).second.size()];
				std::vector<double>::iterator itxx = (*itx).second.begin();
				std::vector<double>::iterator ityy = (*ity).second.begin();
				for(; itxx!=(*itx).second.end() && ityy!=(*ity).second.end(); itxx++,ityy++) {
					x[itxx-(*itx).second.begin()] = (*itxx);
					y[ityy-(*ity).second.begin()] = (*ityy);
				}
				tagloc::circumcircle(x,y,(*itx).second.size(),cx,cy,r);
				ROS_INFO("Got: [%f,%f],[%f] for %d",cx,cy,r,(*itx).first);
				tagloc::Tag ctag;
				ctag.frequency = (*itx).first;
				ctag.target_pose.x = cx;
				ctag.target_pose.y = cy;
				double sig = r*r/9;
				ctag.target_cov[0] = sig;
				ctag.target_cov[1] = 0;
				ctag.target_cov[2] = 0;
				ctag.target_cov[3] = sig;
				res.results.push_back(ctag);
			}
		}
	}
	return ok;
}

/**
 * Single target hybrid initialization
 * Performs search for first frequency in the request
 * While on the way back on each leg, takes a bearing measurement
 */
bool singleHybridInit(tagloc::Init::Request& req, tagloc::Init::Response &res){
	ok = true;
	ROS_INFO("Heard search request.est");
	if (req.freqs.size()>1){
		ROS_WARN("Can't handle more than one request.frequest.! Only doing the first");
		while(req.freqs.size()>1){
			req.freqs.pop_back();
		}
	} else if(req.freqs.size() < 1) {
		ROS_ERROR("You didn't pass a valid frequency!");
		return false;
	}
	double startx = lastState.pose.x;
	double starty = lastState.pose.y;
	double direction = req.extras[0];
	tagloc::UpdateStep zs[4];
	int goodmeasures=0;
	std::vector<std::pair<double,double> > startLocation,endLocation,cPoints;
	std::vector<int> foundFreqs;
	if (ok){
		std::pair<double,double> start = std::pair<double,double>(0,0);
		std::pair<double,double> end = std::pair<double,double>(0,0);
		doLineSearch(req.freqs,tagloc::normalizeAngle(direction+M_PI/4),1,startLocation,endLocation,foundFreqs);
		if(foundFreqs.size() > 0) { //if something was found
			start = startLocation.front();
			end = endLocation.front();
			double dx = startLocation.front().first+endLocation.front().first;
			double dy = startLocation.front().second+endLocation.front().second;
			dx = dx/2;
			dy = dy/2;
			moveChassis(dx,dy);
			tagloc::Measurement tlm;
			tlm.request.frequency = req.freqs[0];
			ros::service::call(TL_GET_MEASUREMENT,tlm.request,tlm.response);
			if (tlm.response.valid){
				zs[goodmeasures].request.frequency = req.freqs[0];
				zs[goodmeasures].request.fromX = lastState.pose.x;
				zs[goodmeasures].request.fromY = lastState.pose.y;
				zs[goodmeasures].request.fromTH = lastState.pose.theta;
				zs[goodmeasures].request.Zlocal = tlm.response.bearing_local;
				goodmeasures++;
			}
		}
		moveChassis(startx,starty);
		doLineSearch(req.freqs,tagloc::normalizeAngle(direction+M_PI/4),-1,startLocation,endLocation,foundFreqs);
		if(foundFreqs.size() > 0) {
			if(start!=end) {//we had some valid points on the first leg
				start = endLocation.front(); //update only the start of the line
			} else {
				start = startLocation.front();
				end = endLocation.front();
			}
			double dx = startLocation.front().first+endLocation.front().first;
			double dy = startLocation.front().second+endLocation.front().second;
			dx = dx/2;
			dy = dy/2;
			moveChassis(dx,dy);
			tagloc::Measurement tlm;
			tlm.request.frequency = req.freqs[0];
			ros::service::call(TL_GET_MEASUREMENT,tlm.request,tlm.response);
			if (tlm.response.valid){
				zs[goodmeasures].request.frequency = req.freqs[0];
				zs[goodmeasures].request.fromX = lastState.pose.x;
				zs[goodmeasures].request.fromY = lastState.pose.y;
				zs[goodmeasures].request.fromTH = lastState.pose.theta;
				zs[goodmeasures].request.Zlocal = tlm.response.bearing_local;
				goodmeasures++;
			}
		}
		moveChassis(startx,starty);
		if(start!=end){ //we got some valid points on either legs
			cPoints.push_back(start);
			cPoints.push_back(end);
		}
	}
	if (ok){
		std::pair<double,double> start = std::pair<double,double>(0,0);
		std::pair<double,double> end = std::pair<double,double>(0,0);
		doLineSearch(req.freqs,tagloc::normalizeAngle(direction-M_PI/4),1,startLocation,endLocation,foundFreqs);
		if(foundFreqs.size() > 0) { //if something was found
			start = startLocation.front();
			end = endLocation.front();
			double dx = startLocation.front().first+endLocation.front().first;
			double dy = startLocation.front().second+endLocation.front().second;
			dx = dx/2;
			dy = dy/2;
			moveChassis(dx,dy);
			tagloc::Measurement tlm;
			tlm.request.frequency = req.freqs[0];
			ros::service::call(TL_GET_MEASUREMENT,tlm.request,tlm.response);
			if (tlm.response.valid){
				zs[goodmeasures].request.frequency = req.freqs[0];
				zs[goodmeasures].request.fromX = lastState.pose.x;
				zs[goodmeasures].request.fromY = lastState.pose.y;
				zs[goodmeasures].request.fromTH = lastState.pose.theta;
				zs[goodmeasures].request.Zlocal = tlm.response.bearing_local;
				goodmeasures++;
			}
		}
		moveChassis(startx,starty);
		doLineSearch(req.freqs,tagloc::normalizeAngle(direction-M_PI/4),-1,startLocation,endLocation,foundFreqs);
		if(foundFreqs.size() > 0) {
			if(start!=end) {//we had some valid points on the first leg
				start = endLocation.front(); //update only the start of the line
			} else {
				start = startLocation.front();
				end = endLocation.front();
			}
			double dx = startLocation.front().first+endLocation.front().first;
			double dy = startLocation.front().second+endLocation.front().second;
			dx = dx/2;
			dy = dy/2;
			moveChassis(dx,dy);
			tagloc::Measurement tlm;
			tlm.request.frequency = req.freqs[0];
			ros::service::call(TL_GET_MEASUREMENT,tlm.request,tlm.response);
			if (tlm.response.valid){
				zs[goodmeasures].request.frequency = req.freqs[0];
				zs[goodmeasures].request.fromX = lastState.pose.x;
				zs[goodmeasures].request.fromY = lastState.pose.y;
				zs[goodmeasures].request.fromTH = lastState.pose.theta;
				zs[goodmeasures].request.Zlocal = tlm.response.bearing_local;
				goodmeasures++;
			}
		}
		if(start!=end){ //we got some valid points on either legs
			cPoints.push_back(start);
			cPoints.push_back(end);
		}
	}

	if (ok){

		//Check if you found valid endpoints
		if(cPoints.size() < 3) {
			ROS_ERROR("Incorrect number of end points found while searching for %d.",req.freqs[0]);
			return false;
		}
		double cx,cy,r;
		double px[cPoints.size()],py[cPoints.size()];
		std::vector<std::pair<double,double> >::iterator iter;
		for(iter=cPoints.begin(); iter!=cPoints.end(); iter++) {
			px[iter-cPoints.begin()] = (*iter).first;
			py[iter-cPoints.begin()] = (*iter).second;
		}
		tagloc::circumcircle(px,py,cPoints.size(),cx,cy,r);
		ROS_INFO("Got: [%f,%f],[%f]",cx,cy,r);
		tagloc::Tag ctag;
		ctag.frequency = req.freqs[0];
		ctag.target_pose.x = cx;
		ctag.target_pose.y = cy;
		double sig = r*r/9;
		ctag.target_cov[0] = sig;
		ctag.target_cov[1] = 0;
		ctag.target_cov[2] = 0;
		ctag.target_cov[3] = sig;

		// Add measurements to triangulation
		tagloc::AddTag atag;
		atag.request.toAdd = ctag;
		if(!ros::service::call(ADD_HYPOTHESIS,atag)){
			ROS_ERROR("Problem adding tag to triangulation");
		}

		for (int i=0;i<goodmeasures;i++){
			// Incorporate measurements into tagloc hypothesis
			if(!ros::service::call(TL_UPDATE_TARGET,zs[i])){
				ROS_ERROR("Problem updating tagloc hypothesis");
			}
			ROS_INFO("Measure %d done.",i);
		}
		res.results.push_back(ctag);
	}

	return ok;
}

bool batchinit(tagloc::Init::Request& req,tagloc::Init::Response &res){
	ok = true;
	ROS_INFO("Heard batch request");
	return true;
}

/**
 * Keeps going on the line till you don't hear on any frequency in the list
 * or maxsteps times, whichever comes first.
 * First samples at location where call was made.
 * @param startLocations vector of locations of first non-zero detection
 * @param endLocations vector of locations of first zero, after some non-zero
 * @param sign Line is aligned with direction*sign
 * @param foundFreqs vector of frequencies detected, index matched with
 * 			@param startLocations and @param endLocations
 */
void doLineSearch(std::vector<int> freqs, double direction, int sign,
		std::vector<std::pair<double,double> >& startLocations, std::vector<std::pair<double,double> >& endLocations, std::vector<int>& foundFreqs){

	double cc = cos(direction);
	double ss = sin(direction);
	int i = 0; //Start from the current location

	//First clear anything in final locations
	startLocations.clear();
	endLocations.clear();
	foundFreqs.clear();
	map<int,bool> marked;

	//Set frequencies
	setFrequency(freqs);

	double startx = lastState.pose.x;
	double starty = lastState.pose.y;
	ROS_INFO("Doing line search from [%f,%f] in direction [%f,%f] for at most [%d] steps, with [%Zu] frequencies",startx,starty,cc,ss,max_steps,freqs.size());
	bool notcomplete = true;
	while (ok && notcomplete && i<max_steps){
		notcomplete = false;
		double dist = i*step_size*sign;

		ROS_INFO("Testing waters at the boundary.");
		double destx = startx + cc*dist;
		double desty = starty + ss*dist;
		double boundaryDist = checkBoundary(destx,desty);
		if(boundaryDist < minDistToShore) {
			ROS_WARN("Too close to the shore. Exiting.");
			notcomplete = false; //it should already be
		} else {
			printf("Safe to move.");
			moveChassis(startx+cc*dist,starty+ss*dist);
			sampleThese(freqs,min_pings,false); //don't set frequencies again
			for (int j=0;j<(int) freqs.size();j++){
				ROS_INFO("%d has %d non-zeros",freqs[j],nz[freqs[j]]);
				if (nz[freqs[j]]>0){
					notcomplete=true;
					std::vector<int>::iterator iter = find(foundFreqs.begin(),foundFreqs.end(),freqs[j]);
					if(iter==foundFreqs.end() || foundFreqs.empty()) {//newly found
						ROS_INFO("Started hearing %d at [%f,%f]",freqs[j],lastState.pose.x,lastState.pose.y);
						foundFreqs.push_back(freqs[j]);
						marked[freqs[j]] = false;
						startLocations.push_back(std::pair<double,double>(lastState.pose.x,lastState.pose.y));
						endLocations.push_back(startLocations.back());
					} else {//advance the end location
						endLocations.at(iter-foundFreqs.begin()) = std::pair<double,double>(lastState.pose.x,lastState.pose.y);
					}
				} else { //if previously found, update last location
					std::vector<int>::iterator iter = find(foundFreqs.begin(),foundFreqs.end(),freqs[j]);
					if(iter!=foundFreqs.end()) {//previously found
						int index = iter-foundFreqs.begin();
						//Only update the end location the first time you don't hear after hearing
						if(marked[freqs[j]] == false) {
							ROS_INFO("Stopped hearing %d at [%f,%f]",freqs[j],lastState.pose.x,lastState.pose.y);
							endLocations.at(index) = std::pair<double,double>(lastState.pose.x,lastState.pose.y);
							marked[freqs[j]] = true;
						}
					}
				}
			}
			i=i+1;
		}
	}
	ROS_INFO("%d frequencies were found on this leg.",(int)foundFreqs.size());
	ROS_INFO("Line search done at [%f,%f]",lastState.pose.x,lastState.pose.y);
}

/**
 * Single point of callback for starting initialization
 * Will choose style depending on param
 */
bool initService(tagloc::Init::Request& req, tagloc::Init::Response &res){

	bool done=false;

	if(req.freqs.size() < 1) {
		ROS_ERROR("0 frequencies passed!");
		return false;
	}

	switch(initStyle) {

	case 0: //single (4) search strategy
		done=singleSearchInit(req,res);
		break;

	case 1: //multi target (4x4) search strategy
		done=multiSearch4x4Init(req,res);
		break;

	case 2: //multi target (4x5) search strategy
		//TODO
		break;

	case 3: //single (4) hybrid strategy
		done=singleHybridInit(req,res);
		break;

	case 4: //multi target (4x4) hybrid strategy
		//TODO
		break;

	case 5: //multi target (4x5) hybrid strategy
		//TODO
		break;

	default: //bad choice?
		ROS_ERROR("Incorrect choice for init/style.");
	}

	return done;
}

/**
 * Single target (4) search strategy
 */
bool singleSearchInit(tagloc::Init::Request& req,tagloc::Init::Response &res){
	ok = true;
	ROS_INFO("Heard search request");
	if (req.freqs.size()>1){
		ROS_WARN("Can't handle more than one freq! Only doing the first");
		while(req.freqs.size()>1){
			req.freqs.pop_back();
		}
	} else if(req.freqs.size() < 1) {
		ROS_ERROR("You didn't pass a valid frequency!");
		return false;
	}
	double startx = lastState.pose.x;
	double starty = lastState.pose.y;
	double direction = req.extras[0];
	std::vector<std::pair<double,double> > startLocation,endLocation;
	std::vector<std::pair<double,double> > cPoints;
	std::vector<int> foundFreqs;
	if (ok){
		doLineSearch(req.freqs,tagloc::normalizeAngle(direction+M_PI/4),1,startLocation,endLocation,foundFreqs);
		std::pair<double,double> start = std::pair<double,double>(0,0);
		std::pair<double,double> end = std::pair<double,double>(0,0);
		if(foundFreqs.size() > 0) {
			start = startLocation.front();
			end = endLocation.front();
		}
		moveChassis(startx,starty);
		doLineSearch(req.freqs,tagloc::normalizeAngle(direction+M_PI/4),-1,startLocation,endLocation,foundFreqs);
		if(foundFreqs.size() > 0) {
			if(start!=end) {//we had some valid points on the first leg
				start = endLocation.front(); //update only the start of the line
			} else {
				start = startLocation.front();
				end = endLocation.front();
			}
		}
		moveChassis(startx,starty);
		if(start!=end){ //we got some valid points on either legs
			cPoints.push_back(start);
			cPoints.push_back(end);
		}
	}
	if (ok){
		doLineSearch(req.freqs,tagloc::normalizeAngle(direction-M_PI/4),1,startLocation,endLocation,foundFreqs);
		std::pair<double,double> start = std::pair<double,double>(0,0);
		std::pair<double,double> end = std::pair<double,double>(0,0);
		if(foundFreqs.size() > 0) {
			start = startLocation.front();
			end = endLocation.front();
		}
		moveChassis(startx,starty);
		doLineSearch(req.freqs,tagloc::normalizeAngle(direction-M_PI/4),-1,startLocation,endLocation,foundFreqs);
		if(foundFreqs.size() > 0) {
			if(start!=end) {//we had some valid points on the first leg
				start = endLocation.front(); //update only the start of the line
			} else {
				start = startLocation.front();
				end = endLocation.front();
			}
		}
		moveChassis(startx,starty);
		if(start!=end){ //we got some valid points on either legs
			cPoints.push_back(start);
			cPoints.push_back(end);
		}
	}

	if (ok){
		//Check if you found valid endpoints
		if(cPoints.size() < 3) {
			ROS_ERROR("Incorrect number of end points found while searching for %d.",req.freqs[0]);
			return false;
		}
		double cx,cy,r;
		double px[cPoints.size()],py[cPoints.size()];
		std::vector<std::pair<double,double> >::iterator iter;
		for(iter=cPoints.begin(); iter!=cPoints.end(); iter++) {
			px[iter-cPoints.begin()] = (*iter).first;
			py[iter-cPoints.begin()] = (*iter).second;
		}
		tagloc::circumcircle(px,py,cPoints.size(),cx,cy,r);
		ROS_INFO("Got: [%f,%f],[%f]",cx,cy,r);
		tagloc::Tag ctag;
		ctag.frequency = req.freqs[0];
		ctag.target_pose.x = cx;
		ctag.target_pose.y = cy;
		double sig = r*r/9;
		ctag.target_cov[0] = sig;
		ctag.target_cov[1] = 0;
		ctag.target_cov[2] = 0;
		ctag.target_cov[3] = sig;
		res.results.push_back(ctag);
	}

	return ok;
}

/**
 * Publishes on the robostix pan topic to @param angle in local frame
 * @param angle between -128 to +127 in degrees
 */
bool setPan(double angle) {

	if(angle < -128) {
		angle = -128;
	} else if(angle > 127) {
		angle = 127;
	}
	std_msgs::Int8 cmd;
	cmd.data = (int)angle;
	panPublisher.publish(cmd);
	return true;
}

/**
 * Cancels initialization
 */
bool cancelinit(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
	ok = false;
	return true;
}

/**
 * Loads params
 */
void loadparams(ros::NodeHandle nh){
	if (nh.hasParam("min_pings")){ //minimum pings heard before declaring target detected
		nh.getParam("min_pings",min_pings);
	} else {
		min_pings = 3;
	}
	ROS_INFO("min_pings: %d",min_pings);

	if (nh.hasParam("min_dist_to_shore")){ //how close can you get to the shore?
		nh.getParam("min_dist_to_shore",minDistToShore);
	} else {
		minDistToShore = 20;
	}
	ROS_INFO("min_dist_to_shore: %f",minDistToShore);

	if (nh.hasParam("style")){ //choose which method to use for the initialization
		nh.getParam("style",initStyle);
	} else {
		ROS_WARN("/init/style not set. Using default.");
		initStyle = 0;
	}
	ROS_INFO("style: %d",initStyle);

	if (nh.hasParam("max_steps")){ //max steps is param now
		nh.getParam("max_steps",max_steps);
	} else {
		ROS_WARN("/init/max_steps not set. Using default.");
		max_steps = 3;
	}
	ROS_INFO("max_steps: %d",max_steps);

	if (nh.hasParam("step_size")){ //step size is param now
		nh.getParam("step_size",step_size);
	} else {
		ROS_WARN("/init/step_size not set. Using default.");
		step_size = 20;
	}
	ROS_INFO("step_size: %0.3f",step_size);

	// Read in home coordinates
	if (nh.hasParam("/husky/home_lat")){
		nh.getParam("/husky/home_lat",homeLat);
	} else {
		ROS_ERROR("/husky/home_lat not set.");
	}
	if (nh.hasParam("/husky/home_lon")){
		nh.getParam("/husky/home_lon",homeLon);
	} else {
		ROS_ERROR("/husky/home_lon not set.");
	}
	ROS_INFO("home gps: [%.6f,%.6f]",homeLat,homeLon);
}

/**
 * Adds watches to paramon to monitor my params
 */
void addwatches(ros::NodeHandle nh){
	std::string my_ns = "init";
	paramon::addWatch(my_ns+"/min_pings");
	paramon::addWatch(my_ns+"/min_dist_to_shore");
	paramon::addWatch(my_ns+"/style");
	paramon::addWatch(my_ns+"/max_steps");
	paramon::addWatch(my_ns+"/step_size");
	paramon::addWatch("husky/home_lat");
	paramon::addWatch("husky/home_lon");
	ROS_INFO("Monitoring my parameters!");
}

int main(int argc, char** argv){
	if (!ros::isInitialized()){ros::init(argc,argv,"init_server");}

	ros::NodeHandle nh("init");
	loadparams(nh);
	addwatches(nh);

	// Uses a single callback and a param to choose style
	ros::ServiceServer init = nh.advertiseService(START_INIT_SERVICE,&initService);
	ros::ServiceServer cancel = nh.advertiseService("cancel",&cancelinit);

	ros::Subscriber dcc = nh.subscribe("/dcc/history",10, &historyIn);
	ros::Subscriber currentState = nh.subscribe(HUSKY_STATE, 10, &huskyIn);
	ros::Subscriber paramonListener = nh.subscribe<std_msgs::String>("/paramon/changed",10,&paramonIn);
	panPublisher = nh.advertise<std_msgs::Int8>("/motor_cmd/pan",10);
	ok = true;

	ros::MultiThreadedSpinner spinner(4); // Use 4 threads
	spinner.spin(); // spin() will not return until the node has been shutdown
	ROS_INFO("Shutting down");

	return EXIT_SUCCESS;
}

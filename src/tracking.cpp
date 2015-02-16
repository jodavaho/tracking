#include <tagloc/tracking.h>
#include <assert.h>
#include <stdio.h>
#include <iostream>

namespace RSN{
	int _TL_MAX_ITERS = 1000;
	namespace BOT{
		static void EKF_UP_NOCOV(const double sensor_x, const double sensor_y, const double bearing_global_radians, const double sensor_sigma, double target[2], double target_i[2], double target_cov[4], const bool ambig=false);
	}
}

static double scond(Eigen::MatrixXd M){
	assert(M.rows()==M.cols());
	Eigen::JacobiSVD<Eigen::MatrixXd> _svdA;
	_svdA.compute(M);
	return _svdA.singularValues()(0)/_svdA.singularValues()(M.rows()-1);
}
//void RSN::IWLS_2D(const int n, const double sensors[][2], const double measurement_values[], const int measurement_types[], const double sensor_sigma[],double target[2], double target_cov[4], const bool ambig, double min_step, int max_iters){}

///HELPER HERE///
static std::vector<double> getVec(const int n, const double A[]){
	std::vector<double> tt(n);
	for (int i=0;i<n;i++){
		tt[i]=A[i];
	}
	return tt;
}
static std::vector<std::vector<double> > getVec(const int n, const double A[][2])
{
	std::vector<std::vector<double> >ss(n, std::vector<double>(2));
	for (int i=0;i<n;i++)
	{
		ss[i][0]=A[i][0];
		ss[i][1]=A[i][1];
	}
	return ss;
}

static void RSN::BOT::EKF_UP_NOCOV(const double sx, const double sy, const double bearing_global_radians, const double sensor_sigma, double target[2], double target_i[2], double target_cov[4], const bool ambig){

	Eigen::MatrixXd tp(2,1); tp<<target[0],target[1];
	Eigen::MatrixXd ti(2,1); ti<<target_i[0],target_i[1];
	Eigen::MatrixXd xr(2,1); xr<<sx,sy;
	Eigen::Matrix2d P = getCovMatrix(target_cov);
	Eigen::MatrixXd H = get_H(xr,ti);
	Eigen::MatrixXd S = get_S(xr,ti,P,sensor_sigma);

	double globalZ = RSN::normalizeAngle(bearing_global_radians);
	double zhypoth = atan2(ti(1)-xr(1),ti(0)-xr(0));
	double innov = RSN::normanglediff(zhypoth,globalZ);

	innov=RSN::normalizeAngle(innov);

	if (ambig && fabs(innov)>M_PI/2){
		innov = RSN::normalizeAngle(innov+M_PI);
	}

	//So 'S' is a scalar ...
	Eigen::MatrixXd K=P*H.transpose()*S.inverse();
	Eigen::MatrixXd xx = K*innov+K*H*(ti-tp);
	Eigen::MatrixXd tip = tp+xx;

	target_i[0] = tip(0);
	target_i[1] = tip(1);
	return;
}


void RSN::BOT::Ml_Grad_Asc_2D(const int n, const std::vector<double> sx, const std::vector<double> sy, const std::vector<double> bearing_global_radians, const std::vector<double> sensor_sigma,double target[2], double target_cov[4], const bool ambig, const int max_iters){

#ifdef __TL_VERBOSE 
	printf("Ml_Grad_Asc_2D:%d,%s\n",__LINE__,__FILE__);
	for (int i=0;i<n;i++){
		printf("s[%d] = %0.3f,%0.3f Z[%d]=%0.3f\n",i,sx[i],sy[i],i,bearing_global_radians[i]);
	}
#endif 
	double step = .01;
	double gradx = 0;
	double grady = 0;
	double zhp,normang;
	double dd,dx,dy;

	for (int i=0;i<max_iters;i++){
		gradx=0;grady=0; for (int j=0;j<n;j++){
			dy = target[1]-sy[j];
			dx = target[0]-sx[j];
			dd = dy*dy+dx*dx;
			zhp = atan2(dy,dx);
			normang = RSN::normanglediff(bearing_global_radians[j],zhp);
			if (ambig && fabs(normang)>M_PI/2)
			{
				normang = RSN::normalizeAngle(normang+M_PI);
			}
			gradx = gradx+normang*(-sin(zhp)) / (sqrt(dd)*sensor_sigma[j]);
			grady = grady+normang*(cos(zhp)) / (sqrt(dd)*sensor_sigma[j]);
		}
		//printf("%d) x: %0.3f,%0.3f dx: %0.3f %0.3f\n",i,target[0],target[1],gradx,grady);
		target[0] = target[0]+step*gradx;
		target[1] = target[1]+step*grady;
		if ((gradx*gradx+grady*grady)<.00001){break;}
	}
}
void RSN::BOT::Ml_Grad_Asc_2D(const int n, const std::vector<std::vector<double> >sensors, const std::vector<double> bearing_global_radians, const std::vector<double> sensor_sigma,double target[2], double target_cov[4], const bool ambig, const int max_iters){

	std::vector<double>sx,sy;
	for (int i=0;i<n;i++){
		sx.push_back(sensors[i][0]);
		sy.push_back(sensors[i][1]);
	}
	RSN::BOT::Ml_Grad_Asc_2D(n,sx,sy,bearing_global_radians,sensor_sigma,target,target_cov,ambig,max_iters);
}

void RSN::BOT::Ml_Grad_Asc_2D(const int n, const double sensors[][2], const double bearing_global_radians[], const double sensor_sigma[],double target[2], double target_cov[4], const bool ambig, const int max_iters ){
	std::vector<double>sx,sy;
	for (int i=0;i<n;i++){
		sx.push_back(sensors[i][0]);
		sy.push_back(sensors[i][1]);
	}
	std::vector<double>bs = getVec(n,bearing_global_radians);
	std::vector<double>rs = getVec(n,sensor_sigma);
	RSN::BOT::Ml_Grad_Asc_2D(n,sx,sy,bs,rs,target,target_cov,ambig,max_iters);
}

void RSN::BOT::IEKF_2D(const double sx, const double sy, const double bearing_global_radians, const double sensor_sigma,double target[2], double target_cov[4], const bool ambig, double min_step, int max_iters){

#ifdef __TL_VERBOSE
	printf("IEKF_2D:%d,%s\n",__LINE__,__FILE__);
#endif 
	double ti[2],tp[2], ddx;
	ddx = 9999;
	tp[0]=target[0];
	tp[1]=target[1];
	ti[0]=target[0];
	ti[1]=target[1];
	for (int i=0;i<max_iters-1 && sqrt(ddx)>min_step; i++){
		EKF_UP_NOCOV(sx,sy, bearing_global_radians, sensor_sigma, tp, ti, target_cov, ambig);
		ddx = ti[0]*target[0]+ti[1]+target[1];
	}
	target[0]=ti[0];
	target[1]=ti[1];

	EKF_UP_2D(sx,sy,bearing_global_radians, sensor_sigma, target, target_cov, ambig);
}
void RSN::BOT::IEKF_2D(const double sensor[2], const double bearing_global_radians, const double sensor_sigma,double target[2], double target_cov[4], const bool ambig, double min_step, int max_iters)
{

}


//uninspiring:
bool RSN::in_ellipse(const Eigen::MatrixXd c, const Eigen::MatrixXd P, const Eigen::MatrixXd q, const double s){
	Eigen::MatrixXd cp = closest_pt_ellipse(c,P,q,s);
	return (cp-c).norm()>(cp-q).norm();
}
//uninspiring:
double RSN::distance_to_ellipse(const Eigen::MatrixXd c, const Eigen::MatrixXd P, const Eigen::MatrixXd q, const double s){
	Eigen::MatrixXd cp = closest_pt_ellipse(c,P,q,s);
	return (cp-c).norm();
}
Eigen::MatrixXd RSN::closest_pt_ellipse(const Eigen::MatrixXd center_pt, const Eigen::MatrixXd Cov_Matrix, const Eigen::MatrixXd query_pt, const double scalar){
	int N = 10000;
	Eigen::SVD<Eigen::MatrixXd> svd(Cov_Matrix);

	//Transform to local
	Eigen::MatrixXd pts = Eigen::MatrixXd::Zero(2,N);
	Eigen::MatrixXd distances = Eigen::MatrixXd::Ones(1,N)*999999.9;
	Eigen::MatrixXd thetas = Eigen::MatrixXd::Zero(1,N);
	Eigen::MatrixXd pell = Eigen::MatrixXd::Zero(2,1);

	double th=0.0;
	int min_N=0;
	double cth,sth;
	double a,b;
	Eigen::MatrixXd sv = svd.singularValues();
	Eigen::MatrixXd rot = svd.matrixU();
	double det = rot.determinant();
	a = sqrt(sv(0));
	b = sqrt(sv(1));
	for (int i=0;i<N;i++){
		th+=(2*M_PI)/N;

		cth = cos(th);
		sth = sin(th);
		pell(0) = a*cth;
		pell(1) = b*sth;

		thetas(i) = th;
		distances(i) = ((rot*pell*scalar*det+center_pt) - query_pt).norm();
		if (distances(i)<distances(min_N)){
			min_N=i;
		}
	}

	Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(2,1);
	ret(0)=cos(thetas(min_N))*a;
	ret(1)=sin(thetas(min_N))*b;
	ret = rot*(ret*scalar*det) + center_pt;
	return ret;
}
void RSN::BOT::IWLS_2D(const int n, const std::vector<double>sx, const std::vector<double> sy, const std::vector<double> bearing_global_radians, const std::vector<double> sensor_sigma,double target[2], double target_cov[4], const bool ambig, double min_step, int max_iters){


#ifdef __TL_VERBOSE
	printf("IWLS_2D:%d,%s\n",__LINE__,__FILE__);
#endif
	double ddx = 999999.9;
	Eigen::MatrixXd tp(2,1); tp<<target[0],target[1];
	Eigen::MatrixXd _sensors(2,n);
	Eigen::MatrixXd _R(n,1);
	Eigen::MatrixXd _z(n,1); 
	Eigen::MatrixXd _y(n,1);
	Eigen::MatrixXd _H(n,2); 
	Eigen::MatrixXd _HRi(n,2); 
	Eigen::MatrixXd _A=Eigen::MatrixXd::Zero(2,2);
	Eigen::MatrixXd _nZ=Eigen::MatrixXd::Zero(2,1);
	
	//set up storage
	for (int i=0;i<n;i++){
		Eigen::MatrixXd xr(2,1); xr<<sx[i],sy[i];

 		_R(i)=sensor_sigma[i];
		_sensors.col(i) = xr;
		_z(i)=bearing_global_radians[i];

		double expz = atan2(target[1]-sy[i],target[0]-sx[i]);
		_y(i) = normanglediff(expz,_z(i));
		if (ambig){
			if (fabs(_y(i))>M_PI/2){
				
#ifdef __TL_VERBOSE
				printf("flipping: %d\n",i);
#endif
				_z(i)=RSN::normalizeAngle(bearing_global_radians[i]+M_PI);
				_y(i) = normanglediff(expz,_z(i));
			}
		}
	}
#ifdef __TL_VERBOSE
	std::cout<<"tp:"<<std::endl;
	std::cout<<tp<<std::endl;	
	std::cout<<"sensors:"<<std::endl;
	std::cout<<_sensors<<std::endl;	
	std::cout<<"R:"<<std::endl;
	std::cout<<_R<<std::endl;	
	std::cout<<"z:"<<std::endl;
	std::cout<<_z<<std::endl;	
	std::cout<<"y:"<<std::endl;
	std::cout<<_y<<std::endl;	
#endif
	for (int tt = 0;tt<max_iters && ddx > min_step;tt++)
	{
	
		//clear norm and FIM
		_nZ=Eigen::MatrixXd::Zero(2,1);
		_A=Eigen::MatrixXd::Zero(2,2);

		//new H for new hyp
		for (int i=0;i<n;i++){
			Eigen::MatrixXd xr = _sensors.col(i);
			Eigen::MatrixXd ht = get_H(xr,tp);
			double expz = atan2(tp(1)-sy[i],tp(0)-sx[i]);
			_y(i) = normanglediff(expz,_z(i));
			if (ambig){
				if (fabs(_y(i))>M_PI/2){

#ifdef __TL_VERBOSE
					printf("flipping: %d\n",i);
#endif
					_z(i)=RSN::normalizeAngle(bearing_global_radians[i]+M_PI);
					_y(i) = normanglediff(expz,_z(i));
				}
			}
			_H.row(i) = ht;
			_HRi.row(i) = _H.row(i)/((_R(i)*_R(i)));
		}

		//calc FIM(x hat)
		_A = _A + _HRi.transpose()*_H;
		//calc norm innov
		_nZ = _nZ + _HRi.transpose() * _y;

		double sc = scond(_A);
#ifdef __TL_VERBOSE
		printf("svd_cond (_A): %0.3f\n",sc);
#endif
		if (sc>99999999||sc!=sc){
			fprintf(stderr,"Matrix singular to working precision (sc=%f). That's a wrap.\n",sc);
			return;
		}

		Eigen::Vector2d dx= _A.ldlt().solve(_nZ);
		tp = tp + dx;
		ddx = dx.dot(dx);
#ifdef __TL_VERBOSE
		printf("%f\n",ddx);
#endif
	}

	target[0] = tp(0);
	target[1] = tp(1);
	_A = get_FIM(_sensors,_R,tp);
	_A = _A.inverse();
	target_cov[0] = _A(0,0);
	target_cov[1] = _A(0,1);
	target_cov[2] = _A(1,0);
	target_cov[3] = _A(1,1);
}
void RSN::BOT::IWLS_2D(const int n, const double sensors[][2], const double bearing_global_radians[], const double sensor_sigma[],double target[2], double target_cov[4], const bool ambig, double min_step, int max_iters)
{
	std::vector<double>sx,sy;
	for (int i=0;i<n;i++){
		sx.push_back(sensors[i][0]);
		sy.push_back(sensors[i][1]);
	}
	std::vector<double>bs = getVec(n,bearing_global_radians);
	std::vector<double>rs = getVec(n,sensor_sigma);
	RSN::BOT::IWLS_2D(n,sx,sy,bs,rs,target,target_cov,ambig,min_step,max_iters);
}
void RSN::BOT::IWLS_2D(const int n, const std::vector<std::vector<double> >sensors, const std::vector<double> bearing_global_radians, const std::vector<double> sensor_sigma,double target[2], double target_cov[4], const bool ambig, double min_step, int max_iters)
{
	std::vector<double>sx,sy;
	for (int i=0;i<n;i++){
		sx.push_back(sensors[i][0]);
		sy.push_back(sensors[i][1]);
	}
	RSN::BOT::IWLS_2D(n,sx,sy,bearing_global_radians,sensor_sigma,target,target_cov,ambig,min_step,max_iters);
}

void RSN::BOT::EKF_UP_2D(const double sx, const double sy, const double bearing_global_radians, const double sensor_sigma, double target[2], double target_cov[4], const bool ambig){
#ifdef __TL_VERBOSE
	printf("EKF_UP_2D:%d,%s\n",__LINE__,__FILE__);
#endif
	Eigen::MatrixXd tp(2,1); tp<<target[0],target[1];
	Eigen::MatrixXd xr(2,1); xr<<sx,sy;
	Eigen::Matrix2d P = getCovMatrix(target_cov);
	Eigen::MatrixXd H = get_H(xr,tp);
	Eigen::MatrixXd S = get_S(xr,tp,P,sensor_sigma);

	double globalZ = RSN::normalizeAngle(bearing_global_radians);
	double zhypoth = atan2(tp(1)-xr(1),tp(0)-xr(0));
	double innov = RSN::normanglediff(zhypoth,globalZ);

	innov=RSN::normalizeAngle(innov);

	if (ambig && fabs(innov)>M_PI/2){
		innov = RSN::normalizeAngle(innov+M_PI);
	}

	//So 'S' is a scalar ...
	Eigen::MatrixXd K=P*H.transpose()*S.inverse();
	Eigen::MatrixXd xx = K*innov;
	Eigen::MatrixXd xp = tp+xx;
	Eigen::MatrixXd Pp=(Eigen::MatrixXd::Identity(2,2)-K*H)*P;

	target[0] = xp(0);
	target[1] = xp(1);
	target_cov[0] = Pp(0,0);
	target_cov[1] = Pp(0,1);
	target_cov[2] = Pp(1,0);
	target_cov[3] = Pp(1,1);

	return;
}
void RSN::BOT::EKF_UP_2D(const double sensor[2], const double bearing_global_radians, const double sensor_sigma, double target[2], double target_cov[4], const bool ambig){
	EKF_UP_2D(sensor[0],sensor[1],bearing_global_radians,sensor_sigma,target,target_cov, ambig);
}
Eigen::MatrixXd RSN::BOT::get_FIM(const Eigen::MatrixXd col_wise_sensors, const Eigen::MatrixXd vector_of_sigma, const Eigen::MatrixXd target)
{
#ifdef __TL_VERBOSE
	printf("get_FIM:%d,%s\n",__LINE__,__FILE__);
#endif
	unsigned int n = col_wise_sensors.cols();
	Eigen::MatrixXd _H(n,2); 
	Eigen::MatrixXd _HRi(n,2); 
	Eigen::MatrixXd _A=Eigen::MatrixXd::Zero(2,2);
	for (unsigned int i=0;i<n;i++){
		Eigen::MatrixXd xr = col_wise_sensors.col(i);
		Eigen::MatrixXd ht = get_H(xr,target);
		_H.block<1,2>(i,0) = ht;
		_HRi.row(i) = _H.row(i)/(vector_of_sigma(i)*vector_of_sigma(i));
	}
	_A = _A + _HRi.transpose()*_H;
	return _A;
}
Eigen::MatrixXd RSN::BOT::get_S(const double sensor[2],const double target[2],const double cov[4],const double sensor_sigma){	
	Eigen::MatrixXd P(2,2); P<<cov[0],cov[1],cov[2],cov[3];
	Eigen::MatrixXd H = get_H(sensor,target);
	Eigen::MatrixXd S=H*P*H.transpose() + (Eigen::MatrixXd(1,1)<<sensor_sigma*sensor_sigma).finished();
	return S;
}
Eigen::MatrixXd RSN::BOT::get_S(const Eigen::MatrixXd sensor,const Eigen::MatrixXd target,const Eigen::MatrixXd P, const double sensor_sigma){	
	//Eigen::MatrixXd P(2,2); P<<cov[0],cov[1],cov[2],cov[3];
	Eigen::MatrixXd H = get_H(sensor,target);
	Eigen::MatrixXd S=H*P*H.transpose() + (Eigen::MatrixXd(1,1)<<sensor_sigma*sensor_sigma).finished();
	return S;
}
Eigen::MatrixXd RSN::BOT::get_H(const Eigen::MatrixXd sensor,const Eigen::MatrixXd target){
	Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1,2);
	//double dx = sensor[0]-target[0];
	//double dy = sensor[1]-target[1];
	// Compatible with eigen3
	double dx = sensor(0)-target(0);
	double dy = sensor(1)-target(1);
	double dd = dx*dx+dy*dy;
	if (dd>0){
		H(0,0) = -dy/dd;
		H(0,1) = dx/dd;
	}
	return H;
}

Eigen::MatrixXd RSN::BOT::get_H(const double sensor[2],const double target[2]){
	Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1,2);
	double dx = sensor[0]-target[0];
	double dy = sensor[1]-target[1];
	double dd = dx*dx+dy*dy;
	if (dd>0){
		H(0,0) = -dy/dd;
		H(0,1) = dx/dd;
	}
	return H;
}

Eigen::MatrixXd RSN::getCovMatrix(const double covariance[4]){
	Eigen::MatrixXd P(2,2); P<<covariance[0],covariance[1],covariance[2],covariance[3];
	return P;
}
void RSN::line_intersection(const size_t N, double p[][2], double th[], double &px, double &py,double c[4]){
	std::vector<double> ths = getVec(N,th);
	std::vector<std::vector<double> > ss = getVec(N,p);
	RSN::line_intersection(N, ss, ths, px, py,c);
}

void RSN::line_intersection(const size_t N, const std::vector<double>px, std::vector<double>py, std::vector<double> th, double &opx, double &opy,double c[4]){
#ifdef __TL_VERBOSE
	printf("line_intersection:%d,%s\n",__LINE__,__FILE__);
#endif


	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N,2);
	Eigen::MatrixXd b = Eigen::MatrixXd::Zero(N,1);
	for (unsigned int i=0;i<N;i++){
		double sth = sin(th[i]);
		double cth = cos(th[i]);
		A(i,0) = -sth;
		A(i,1) = cth;
		b(i) = -sth*px[i]  + cth*py[i];
	}
#ifdef __TL_VERBOSE
	std::cout<<A<<std::endl;
#endif
	Eigen::MatrixXd y = A.transpose()*b;
	Eigen::MatrixXd Ap = A.transpose()*A;
	Eigen::MatrixXd x = Ap.ldlt().solve(y);
	opx = x(0);
	opy = x(1);
	Eigen::MatrixXd _sensors=Eigen::MatrixXd::Zero(2,N);
	Eigen::MatrixXd _R = Eigen::MatrixXd::Zero(1,N);

	if (c!=NULL){
		for (unsigned int i=0;i<N;i++){
			Eigen::MatrixXd xr(2,1); xr<<px[i],py[i];
			_R(i)=M_PI/4;
			_sensors.col(i) = xr;
		}
		Eigen::MatrixXd F = BOT::get_FIM(_sensors,_R,x);
		c[0]=F(0,0);c[1]=F(0,1);
		c[1]=F(1,0);c[2]=F(1,1);
	}
}
void RSN::line_intersection(const size_t N, const std::vector<std::vector<double> >p, std::vector<double> th, double &px, double &py,double c[4]){
#ifdef __TL_VERBOSE
	printf("line_intersection:%d,%s\n",__LINE__,__FILE__);
#endif
	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N,2);
	Eigen::MatrixXd b = Eigen::MatrixXd::Zero(N,1);
	for (unsigned int i=0;i<N;i++){
		double sth = sin(th[i]);
		double cth = cos(th[i]);
		A(i,0) = -sth;
		A(i,1) = cth;
		b(i) = -sth*p[i][0]  + cth*p[i][1];
	}
#ifdef __TL_VERBOSE
	std::cout<<A<<std::endl;
#endif
	Eigen::MatrixXd y = A.transpose()*b;
	Eigen::MatrixXd Ap = A.transpose()*A;
	Eigen::MatrixXd x = Ap.ldlt().solve(y);
	px = x(0);
	py = x(1);

	if (c!=NULL){
		Eigen::MatrixXd _sensors = Eigen::MatrixXd::Zero(2,N);
		Eigen::MatrixXd _R= Eigen::MatrixXd::Zero(1,N);
		for (unsigned int i=0;i<N;i++){
			Eigen::MatrixXd xr(2,1); xr<<p[i][0],p[i][1];
			_R(i)=M_PI/4.0;
			_sensors.col(i) = xr;
		}
		Eigen::MatrixXd F = BOT::get_FIM(_sensors,_R,x);
		c[0]=F(0,0);c[1]=F(0,1);
		c[1]=F(1,0);c[2]=F(1,1);
	}
}
void RSN::circumcircle(const size_t N, const std::vector<std::vector<double> >p, double &centerx, double &centery, double& radius){
#ifdef __TL_VERBOSE
	for(int i=0; i<(int)N; i++) {
		printf("#CIRCUMCIRCLE_INPUT: [%.5f %.5f]\n",p[i][0],p[i][1]);
	}
#endif

	if (N<2){radius=0;centerx=0;centery=0;return;}
	if (N==2){
		centerx = p[0][0]+p[1][0];
		centerx /=2;
		centery = p[0][1]+p[1][1];
		centery /=2;
		double dx = p[0][0]-p[1][0];
		double dy = p[0][1]-p[1][1];
		radius = (.5)*sqrt(dx*dx+dy*dy);
		return;
	}
	double dx[N-1];
	double dy[N-1];

	double dd[N-1];
	double hpx[N-1];
	double hpy[N-1];

	double nx[N-1];
	double ny[N-1];

#ifdef TAGLOC_USE_CV
	cv::Mat_<double> A= cv::Mat::zeros(N-1,2,CV_64F);
	cv::Mat_<double> b= cv::Mat::zeros(N-1,1,CV_64F);
#else
	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N-1,2);
	Eigen::MatrixXd b = Eigen::MatrixXd::Zero(N-1,1);
#endif

	for (int i=0;i<(int)N-1;i++){
		//printf("(%0.3f,%0.3f)",p[i+1][0],p[i+1][1]);
		//printf("-(%0.3f,%0.3f)",p[i][0],p[i][1]);
		dx[i]=p[i+1][0]-p[i][0];
		dy[i]=p[i+1][1]-p[i][1];
		//printf("=(%0.3f,%0.3f)\n",dx[i],dy[i]);
		dd[i] = dx[i]*dx[i]+dy[i]*dy[i];
		dd[i] = sqrt(dd[i]);
		if (dd[i]>.001){
			nx[i] = dx[i]/dd[i];
			ny[i] = dy[i]/dd[i];
		}
		else{
			nx[i] = 0;
			ny[i] = 0;
		}
		hpx[i] = p[i][0]+nx[i]*dd[i]/2;
		hpy[i] = p[i][1]+ny[i]*dd[i]/2;
		//printf("hp:%0.3f,%0.3f with dist: ",hpx[i],hpy[i]);
		//printf("%0.3f\n",dd[i]);
		b(i)=hpx[i]*nx[i] + hpy[i]*ny[i];
		A(i,0)=nx[i];
		A(i,1)=ny[i];
	}
#ifdef TAGLOC_USE_CV
	cv::Mat_<double> x = (A.t()*A).inv()*A.t()*b;
#else
	Eigen::MatrixXd bp = A.transpose()*b;
	Eigen::MatrixXd Ap = A.transpose()*A;
#ifdef __TL_VERBOSE
	printf("\tsvd_cond (A'): %0.3f\n",scond(Ap));
#endif
	Eigen::MatrixXd x = Ap.ldlt().solve(bp);
#endif
	centerx = x(0);
	centery = x(1);
	radius=0;
	for (int i=0;i<(int)N;i++){
		radius = radius + sqrt(std::pow(centerx - p[i][0],2) + std::pow(centery-p[i][0],2));
	}
	radius = radius / N;
}
void RSN::circumcircle(const size_t N, double px[], double py[], double &centerx, double &centery, double& radius){
	
	std::vector<std::vector<double> > pts(N,std::vector<double>(2)); //I hate this...

	for(int i=0; i<(int)N; i++) {
		//printf("#CIRCUMCIRCLE_INPUT: [%.5f %.5f]\n",px[i],py[i]);
		pts[i][0]=px[i];
		pts[i][1]=py[i];
	}
	RSN::circumcircle(N,pts,centerx,centery,radius);
}




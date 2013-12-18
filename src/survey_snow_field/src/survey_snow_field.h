#include <eigen3/Eigen/Dense>

#define NUMDATAPOINTS 25
#define NUMCORNERS 2	
#define PI 3.14159265359

double gpsTolerance = 0.00001;

void geod2ecef(double lat, double lon, double h, double& x, double& y, double& z){
    //constants for WGS-84 system
    double a = 6378137.0;              // earth semimajor axis [m]
    double f = 298.257223563;           // flattening index     [?]
    double b = a - ((1/f)*a);           // earth semiminor axis [m]
    
    double N = a*a/sqrt(a*a * cos(lat)*cos(lat) + b*b * sin(lat)*sin(lat));
    double N1 = (b/a)*(b/a) * N;
    
    x = (N+h)*cos(lat)*cos(lon);
    y = (N+h)*cos(lat)*sin(lon);
    z = (N1+h)*sin(lat);
}


class Coordinates{

 public: 
  double lat;
  double lon;
  double h;
  double east;
  double north;
  double up;

  double reflat;
  double reflon;
  double refh;

  Coordinates(){
    lat = 0;
    lon = 0;
    h = 0;
    east = 0;
    north = 0;
    up = 0;
    reflat = 0;
    reflon = 0;
    refh = 0;
  }

  Coordinates(double lat, double lon, double h){
    this->lat = lat;
    this->lon = lon;
    this->h = h;
    east = 0;
    north = 0;
    up = 0;
    reflat = 0;
    reflon = 0;
    refh = 0;
  }

  void setReference(Coordinates ref){
    this->reflat = ref.lat;
    this->reflon = ref.lon;
  }

  void geod2enu(){
    //convert to radians:
    double deg2rad = PI/180;
    double latrad = lat * deg2rad;
    double lonrad = lon * deg2rad;
    double reflatrad = reflat * deg2rad;
    double reflonrad = reflon * deg2rad;

    //fill in cartesian coordinates:
    double pointx = 0, pointy = 0, pointz = 0;
    double refx = 0, refy = 0, refz = 0; 
    geod2ecef(latrad,lonrad,h,pointx,pointy,pointz);
    geod2ecef(reflatrad,reflonrad,refh,pointx,pointy,pointz);

    //create rotation matrix & vectors:
    Eigen::MatrixXf rotationMatrix(3,3);
    rotationMatrix << -sin(lonrad) , cos(lonrad) , 0
	, -sin(latrad)*cos(lonrad) , -sin(latrad)*sin(lonrad) , cos(latrad)
	, cos(latrad)*cos(lonrad) , -cos(latrad)*sin(lonrad) , sin(latrad);

    Eigen::VectorXf positionDiffVector(3,1);
    positionDiffVector << refx-pointx,  refy-pointy, refz-pointz;

    Eigen::VectorXf enu(3,1);
    enu = rotationMatrix * positionDiffVector;

    this->east = enu(1,1);
    this->north = enu(1,2);
    this->up = enu(1,3);

  }

};



#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <iostream>
#include <vector>
#include <string>  

#include <fstream>
#include <sstream>
#include <cmath> 
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;


class ParticleFilter{
	private:
	
	struct Odometry
	{
		double r1;
		double t;
		double r2;
	};
	
	vector<Odometry> odomVec;
	
	struct Particle
	{
		double weight;
		double poseX;
		double poseY;
		double poseTheta;
		
      };
      
    vector<Particle> particles;
    
	double NormalAngle(double angle);
	
	public:
	
	void PF();
	
};










#endif // PARTICLEFILTER_H

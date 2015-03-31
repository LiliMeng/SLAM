#include <vector>
#include <string>  
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath> 
#include <Eigen/Core>
#include <Eigen/Dense>
#include <random>
#include "ParticleFilter.h"


using namespace std;
using namespace Eigen;

double ParticleFilter::NormalAngle(double angle)
{
	while(angle>M_PI)
	{
		angle=angle-2*M_PI;
		
		}
	while(angle<-M_PI)
	{
		angle=angle+2*M_PI;
		}
	return angle;
}


void ParticleFilter::PF(){
	
	string a;
	
	ifstream fin("odometry.dat");
	if(!fin)
	{
		cout<<"Error"<<endl;
	}
    ofstream fout("ParticleFilter.csv");
    if(!fout)
    {
	   cout<<"cannot output data"<<endl;
    } 
    
    while(fin>>a)
    {
	   
		if (a.compare("ODOMETRY")==0)
			{  
				Odometry Odom;
				fin>>a;
				Odom.r1 = NormalAngle(stod(a)); 
				fin>>a; 
				Odom.t = stod(a); 
				fin>>a;
				Odom.r2=NormalAngle(stod(a));
				
				//cout<<Odom.r1<<"\t"<<Odom.t<<"\t"<<Odom.r2<<"\t"<<endl;
				//fout<<Odom.r1<<" "<<Odom.t<<" "<<Odom.r2<<" "<<endl;
				odomVec.push_back(Odom);
			}
			//cout<<odomVec.size()<<endl;
		}
	
	double r1Noise = 0.005;
    double transNoise = 0.01;
    double r2Noise = 0.005;
          
    int numParticles = 100;
    
    
     //initialize the particles
     for (int m=0; m<numParticles;m++)
     {
		 Particle Par;
		 Par.weight=1.0/numParticles;
		 Par.poseX=0;
		 Par.poseY=0;
		 Par.poseTheta=0;
		 particles.push_back(Par); 
     }
     
    
    vector<double> weightVec;
    
    for(int t=0; t<odomVec.size(); t++)
    {
		
		std::default_random_engine generator;
		std::normal_distribution<double> distributionR1(odomVec[t].r1,r1Noise);
		std::normal_distribution<double> distributionTrans(odomVec[t].t,transNoise);
		std::normal_distribution<double> distributionR2(odomVec[t].r2,r2Noise);
    
		//Perform filter update for each odometry-observation read from the data file
		for (int m=0; m<numParticles; m++)
		{
			double Dr1=distributionR1(generator);
			double Dtrans=distributionTrans(generator);
			double Dr2=distributionR2(generator);
        
            particles[m].weight=1.0/numParticles;
			particles[m].poseX = particles[m].poseX+Dtrans*cos(particles[m].poseTheta+Dr1);
			particles[m].poseY=particles[m].poseY+Dtrans*sin(particles[m].poseTheta+Dr1);
			particles[m].poseTheta=NormalAngle(particles[m].poseTheta+Dr1+Dr2);	
     
			particles.push_back(particles[m]);
			weightVec.push_back(particles[m].weight);
			//cout<<particles[m].poseX<<"\t"<<particles[m].poseY<<"\t"<<particles[m].poseTheta<<"\t"<<particles[m].weight<<endl;
			//fout<<particles[m].poseX<<" "<<particles[m].poseY<<" "<<particles[m].poseTheta<<endl;
			
		//Resampling using low variance sampler 
		double n=1.0/numParticles;
		std::random_device rd;
        std::mt19937_64 mt(rd());
        std::uniform_real_distribution<double> distribution(0, n);
        double r=distribution(mt);
        double c=weightVec[0];
        int i=0;
        //cout<<r<<endl;
        
        for(int m=0; m<numParticles; m++)
        {
			double U=r+(m-1)*1.0/numParticles;
			while(U>c)
			{
				i=i+1;
				c=c+weightVec[i];
				}
	     particles.push_back(particles[i]);
		} 
		
		cout<<particles[m].poseX<<"\t"<<particles[m].poseY<<"\t"<<particles[m].poseTheta<<"\t"<<particles[m].weight<<endl;
	    fout<<particles[m].poseX<<" "<<particles[m].poseY<<" "<<particles[m].poseTheta<<endl;
		}
	}
	return;
}
	


int main()
{
	
       ParticleFilter PF;
       
       PF.PF();
    
       return 0;
     
	}

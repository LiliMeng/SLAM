#include <stdio.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include <cmath> 
#include "EKFSLAM.h"

using namespace Eigen;
using namespace std;

double EKFSLAM::NormalAngle(double angle)
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

void EKFSLAM::ekfslam()
{
	string a;
	double b=0;
	double c=0;
	double d=0;

	vector<int> landmarkcountvec;
	vector<struct Odometry2> combine;

	ifstream fin("sensor_data.dat");
	
	if(!fin)
	{
		cout<<"Error"<<endl;
	}
	
	int landmarkcount=0;
     
    char str[200];
    int count=-1;
    while(fin.getline(str,200))
    {
		string m=str;
		istringstream iss(m);
		iss>>a;
		iss>>b;
		iss>>c;
		iss>>d;
	    
	    //cout<<a<<"\t"<<b<<"\t"<<c<<"\t"<<d<<"\t"<<endl;
	    
			if (a.compare("ODOMETRY")==0)
			{  
				if(landmarkcount>0)
					landmarkcountvec.push_back(landmarkcount);
				landmarkcount=0;
				Odometry2 Odom;
				Odom.r1 =NormalAngle(b);
				Odom.t = c;
				Odom.r2 =NormalAngle(d);
				vector<struct sensor> tmpsensor;
				Odom.senso=tmpsensor;
				count++;
				combine.push_back(Odom);
				
			}
			else
			{   
				landmarkcount++;
				sensor sen;
				sen.id = b;
				sen.r = c;
				sen.theta = NormalAngle(d);
				combine[count].senso.push_back(sen);
			}
	}
	
	ofstream fout("sensor_data.csv");
    if(!fout)
    {
	   cout<<"cannot output data"<<endl;
    } 
    
    
     int N=landmarkcountvec.size();
     landmark observedLandmarks[9]; //it shall be declared outside the for loop, if not, it was initiliazed false every loop
     //Initialization of mu
	 mu=MatrixXd::Zero(3+2*m,1);
     //Initialization of covariance matrix Sigma
     Sigma=MatrixXd::Zero(3+2*m,3+2*m);
     
       for(int i=0;i<N;i++)
      {
	    
		//odometry model for updating the robot pose
		MatrixXd OdomModel(3,1);
		OdomModel<< combine[i].t*cos(mu(2,0)+combine[i].r1),
					combine[i].t*sin(mu(2,0)+combine[i].r1),
					combine[i].r1+combine[i].r2;      
	 
	    OdomModel(2,0)=NormalAngle(OdomModel(2,0));
		
		//map the robot pose + landmark positions to 3+2*m dimensions (as it's SLAM problem, the robot pose and landmark positions shall be estimated and updated together)    
		MatrixXd F;
		F=MatrixXd::Identity(3,3+2*m);
		
		// update the mu (robot pose+landmark posiinvisiligntions) to 3+2*m dimensions; 
		mu=mu+F.transpose()*OdomModel;
	    
	    mu(2,0)=NormalAngle(mu(2,0));
	    
		//update the covariance (robot pose+ landmark positions) matrix
		MatrixXd PredictJacobianG(3+2*m,3+2*m);
		//Jacobian of the robot motion
		MatrixXd RobotMotionJacobian(3,3);
		RobotMotionJacobian<<0,0,-combine[i].t*sin(mu(2,0)+combine[i].r1),
							0,0,combine[i].t*cos(mu(2,0)+combine[i].r1),
							0,0,0;
	
		//Map the JacobianG to 3+2*m space by including both the robot motion and the landmarks covariance
		PredictJacobianG=MatrixXd::Identity(3+2*m,3+2*m)+F.transpose()*RobotMotionJacobian*F;
		
		double motionNoise=0.1;
		MatrixXd R(3,3);
		R<<motionNoise, 0, 0,
			0, motionNoise, 0,
			0, 0, motionNoise/10;
	
		//Covariance Update equation, till now the EKF prediction step is done! 
		Sigma=PredictJacobianG*Sigma*PredictJacobianG.transpose()+F.transpose()*R*F;

        
		//For all observed landmarks, do the following: 
		for (int k=0; k< landmarkcountvec[i]; k++) 
		{
            int j=combine[i].senso[k].id;
             //ZObservation
			MatrixXd ZObservation(2,1);
		    ZObservation(0,0)=combine[i].senso[k].r;
			ZObservation(1,0)=combine[i].senso[k].theta;
	        
			//if landmark j has never seen before:
			if(observedLandmarks[j].ini==false)
			{
				mu(3+2*j-2,0)=mu(0,0)+combine[i].senso[k].r*cos(combine[i].senso[k].theta+mu(2,0));
				mu(3+2*j-1,0)=mu(1,0)+combine[i].senso[k].r*sin(combine[i].senso[k].theta+mu(2,0));          
				observedLandmarks[j].ini=true;
			}

			//Compute expected observation according to the current estimate
			MatrixXd Delta(2,1);
			Delta(0,0)=mu(3+2*j-2,0)-mu(0,0),
			Delta(1,0)=mu(3+2*j-1,0)-mu(1,0);
	        
			double q=Delta(0,0)*Delta(0,0)+Delta(1,0)*Delta(1,0); 
	           
	        //Predicted observation
			MatrixXd h(2,1);
			h<<sqrt(q),
				NormalAngle(atan2(Delta(1,0),Delta(0,0))-mu(2,0));
			
			// Observation Jacobian
			
			MatrixXd JacobianHlow(2,5);
			
			JacobianHlow<<-Delta(0,0)*sqrt(q)/q,-Delta(1,0)*sqrt(q)/q,0,Delta(0,0)*sqrt(q)/q, Delta(1,0)*sqrt(q)/q,
							Delta(1,0)/q, -Delta(0,0)/q, -1, -Delta(1,0)/q, Delta(0,0)/q;
			
			
			MatrixXd Fj;
			Fj=MatrixXd::Zero(5,3+2*m);
			Fj(0,0)=1;
			Fj(1,1)=1;
			Fj(2,2)=1;
			Fj(3,3+2*j-2)=1;
			Fj(4,3+2*j-1)=1;
	             
	            
			//Map the Jacobian from lower to higher space  
	             
			MatrixXd JacobianHhigh(2,3+2*m);
		    JacobianHhigh=JacobianHlow*Fj;
			
	        //Observation Noise Q
	        MatrixXd Q(2,2);
			Q<<0.01,0,
			    0,0.01;
	               
			//Compute the Kalman Gain
			MatrixXd K;
			
			MatrixXd Temp;
			Temp=JacobianHhigh*Sigma*JacobianHhigh.transpose()+Q;
			
			K=Sigma*JacobianHhigh.transpose()*Temp.inverse();
			
			//Update mu and Sigma according to Observation
			MatrixXd Zdifference(2,1);
			Zdifference=ZObservation-h;
			Zdifference(1,0)=NormalAngle(Zdifference(1,0));
			
			mu=mu+K*Zdifference;
			Sigma=(MatrixXd::Identity(3+2*m,3+2*m)-K*JacobianHhigh)*Sigma;
			
			mu(2,0)=NormalAngle(mu(2,0));
		}
		cout<<mu(0,0)<<", "<<mu(1,0)<<", "<<mu(2,0)<<endl;  
		fout<<mu(0,0)<<" "<<mu(1,0)<<" "<<mu(2,0)<<endl;
		
   }
   
}


int main()
{        
	   EKFSLAM a;
       a.ekfslam();
  
	   return 0;
}

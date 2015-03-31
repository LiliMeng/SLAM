#include <stdio.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include <cmath> 
#include <limits>
#include <algorithm>
#include "EKFSLAM_UnknownCorrespondence.h"

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
     //Initialization of mu
	 mu=MatrixXd::Zero(3+2*m,1);
     //Initialization of covariance matrix Sigma
     Sigma=MatrixXd::Zero(3+2*m,3+2*m);
     /*for(int i=3;i<3+2*m; i++)
     Sigma(i,i)=std::numeric_limits<double>::max(); */  // if set the Sigma(i,i) to max or infinity, the output is -nan
     
     
       for(int t=1;t<66;t++)
      {
		//odometry model for updating the robot pose
		MatrixXd OdomModel(3,1);
		OdomModel<< combine[t].t*cos(mu(2,0)+combine[t].r1),
					combine[t].t*sin(mu(2,0)+combine[t].r1),
					combine[t].r1+combine[t].r2;      
	 
	    OdomModel(2,0)=NormalAngle(OdomModel(2,0));
		
		//map the robot pose + landmark positions to 3+2*m dimensions (as it's SLAM problem, the robot pose and landmark positions shall be estimated and updated together)    
		MatrixXd F;
		F=MatrixXd::Identity(3,3+2*m);
		MatrixXd Zdifference(2,1);
		// update the mu (robot pose+landmark posiinvisiligntions) to 3+2*m dimensions; 
		mu=mu+F.transpose()*OdomModel;
	    
	    mu(2,0)=NormalAngle(mu(2,0));
	    
		//update the covariance (robot pose+ landmark positions) matrix
		MatrixXd PredictJacobianG(3+2*m,3+2*m);
		//Jacobian of the robot motion
		MatrixXd RobotMotionJacobian(3,3);
		RobotMotionJacobian<<0,0,-combine[t].t*sin(mu(2,0)+combine[t].r1),
							0,0,combine[t].t*cos(mu(2,0)+combine[t].r1),
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
      
        int Nt;
        if(t==0)
        Nt=0;
        else
        Nt= landmarkcountvec[t-1];
       
		//For all observed landmarks, do the following: 
		for (int i=0; i< landmarkcountvec[t]; i++) 
		{
			MatrixXd Phai;
			MatrixXd JacobianHhigh(2,3+2*m);
			vector<double> Pi_vector;
			
             //ZObservation
			MatrixXd ZObservation(2,1);
		    ZObservation(0,0)=combine[t].senso[i].r;
			ZObservation(1,0)=combine[t].senso[i].theta;
	        
			mu(3+2*(Nt+1)-2,0)=mu(0,0)+combine[t].senso[i].r*cos(combine[t].senso[i].theta+mu(2,0));
			mu(3+2*(Nt+1)-1,0)=mu(1,0)+combine[t].senso[i].r*sin(combine[t].senso[i].theta+mu(2,0));      
			
			// for k=1 to Nt+1, do
			for(int k=1;k<Nt+2;k++)
			{
			MatrixXd Delta(2,1);
			Delta(0,0)=mu(3+2*k-2,0)-mu(0,0),
			Delta(1,0)=mu(3+2*k-1,0)-mu(1,0);
	        
			double q=Delta(0,0)*Delta(0,0)+Delta(1,0)*Delta(1,0); 
	           
	        //Predicted observation
			MatrixXd h(2,1);
			h<<sqrt(q),
				NormalAngle(atan2(Delta(1,0),Delta(0,0))-mu(2,0));
			
			// Observation Jacobian
			MatrixXd JacobianHlow(2,5);
			JacobianHlow<<-Delta(0,0)*sqrt(q)/q,-Delta(1,0)*sqrt(q)/q,0,Delta(0,0)*sqrt(q)/q, Delta(1,0)*sqrt(q)/q,
							Delta(1,0)/q, -Delta(0,0)/q, -1, -Delta(1,0)/q, Delta(0,0)/q;
			
			MatrixXd Fk;
			Fk=MatrixXd::Zero(5,3+2*m);
			Fk(0,0)=1;
			Fk(1,1)=1;
			Fk(2,2)=1;
			Fk(3,3+2*k-2)=1;
			Fk(4,3+2*k-1)=1; 
	            
			//Map the Jacobian from lower to higher space  
		    JacobianHhigh=JacobianHlow*Fk;
			
	        //Observation Noise Q
	        MatrixXd Q(2,2);
			Q<<0.01,0,
			    0,0.01;
	        
	        Phai=JacobianHhigh*Sigma*JacobianHhigh.transpose()+Q;
	        
			Zdifference=ZObservation-h;
			Zdifference(1,0)=NormalAngle(Zdifference(1,0));
			
			MatrixXd Pi;
			Pi=Zdifference.transpose()*Phai.inverse()*Zdifference;
			
			//cout<<Pi(0,0)<<endl;
		    Pi_vector.push_back(Pi(0,0));
		    } 
		    
		    double PiNtPlusOne;
	        PiNtPlusOne=100;
	        Pi_vector.push_back(PiNtPlusOne);
	        
	        double j=*(min_element(Pi_vector.begin(),Pi_vector.end()));
	        
	        if(Nt<j)
	        Nt=j;
	        else
	        Nt=Nt;
            
			//Compute the Kalman Gain
			MatrixXd K;
			K=Sigma*JacobianHhigh.transpose()*Phai.inverse();
			
			//Update mu and Sigma according to Observation
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

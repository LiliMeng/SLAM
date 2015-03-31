#ifndef EKFSLAM_UNKNOWNCORRESPONDENCE_H
#define EKFSLAM_UNKNOWNCORRESPONDENCE_H

#include<Eigen/Dense>
#include<vector>

using namespace std;
using namespace Eigen;


class EKFSLAM
{
    public:
        
        MatrixXd mu;
        MatrixXd Sigma;
        
        struct sensor
		{
			int id;
			double r;
			double theta;
		};

		struct Odometry2
		{
			double r1;
			double t;
			double r2;
			vector<sensor> senso;
		};
		
		struct landmark
		{
			bool ini;
			landmark()
			{
				ini = false;
			}
			
		};
		
		
		int m=9;
		
        double NormalAngle(double angle);
       
        void ekfslam();
     
       
    

};

#endif // EKFSLAM_UNKNOWNCORRESPONDENCE_H

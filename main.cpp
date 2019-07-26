#include "KF.h"
#include <iostream>
#include "Eigen/Dense"

using namespace std;

int main(){
	double m_x = 0.0;
	double m_y = 0.0;
	double last_timestamp = 0.0;
	double now_timestamp = 0.0;
	KalmanFilter kf;

	while(1){
		// initialize kalman filter
		if (! kf.IsInitialized()){
			last_timestamp = now_timestamp;

			//initialize state vector x
			Eigen::VectorXd x_in(4,1);
			x_in << m_x, m_y, 0.0, 0.0;
			kf.Initialization(x_in);

			//initialize control vector
			Eigen::VectorXd u_in(4,1);
			u_in << 0.0, 0.0, 0.0, 0.0;
			kf.SetU(u_in);

			//initialize covariance matrix P
			Eigen::MatrixXd P_in(4,4);
			P_in << 1, 0, 0 ,0,
					0, 1, 0, 0,
					0, 0, 100, 0,
					0, 0, 0, 100;
			kf.SetP(P_in);

			//initialize process covariance matrix Q
			Eigen::MatrixXd Q_in(4,4);
			Q_in << 1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1;
			kf.SetQ(Q_in);

			//initialize Measurement Matrix H
			Eigen::MatrixXd H_in(2,4);
			H_in << 1, 0, 0, 0,
					0, 1, 0, 0;
			kf.SetH(H_in);

			//initialize measurement covariance matrix R
			//R is provided by Sensor supplier
			Eigen::MatrixXd R_in(2,2);
			R_in << 0.0225, 0,
					0, 0.0225;
			kf.SetR(R_in);
			//continue;
		}

		//state transistion matrix
		double delta_t = now_timestamp - last_timestamp;
		Eigen::MatrixXd F_in(4,4);
		F_in << 1, 0, delta_t, 0,
				0, 1, 0, delta_t,
				0, 0, 1, 0,
				0, 0, 0, 1;
		kf.SetF(F_in);
		kf.prediction();

		//measurement value Z
		Eigen::VectorXd Z(2,1);
		Z << m_x, m_y;
		kf.MeasurementUpdate(Z);

		Eigen::VectorXd x_out = kf.Get_x();
		cout<<"kalman filter output x :"<< x_out(0) <<endl;
		cout<<"kalman filter output y :"<< x_out(1) <<endl;
	}
}
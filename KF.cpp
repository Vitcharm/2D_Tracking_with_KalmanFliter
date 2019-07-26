#include "KF.h"
#include <iostream>
using namespace std;

KalmanFilter::KalmanFilter(void){
	is_initialized_ = false;
	cout<<"Object is being created"<<endl;
}

KalmanFilter::~KalmanFilter(void){
	cout<<"Object is being deleted"<<endl;
}

void KalmanFilter::Initialization(Eigen::VectorXd x_in){
	x_ = x_in;
}

bool KalmanFilter::IsInitialized(){
	return is_initialized_;
}

void KalmanFilter::SetU(Eigen::VectorXd u_in){
	u_ = u_in;
}	

void KalmanFilter::SetF(Eigen::MatrixXd F_in){
	F_ = F_in;
}

void KalmanFilter::SetP(Eigen::MatrixXd P_in){
	P_ = P_in;
}

void KalmanFilter::SetQ(Eigen::MatrixXd Q_in){
	Q_ = Q_in;
}

void KalmanFilter::SetH(Eigen::MatrixXd H_in){
	H_ = H_in;
}

void KalmanFilter::SetR(Eigen::MatrixXd R_in){
	R = R_in;
}

void KalmanFilter::prediction(){
	x_ = F_ * x_ + u_;	
	Eigen::MatrixXd Ft = F_.transpose();
	P_ = (F_ * P_ * Ft) + Q_;
}

void KalmanFilter::MeasurementUpdate(const Eigen::VectorXd &Z){
	Eigen::MatrixXd Y = Z - (H_ * x_); 
	Eigen::MatrixXd Ht = H_.transpose();
	Eigen::MatrixXd S = H_ * P_ * Ht + R;
	Eigen::MatrixXd K = P_ * Ht * S.inverse();
	x_ = x_ + (K * Y);
	int size = x_.size();
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(size, size);
	P_ = (I - K * H_) * P_;
}

Eigen::VectorXd KalmanFilter::Get_x(){
	return x_;
}

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
public:
	//constructor
	KalmanFilter();

	//deconstructor
	~KalmanFilter();

 	/**
	 * \函数：Initialization
	 * \说明：初始化状态向量x
	 * \输入参数：x_in
	 * \输出参数：无
	 * \返回值：无
	 * \其他：无
	 */
	void Initialization(Eigen::VectorXd x_in);

 	/**
	 * \函数：IsInitialized
	 * \说明：判断kalman filter是否已经初始化
	 * \输入参数：无
	 * \输出参数：is_initialized_
	 * \返回值：无
	 * \其他：无
	 */
	bool IsInitialized();

 	/**
	 * \函数：SetU
	 * \说明：初始化外部控制向量u
	 * \输入参数：u_in
	 * \输出参数：无
	 * \返回值：无
	 * \其他：无
	 */
	void SetU(Eigen::VectorXd u_in);

 	/**
	 * \函数：SetF
	 * \说明：设定状态转移矩阵（state transistion matrix）F
	 * \输入参数：F_in
	 * \输出参数：无
	 * \返回值：无
	 * \其他：无
	 */
	void SetF(Eigen::MatrixXd F_in);

 	/**
	 * \函数：SetP
	 * \说明：设定状态协方差矩阵（state covariance matrix）P
	 * \输入参数：P_in
	 * \输出参数：无
	 * \返回值：无
	 * \其他：无
	 */
	void SetP(Eigen::MatrixXd P_in);

 	/**
	 * \函数：SetQ
	 * \说明：设定过程噪声（process covariance matrix）Q
	 * \输入参数：Q_in
	 * \输出参数：无
	 * \返回值：无
	 * \其他：无
	 */
	void SetQ(Eigen::MatrixXd Q_in);

 	/**
	 * \函数：SetH
	 * \说明：设定测量矩阵（Measurement Matrix)H
	 * \输入参数：H_in
	 * \输出参数：无
	 * \返回值：无
	 * \其他：无
	 */
	void SetH(Eigen::MatrixXd H_in);

 	/**
	 * \函数：SetR
	 * \说明：设定测量噪声矩阵（measurement covariance matrix）R
	 * \输入参数：R_in
	 * \输出参数：无
	 * \返回值：无
	 * \其他：无
	 */
	void SetR(Eigen::MatrixXd R_in);

 	/**
	 * \函数：prediction
	 * \说明：计算预测模块————计算预测状态向量x，计算状态协方差矩阵P
	 * \输入参数：无
	 * \输出参数：无
	 * \返回值：无
	 * \其他：无
	 */
	void prediction();

 	/**
	 * \函数：MeasurementUpdate
	 * \说明：计算观测模块————计算卡尔曼增益K，更新状态向量x,更新状态协方差矩阵P，滤波器闭环。
	 * \输入参数：Z
	 * \输出参数：无
	 * \返回值：无
	 * \其他：无
	 */
	void MeasurementUpdate(const Eigen::VectorXd &Z);

 	/**
	 * \函数：Get_X
	 * \说明：状态向量x的接口
	 * \输入参数：无
	 * \输出参数：
	 * \返回值：无
	 * \其他：无
	 */
	Eigen::VectorXd Get_x();

private:
	//flag of initialization
	bool is_initialized_;

	//state vector
	Eigen::VectorXd x_;	

	//control vector
	Eigen::VectorXd u_;

	//state transistion matrix
	Eigen::MatrixXd F_;

	//state covariance matrix
	Eigen::MatrixXd P_;

	//state process covariance matrix
	Eigen::MatrixXd Q_;

	//measurement covariance matrix
	Eigen::MatrixXd R;

	//Measurement Matrix
	Eigen::MatrixXd H_;
};

#endif
#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

const double PI = 3.14159;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
	x_ = F_*x_;
	P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd y = VectorXd(2);
	y = z - H_*x_;
	MatrixXd S = MatrixXd(2, 2);
	S = H_*P_*H_.transpose() + R_;
	MatrixXd K = MatrixXd(4, 2);
	K = P_*H_.transpose()*S.inverse();
	x_ = x_ + K*y;
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	float h1 = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
	float h2 = atan2(x_(1), x_(0));
	//std::cout << h2;

	if (fabs(h1)<0.0001)
	{
		h1 = 0.0001;
	}
	float h3 = (x_(0)*x_(2) + x_(1)*x_(3)) / h1;
	VectorXd h = VectorXd(3);
	h << h1, h2, h3;
	VectorXd y = VectorXd(3);
	y = z - h;
	while (y(1)>PI)
	{
		y(1) -= 2 * PI;
	}
	while (y(1)<-PI)
	{
		y(1) += 2 * PI;
	}
	MatrixXd S = MatrixXd(3, 3);
	S = H_*P_*H_.transpose() + R_;
	MatrixXd K = MatrixXd(4, 3);
	K = P_*H_.transpose()*S.inverse();
	x_ = x_ + K*y;
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K*H_)*P_;

}
// need to check the range of y between -pi and +pi.
// need to check dependency between H_Jacobian and H_ for UpdateEKF.
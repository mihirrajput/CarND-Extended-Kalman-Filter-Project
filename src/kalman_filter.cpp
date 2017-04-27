#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
	VectorXd y = z - H_*x_;
	MatrixXd S = H_*P_*H_.transpose() + R_;
	MatrixXd K = P_*H_.transpose()*S.inverse();
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
	if (fabs(h1)<0.001)
	{
		h1 = 0.001;
	}
	float h3 = (x_(0)*x_(2) + x_(1)*x_(3)) / h1;
	VectorXd h = VectorXd(4);
	h << h1, h2, h3;

	VectorXd y = z - h;
	MatrixXd S = H_*P_*H_.transpose() + R_;
	MatrixXd K = P_*H_.transpose()*S.inverse();
	x_ = x_ + K*y;
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K*H_)*P_;

}
// need to check the range of y between -pi and +pi.
// need to check dependency between H_Jacobian and H_ for UpdateEKF.
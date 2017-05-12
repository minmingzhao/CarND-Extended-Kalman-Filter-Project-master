#include "kalman_filter.h"
#include <cmath>
using Eigen::MatrixXd;
using Eigen::VectorXd;
#include <iostream>
using namespace std;

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
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	KF(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	VectorXd h = VectorXd(3);
	double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
	double phi = atan2(x_(1),x_(0));
	double rho_dot = (x_(0)*x_(2)+x_(1)*x_(3))/rho;
	h << rho,phi,rho_dot;
	VectorXd z_pred = h;
	VectorXd y = z - z_pred;
	while (y(1) >= M_PI) {
		y(1) = y(1) - 2*M_PI;
	}
	while (y(1) < -M_PI) {
			y(1) = y(1) + 2*M_PI;
		}
	KF(y);
}

// Universal update Kalman Filter step. Equations from the lectures
void KalmanFilter::KF(const VectorXd &y){
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;
  // New state
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

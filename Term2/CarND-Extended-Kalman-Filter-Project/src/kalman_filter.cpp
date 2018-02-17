#include <math.h>
#include "kalman_filter.h"
const float TWO_PI = 2 * M_PI;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

VectorXd CartesianToPolar(const VectorXd &x_state) {
	float px, py, vx, vy;
	px = x_state[0];
	py = x_state[1];
	vx = x_state[2];
	vy = x_state[3];

	if (fabs(px) < 0.0001)
		px = 0.0001;


	float rho, phi, rho_dot;
	rho = sqrt(px*px + py * py);
	phi = atan2(py, px);


	if (fabs(rho) < 0.000001)
		rho = 0.000001;

	rho_dot = (px * vx + py * vy) / rho;

	VectorXd h_x = VectorXd(3);
	h_x << rho, phi, rho_dot;

	return h_x;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	VectorXd z_pred = CartesianToPolar(x_);
	VectorXd y = z - z_pred;

	NormalizeAngles(y);
	

	// following is exact the same as in the function of KalmanFilter::Update()
	MatrixXd Ht = H_.transpose();
	MatrixXd PHt = P_ * Ht;
	MatrixXd S = H_ * PHt + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void NormalizeAngles(Eigen::VectorXd &y)
{
	if (y[1] > M_PI)
		y[1] -= TWO_PI;
	else if (y[1] < -M_PI)
		y[1] += TWO_PI;
}


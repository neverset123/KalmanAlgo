#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "Eigen/Dense"

class KalmanFilter
{
public: 
	KalmanFilter()
	{
		is_initialized_=false;
	};

	//~KalmanFilter(){};

	Eigen:: VectorXd GetX()
	{
		return x_;
	}

	bool IsInitialized()
	{
		return is_initialized_;
	}

	void Initialization(Eigen::VectorXd x_in)
	{
		x_=x_in;
		is_initialized_=true;
	}

	void SetF(Eigen::MatrixXd F_in)
	{
		F_=F_in;
	};

	void SetP(Eigen::MatrixXd P_in)
	{

		P_=P_in;
	};
	void SetQ(Eigen::MatrixXd Q_in)
	{

		Q_=Q_in;
	};
	void SetH(Eigen::MatrixXd H_in)
	{

		H_=H_in;
	};
	void SetR(Eigen::MatrixXd R_in)
	{

		R_=R_in;
	};

	void Prediction()
	{
		x_=F_*x_;
		Eigen::MatrixXd Ft=F_.transpose();
		P_=F_*P_*Ft+Q_;
	};
	void MeasurementUpdate(const Eigen::VectorXd &z)
	{
		Eigen::VectorXd y=z-H_*x_;
		Eigen::MatrixXd S=H_*P_*H_.transpose()+R_;
		Eigen:: MatrixXd K=P_*H_.transpose()*S.inverse();
		x_=x_+K*y;
		int size =x_.size();
		Eigen::MatrixXd I=z.Identity(size, size);
		P_=(I-K*H_)*P_;
	};



private:
	bool is_initialized_;
	Eigen::VectorXd x_;
	Eigen::MatrixXd F_;
	Eigen::MatrixXd P_;
	Eigen::MatrixXd Q_;
	Eigen::MatrixXd H_;
	Eigen::MatrixXd R_;
		
};

class ExtendedKalmanFilter
{
public:
	ExtendedKalmanFilter()
	{
		is_initialized_=false;

	};

	~ExtendedKalmanFilter(){};
	void initialization(Eigen::VectorXd x_in)
	{
		x_=x_in;
	};

	void SetF(Eigen::MatrixXd F_in)
	{
		F_=F_in;
	};

	void SetP(Eigen::MatrixXd P_in)
	{

		P_=P_in;
	};
	void SetQ(Eigen::MatrixXd Q_in)
	{

		Q_=Q_in;
	};
	void SetR(Eigen::MatrixXd R_in)
	{

		R_=R_in;
	};
	void Prediction()
	{
		x_=F_*x_;
		Eigen::MatrixXd Ft=F_.transpose();
		P_=F_*P_*Ft+Q_;
	};

	void CalculateJacobianMatrix()
	{
		Eigen::MatrixXd Hj(3,4);
		float px=x_(0);
		float py=x_(1);
		float vx=x_(2);
		float vy=x_(3);
		
		float c1=px*px+py*py;
		float c2=sqrt(c1);
		float c3=(c1*c2);

		if(fabs(c1)<0.0001)
		{
			H_=Hj;
			return;
		}

		Hj<<(px/c2), (py/c2), 0,0,
			-(py/c1), (px/c1), 0,0,
			py*(vx*py-vy*px)/c3, px*(px*vy-py*vx)/c3, px/c2, py/c2;
		H_=Hj;
		return;

	};

	void MeasurementUpdate(const Eigen::VectorXd &z)
	{
		double rho=sqrt(x_(0)*x_(0)+x_(1)*x_(1));
		double theta=atan2(x_(1), x_(0));
		double rho_dot=(x_(0)*x_(2)+x_(1)*x_(3))/rho;
		Eigen::VectorXd h=Eigen::VectorXd(3);
		h<<rho, theta, rho_dot;
		Eigen::VectorXd y=z-h;
		CalculateJacobianMatrix();
		Eigen::MatrixXd S=H_*P_*H_.transpose();
		Eigen::MatrixXd K=P_*H_.transpose()*S.inverse();
		x_=x_+K*y;
		Eigen::MatrixXd I=z.Identity(x_.size(), x_.size());
		P_=(I-K*H_)*P_;
	};

private:
	bool is_initialized_;
	Eigen::VectorXd x_;
	Eigen::MatrixXd F_;
	Eigen::MatrixXd P_;
	Eigen::MatrixXd Q_;
	Eigen::MatrixXd H_;
	Eigen::MatrixXd R_;

};

#endif
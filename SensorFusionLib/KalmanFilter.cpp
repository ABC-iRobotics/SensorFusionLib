#include "KalmanFilter.h"



KalmanFilter::KalmanFilter(BaseSystemData data, StatisticValue state_) :
	SystemManager(data, state_) {}


KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::Step(double dT) { // update, collect measurement, correction via Kalman-filtering
	Eigen::MatrixXd sg1, sg2;
	StatisticValue x_pred = Eval(System::TIMEUPDATE, dT, (*this)(STATE), (*this)(DISTURBANCE), sg1, sg2);
	Eigen::MatrixXd Syxpred;
	Eigen::VectorXd y_meas = (*this)(OUTPUT).vector;
	StatisticValue y_pred = Eval(System::MEASUREMENTUPDATE, dT, x_pred, (*this)(NOISE), Syxpred, sg2);

	StepClock(dT);
	PredictionDone(x_pred, y_pred);

	Eigen::MatrixXd K = Syxpred.transpose() * y_pred.variance.inverse();
	Eigen::MatrixXd Sxnew = x_pred.variance - K * Syxpred;
	StatisticValue newstate = StatisticValue(x_pred.vector + K * (y_meas - y_pred.vector),
		(Sxnew + Sxnew.transpose()) / 2.);

	FilteringDone(newstate);

	State() = newstate;
	resetMeasurement();
}

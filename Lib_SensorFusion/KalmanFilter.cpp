#include "KalmanFilter.h"



KalmanFilter::KalmanFilter(BaseSystemData data, StatisticValue state_) :
	SystemManager(data, state_) {}


KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::Step(TimeMicroSec dT) { // update, collect measurement, correction via Kalman-filtering
	Eigen::MatrixXd sg1, sg2;
	StatisticValue x_pred = Eval(STATE_UPDATE, dT.TimeInS(), (*this)(STATE), (*this)(DISTURBANCE), sg1, sg2);
	Eigen::MatrixXd Syxpred;
	StatisticValue y_meas = (*this)(OUTPUT);
	StatisticValue y_pred = Eval(OUTPUT_UPDATE, dT.TimeInS(), x_pred, (*this)(NOISE), Syxpred, sg2);

	StepClock(dT);
	PredictionDone(x_pred, y_pred);

	Eigen::MatrixXd Syy = y_pred.variance + y_meas.variance;

	Eigen::MatrixXd K = Syxpred.transpose() * Syy.inverse();
	Eigen::MatrixXd Sxnew = x_pred.variance - K * Syxpred;
	StatisticValue newstate = StatisticValue(x_pred.vector + K * (y_meas.vector - y_pred.vector),
		(Sxnew + Sxnew.transpose()) / 2.);

	FilteringDone(newstate);

	State() = newstate;
	resetMeasurement();
}

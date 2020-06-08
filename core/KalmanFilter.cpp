#include "KalmanFilter.h"

using namespace SF;

KalmanFilter::KalmanFilter(BaseSystemData data, StatisticValue state_, const Time& currentTime) :
	SystemManager(data, state_), lastStepTime(currentTime){}


KalmanFilter::~KalmanFilter()
{
}

using namespace SF;

void KalmanFilter::Step(const DTime& dT) { // update, collect measurement, correction via Kalman-filtering
	double dT_sec = duration_cast_to_sec(dT);
	Eigen::MatrixXd sg1, sg2;
	StatisticValue x_pred = Eval(STATE_UPDATE, dT_sec, (*this)(STATE), (*this)(DISTURBANCE), sg1, sg2);
	Eigen::MatrixXd Syxpred;
	StatisticValue y_meas = (*this)(OUTPUT);
	StatisticValue y_pred = Eval(OUTPUT_UPDATE, dT_sec, x_pred, (*this)(NOISE), Syxpred, sg2);

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

void SF::KalmanFilter::SamplingTimeOver(const Time & currentTime) {
	Step(duration_cast(currentTime - lastStepTime));
}

void SF::KalmanFilter::MsgQueueEmpty(const Time & currentTime) {
	Step(duration_cast(currentTime - lastStepTime));
}

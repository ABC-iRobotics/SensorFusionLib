#include "KalmanFilter.h"

using namespace SF;

KalmanFilter::KalmanFilter(BaseSystemData data, StatisticValue state_) :
	SystemManager(data, state_) {}


KalmanFilter::~KalmanFilter()
{
}

using namespace SF;

void KalmanFilter::Step(const DTime& dT) { // update, collect measurement, correction via Kalman-filtering
	double dT_sec = duration_cast_to_sec(dT);
	Eigen::MatrixXd sg1, sg2;
	StatisticValue x_pred = Eval(STATE_UPDATE, dT_sec, (*this)(STATE), (*this)(DISTURBANCE), sg1, sg2);
	// Offset the rad state variables into the allowed +-pi interval
	{
		auto isStateRad_ = isStateRad();
		Eigen::VectorXd& x_pred_v = x_pred.vector;
		for (int i = 0; i < isStateRad_.size(); i++)
			if (isStateRad_(i) == 1) {
				x_pred_v(i) = fmod(x_pred_v(i), 2. * EIGEN_PI);
				if (x_pred_v(i) < -EIGEN_PI)
					x_pred_v(i) += EIGEN_PI * 2.;
				if (x_pred_v(i) > EIGEN_PI)
					x_pred_v(i) -= EIGEN_PI * 2.;
			}
	}
	Eigen::MatrixXd Syxpred;
	StatisticValue y_meas = (*this)(OUTPUT);
	StatisticValue y_pred = Eval(OUTPUT_UPDATE, dT_sec, x_pred, (*this)(NOISE), Syxpred, sg2);

	PredictionDone(x_pred, y_pred);
	StatisticValue newstate = x_pred;
	if (y_pred.Length() > 0) {
		Eigen::MatrixXd Syy = y_pred.variance + y_meas.variance;

		Eigen::MatrixXd K = Syxpred.transpose() * Syy.inverse();
		Eigen::MatrixXd Sxnew = (x_pred.variance - K * Syxpred);
		Eigen::VectorXd ydiff = y_meas.vector - y_pred.vector;

		auto isyrad = isOutputRad(false);
		for (int i=0;i<isyrad.size();i++)
			if (isyrad(i)==1) {
				ydiff(i) = fmod(ydiff(i), 2. * EIGEN_PI);
				if (ydiff(i) < -EIGEN_PI)
					ydiff(i) += EIGEN_PI * 2.;
				if (ydiff(i) > EIGEN_PI)
					ydiff(i) -= EIGEN_PI * 2.;
			}
		newstate = StatisticValue(x_pred.vector + K * ydiff,
			(Sxnew + Sxnew.transpose()) / 2.);
	}
	FilteringDone(newstate);

	State() = newstate;
	resetMeasurement();
}

void SF::KalmanFilter::SamplingTimeOver(const Time & currentTime) {
	if (firstStep) {
		firstStep = false;
		lastStepTime = currentTime;
	}
	Step(duration_cast(currentTime - lastStepTime));
	lastStepTime = currentTime;
}

void SF::KalmanFilter::MsgQueueEmpty(const Time & currentTime) {
	if (firstStep) {
		firstStep = false;
		lastStepTime = currentTime;
	}
	Step(duration_cast(currentTime - lastStepTime));
	lastStepTime = currentTime;
}

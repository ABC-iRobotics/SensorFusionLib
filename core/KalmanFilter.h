#pragma once
#include "SystemManager.h"

namespace SF {

	/*! \brief Class to perform Kalman-filtering on complex asynchronous multisensor systems
	*
	* The functionality of SystemManager superclass wraps the models into a simple state model
	*
	* \f$ \mathbf x_k = \mathbf A\cdot \mathbf x_{k-1} + \mathbf B
	 \cdot \mathbf w_k + \mathbf f( \mathbf x_{k-1}, \mathbf w_k) \f$
	*
	* \f$ \mathbf y_k = \mathbf C\cdot \mathbf x_{k} + \mathbf D
	 \cdot \mathbf v_k + \mathbf g( \mathbf x_{k}, \mathbf v_k) \f$
	* - according to the current status of the sensors. (For the details, see the description of class SystemManager)
	*
	* This subclass calls the Time Update and the implements the Kalman-filtering within the Step() function as
	*
	* - 0. \f$ \hat{\mathbf x}_{k-1} \f$ and its covariance matrix \f$\hat{\Sigma}_{k-1} \f$ are initial guess or comes from the previous filtering.
	* - 1. (During the elapsed \f$ dT \f$ time the recieved sensor measurements was registered (CallbackGotDataMsg())
	*    Time-update: step the time with \f$ dT \f$
	*
	* State update:
	*
	* \f$ \overline{\mathbf x}_k = \mathbf A(dT) \cdot \hat{\mathbf x}_{k-1} + \mathbf B(dT)
	 \cdot \mathbf w_k + \mathbf f(dT, \hat{\mathbf x}_{k-1}, \mathbf w_k) \f$
	 * its covariance matrix as \f$ \overline{\Sigma}_k\f$
	*
	* Output update (to the available sensors):
	*
	* \f$ \overline{\mathbf y}_k = \mathbf C(dT)\cdot \overline{\mathbf x}_{k} + \mathbf D(dT)
	 \cdot \mathbf v_k + \mathbf g(dT, \overline{\mathbf x}_{k}, \mathbf v_k) \f$
	 * its covariance matrices as \f$ \overline{\Sigma}_{yy,k}\f$, \f$ \overline{\Sigma}_{yx,k}\f$
	*
	* - 2. Measurement update (or filtering):
	*
	* \f$ \hat{\mathbf x}_k = \overline{\mathbf x}_k + \mathbf K (\mathbf y_{meas,k} - \overline{\mathbf y}_k) \f$ its covariance matrix as \f$ \hat{\Sigma}_k\f$
	*
	* where \f$ \mathbf K = \overline{\Sigma}_{xy,k}\left(\overline{\Sigma}_{yy,k} + \Sigma_{yy,meas,k} \right)^{-1} \f$
	*/
	class KalmanFilter : public SystemManager {
		Time lastStepTime;

	public:
		KalmanFilter(BaseSystemData data, StatisticValue state_, const Time& t0 = Now()); /*!< Constructor. */

		~KalmanFilter();

		/*! \brief Time update with the given dT and Kalman-filter based on the available sensors
		*
		* Steps the last filtered state with \f$dT \f$ by applying the time update model, and performs Kalman-filtering, see description of class KalmanFilter for more details.
		*/
		void Step(const DTime& dT);

		void SamplingTimeOver(const Time& currentTime) override; /*!< Is called in each sampling time - input: time */

		void MsgQueueEmpty(const Time& currentTime) override; /*!< Is called if the DataMsgs in the queue were read */

		typedef std::shared_ptr<KalmanFilter> KalmanFilterPtr; /*!< Shared pointer type for the KalmanFilter class */
	};

}

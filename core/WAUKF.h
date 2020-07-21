#include "SystemManager.h"
#include "Eigen/Dense"
#include <map>

namespace SF {

	const unsigned int MAX_WINDOW_SIZE = 200;

	enum ValueType { VALUE, VARIANCE };

	/*! \brief Moving average window implementation
	*
	*  Type can be Eigen::VectorXd or Eigen::MatrixXd
	*/
	template<class Type>
	class MAWindow {
		Type data[MAX_WINDOW_SIZE]; /*!< Buffer*/
		Type out; /*!< Last result*/
		bool upToDate; /*!< Last result is uptodate*/
		bool initialized; /*!< If it is initialized*/
		unsigned int lastWritten; /*!< Index of the latest element */
		unsigned int windowSize; /*!< Used windowsize */
	public:
		MAWindow(unsigned int windowSize_, const Type& initValue); /*!< Constructor with initialization */
		MAWindow(unsigned int windowSize_ = 100);  /*!< Constructor without initialization */
		const Type& Value();  /*!< Get the actual result */
		void AddValue(const Type& value);  /*!< Add a new value */
	};

	/*! \brief Class to perform Kalman-filtering with windowing based parameter estimation on complex asynchronous multisensor systems
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
	*
	* - 3. Parameter estimation
	*
	* Disturbance/noise, value/variance estimation for the chosen systems based on moving average windowing.
	*
	* For more details, see:
	*
	* Gao, Shesheng, Gaoge Hu, and Yongmin Zhong. "Windowing and random weighting‐based adaptive unscented Kalman filter." International Journal of Adaptive Control and Signal Processing 29.2 (2015): 201-223.
	*/
	class WAUKF : public SystemManager {
		Time lastStepTime;
		bool firstStep = true;

	public:
		WAUKF(const BaseSystemData& data, const StatisticValue& state_); /*!< Constructor. */

		void SetDisturbanceValueWindowing(System::SystemPtr ptr, unsigned int windowSize);
		/*!< Add a window to estimate \f$ \mathbf w_i\f$ disturbance value for a given System (= BaseSystem or Sensor) with given window size.*/

		void SetNoiseValueWindowing(System::SystemPtr ptr, unsigned int windowSize);
		/*!< Add a window to estimate \f$ \mathbf v_i\f$ noise value for a given System (= BaseSystem or Sensor) with given window size.*/

		void SetDisturbanceVarianceWindowing(System::SystemPtr ptr, unsigned int windowSize);
		/*!< Add a window to estimate \f$ \Sigma_{ww}\f$ disturbance variance for a given System (= BaseSystem or Sensor) with given window size.*/

		void SetNoiseVarianceWindowing(System::SystemPtr ptr, unsigned int windowSize);
		/*!< Add a window to estimate \f$ \Sigma_{vv}\f$ noise variance for a given System (= BaseSystem or Sensor) with given window size.*/

		typedef std::shared_ptr<WAUKF> WAUKFPtr; /*!< Shared pointer type for the WAUKF class */

		/*! \brief Time update with the given dT and Kalman-filter based on the available sensors
	*
	* Steps the last filtered state with \f$dT \f$ by applying the time update model, and performs Kalman-filtering, see description of class KalmanFilter for more details.
	*/
	// TODO: perform further tests on the adaptive methods
		void Step(const DTime& dT);

		void SamplingTimeOver(const Time& currentTime) override; /*!< Is called in each sampling time - input: time */

		void MsgQueueEmpty(const Time& currentTime) override; /*!< Is called if the DataMsgs in the queue were read */

		/*! \brief The function to inject data (meas. results, noise, disturbance value and/or variances)
		*
		* Omitted if the value is estimated by the adaptive method.
		*/
		void SaveDataMsg(const DataMsg& data, const Time&) override;

	private:
		typedef std::map<unsigned int, MAWindow<Eigen::VectorXd>> mapOfVectorWindows;
		typedef std::map<unsigned int, MAWindow<Eigen::MatrixXd>> mapOfMatrixWindows;

		mapOfVectorWindows noiseValueWindows;
		mapOfVectorWindows disturbanceValueWindows;
		mapOfMatrixWindows noiseVarianceWindows;
		mapOfMatrixWindows disturbanceVarianceWindows;

		const mapOfVectorWindows& _getValueWindows(DataType signal) const;
		const mapOfMatrixWindows& _getVarianceWindows(DataType signal) const;
		bool _isEstimated(unsigned int systemID, DataType signal, ValueType type) const;

		StatisticValue _evalWithV0(TimeUpdateType outType, double Ts, const StatisticValue& state_,
			const StatisticValue& in, Eigen::MatrixXd & S_out_x, Eigen::MatrixXd& S_out_in,
			bool forcedOutput, StatisticValue& v0) const;
	};

	template<class Type>
	inline MAWindow<Type>::MAWindow(unsigned int windowSize_, const Type & initValue) :
		windowSize(windowSize_), initialized(true), lastWritten(0) {
		if (windowSize > MAX_WINDOW_SIZE)
			throw std::runtime_error(std::string("MAWindow::MAWindow Wrong window size to be applied!"));
		out = initValue;
		upToDate = true;
		initialized = true;
		for (unsigned int n = 0; n < MAX_WINDOW_SIZE; n++)
			data[n] = initValue;
	}

	template<class Type>
	inline MAWindow<Type>::MAWindow(unsigned int windowSize_) :
		windowSize(windowSize_), initialized(false), lastWritten(0), upToDate(false) {}

	template<class Type>
	inline const Type & MAWindow<Type>::Value() {
		if (!initialized)
			throw std::runtime_error(std::string("MAWindow::Value Getter cannot be applied before initialization!"));
		if (!upToDate) {
			out = out * 0;
			int starter = lastWritten;
			for (int n = 0; n < (int)windowSize; n++) {
				if (starter - n < 0)
					starter += MAX_WINDOW_SIZE;
				out += data[starter - n];
			}
			out /= windowSize;
			upToDate = true;
		}
		return out;
	}

	template<class Type>
	inline void MAWindow<Type>::AddValue(const Type & value) {
		if (initialized) {
			if (windowSize > 0) {
				lastWritten++;
				if (lastWritten >= MAX_WINDOW_SIZE)
					lastWritten = 0;
				data[lastWritten] = value;
				upToDate = false;
			}
			else {
				out = value;
				upToDate = true;
			}
		}
		else {
			out = value;
			upToDate = true;
			initialized = true;
			for (unsigned int n = 0; n < MAX_WINDOW_SIZE; n++)
				data[n] = value;
		}
	}

}

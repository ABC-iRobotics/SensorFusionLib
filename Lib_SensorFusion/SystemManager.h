#pragma once

#include "Sensor.h"
#include "DataMsg.h"

/*! \brief The class provides functions to build complex asynchronous multisensor models and wrap them into a simple nonlinear model
*
* The \f$ \mathbf x \f$ state vector, \f$ \mathbf y \f$ output vector,
*  \f$ \mathbf w \f$ disturbance vector and \f$ \mathbf v \f$ noise vector are used according to the time update model, see the following description
*
* ----------------------------------------------------------
* Time update model: state update equation
*
* \f$ \mathbf x_k = \mathbf A\cdot \mathbf x_{k-1} + \mathbf B
 \cdot \mathbf w_k + \mathbf f( \mathbf x_{k-1}, \mathbf w_k) \f$
*
* where
*  - the state vector: \f$ \mathbf x = \begin{bmatrix}	\mathbf x_{bs} \\ \mathbf x_{s1} \\ \mathbf x_{s2} \\ 	\vdots \\ \mathbf x_{sn}	\end{bmatrix} \f$
*  - the disturbance vector: 
*  \f$ \mathbf w = \begin{bmatrix}
*	\mathbf w_{bs} \\ \mathbf w_{s1} \\ \mathbf w_{s2} \\ \vdots \\ \mathbf w_{sn}
*	\end{bmatrix}\f$
*  - \f$ \mathbf f( \mathbf x_{k-1}, \mathbf w_k) \f$ denotes the nonlinear part. 
*    The 'dep(STATE_UPDATE,VAR_STATE)' returns if it depends on the elements of \f$ \mathbf x \f$, 
*    the 'dep(STATE_UPDATE,VAR_EXTERNAL)' returns if it depends on the elements of \f$ \mathbf w \f$
*  - the system matrices: \f$ \mathbf A = \begin{bmatrix}
	\mathbf A_{bs} & \mathbf 0 & \mathbf 0 & \dots & \mathbf 0 \\
	\mathbf A'_{s1} & \mathbf A_{s1} & \mathbf 0 & \dots & \mathbf 0 \\
	\mathbf A'_{s2} & \mathbf 0 & \mathbf A_{s2}  & \dots & \mathbf 0 \\
	\vdots & \vdots & \vdots &  & \vdots \\
	\mathbf A'_{sn} & \mathbf 0 & \mathbf 0  & \dots & \mathbf A_{sn} \\
	\end{bmatrix}\f$, \f$ \mathbf B =  \begin{bmatrix}
	\mathbf B_{bs} & \mathbf 0 & \mathbf 0 & \dots & \mathbf 0 \\
	\mathbf B'_{s1} & \mathbf B_{s1} & \mathbf 0 & \dots & \mathbf 0 \\
	\mathbf B'_{s2} & \mathbf 0 & \mathbf B_{s2}  & \dots & \mathbf 0 \\
	\vdots & \vdots & \vdots &  & \vdots \\
	\mathbf B'_{sn} & \mathbf 0 & \mathbf 0  & \dots & \mathbf B_{sn} \\
	\end{bmatrix}\f$
*
* ----------------------------------------------------------
* Time update model: output equation
*
* \f$ \mathbf y_k = \mathbf C\cdot \mathbf x_{k} + \mathbf D
 \cdot \mathbf v_k + \mathbf g( \mathbf x_{k}, \mathbf v_k) \f$
*
* where
*  - the output vector: \f$ \mathbf y = \begin{bmatrix}
	\mathbf y_{bs} \\ \mathbf y_{s1} \\ \mathbf y_{s2} \\ \vdots \\ \mathbf y_{sn}
	\end{bmatrix} \f$,  where \f$ \mathbf y_i = [] \f$ if the basesystem / i-th sensor it is not uptodate
*  - the noise vector: \f$ \begin{bmatrix}
	\mathbf v_{bs} \\ \mathbf v_{s1} \\ \mathbf v_{s2} \\ \vdots \\ \mathbf v_{sn}
	\end{bmatrix} \f$, where \f$ \mathbf v_{si} = [] \f$ if the i-th sensor it is not uptodate
*  - \f$ \mathbf g( \mathbf x_{k}, \mathbf v_k) \f$ denotes the nonlinear part. 
*    The 'dep(OUTPUT_UPDATE,VAR_STATE)' returns if it depends on the elements of \f$ \mathbf x \f$, 
*    the 'dep(OUTPUT_UPDATE,VAR_EXTERNAL)' returns if it depends on the elements of \f$ \mathbf v \f$
*  - the system matrices: \f$ \mathbf C = \begin{bmatrix}
	\mathbf C_{bs} & \mathbf 0 & \mathbf 0 & \dots & \mathbf 0 \\
	\mathbf C'_{s1} & \mathbf C_{s1} & \mathbf 0 & \dots & \mathbf 0 \\
	\mathbf C'_{s2} & \mathbf 0 & \mathbf C_{s2}  & \dots & \mathbf 0 \\
	\vdots & \vdots & \vdots &  & \vdots \\
	\mathbf C'_{sn} & \mathbf 0 & \mathbf 0  & \dots & \mathbf C_{sn} \\
	\end{bmatrix}\f$, \f$ \mathbf D =  \begin{bmatrix}
	\mathbf D_{bs} & \mathbf 0 & \mathbf 0 & \dots & \mathbf 0 \\
	\mathbf D'_{s1} & \mathbf D_{s1} & \mathbf 0 & \dots & \mathbf 0 \\
	\mathbf D'_{s2} & \mathbf 0 & \mathbf D_{s2}  & \dots & \mathbf 0 \\
	\vdots & \vdots & \vdots &  & \vdots \\
	\mathbf D'_{sn} & \mathbf 0 & \mathbf 0  & \dots & \mathbf D_{sn} \\
	\end{bmatrix} \f$
* where
* the block matrices of C, D  that corresponds to basesystem/sensor with obsolethe measurement has row-size 0,
* the block matrices of D that corresponds to a sensor with obsolethe measurement has column-size 0,
*
* ----------------------------------------------------------
* Usage:
*
* - Define a subclass by implementing filtering and time update into overridden function Step().
* - Build the system:
*
* -- Constructor: Initiate with a BaseSystemData instance and its state
*
* -- AddSensors: provide a SensorData instance and its state
*
* - Call:
*
* -- SetProperty() with data incoming from sensors
*
* -- Step(Ts) for filtering
* - Get the results or register a Callback via SetCallback()
 */
class SystemManager {
public:
/*! \brief If constant value is guessed as the \f$ \mathbf y_i \f$ output of a system, its status is CONSTANT, othervise OBSOLETHE / UPTODATE.
*
*
*/
	enum MeasurementStatus { OBSOLETHE, UPTODATE, CONSTANT }; /*!<   */

/*! \brief The class handles the common properties of basesystems and sensors
*
*/
	class SystemData {
		MeasurementStatus measStatus;/*!< Status of the set measurement value */
		StatisticValue noise;		/*!< The known value & uncertaintie of the noise  */
		StatisticValue disturbance;	/*!< The known value & uncertaintie of the disturbance */
		StatisticValue measurement; /*!< The known value & uncertaintie of the measurement */
	protected:
		SystemData(const StatisticValue& noise_, const StatisticValue& disturbance_, unsigned int outputSize);
					/*!< Constructor without providing measurement result */
		SystemData(const StatisticValue& noise_, const StatisticValue& disturbance_,
			const StatisticValue& measurement_, MeasurementStatus measStatus_); /*!< Constructor. */
	public:
		virtual System::SystemPtr getPtr() const = 0;  /*!< Virtual SystemPtr getter. */
		StatisticValue operator()(DataType type, bool forcedOutput = false) const;  /*!< Returns the state, output, noise or disturbance of the system . */
		size_t num(DataType type, bool forcedOutput = false) const;
		 /*!< Returns the number of states, outputs, noises or disturbances of the system . */
		void setValue(const Eigen::VectorXd& value, DataType type);
		 /*!< Set the state/output/disturbance/noise vector. */ 
		void setVariance(const Eigen::MatrixXd& value, DataType type);
		 /*!< Set the state/output/disturbance/noise covariance matrix. */ 
		Eigen::VectorXd getValue(DataType type) const;  /*!< Returns the state/output/disturbance/noise vector. */
		Eigen::MatrixXd getVariance(DataType type) const;  /*!< Returns the state/output/disturbance/noise covariance matrix. */
		void resetMeasurement(); /*!< Set the measurement status OBSOLETHE from UPTODATE. */ 
		bool available() const;  /*!< Returns true if there is an available measurement result*/
		virtual bool isBaseSystem() const = 0;  /*!< To check if it is for a basesystem or a sensor. */
	};

/*! \brief The class handles the properties of basesystems
*
*/
	class BaseSystemData : public SystemData {
		BaseSystem::BaseSystemPtr ptr; /*!< Stores the shared pointer to the BaseSystem instance*/ 
	public:
		BaseSystemData(BaseSystem::BaseSystemPtr ptr_, const StatisticValue& noise_,
			const StatisticValue& disturbance_); /*!< Constructor without providing measurement result */
		BaseSystemData(BaseSystem::BaseSystemPtr ptr_, const StatisticValue& noise_,
			const StatisticValue& disturbance_, const StatisticValue& measurement_ ,
			MeasurementStatus measStatus_); /*!< Constructor. */
		Eigen::VectorXi dep(TimeUpdateType outType, VariableType type,
			bool forcedOutput = false) const; /*!< Returns if the nonlinear part of STATE_UPDATE/OUTPUT_UPDATE depends on the elements of STATE or DISTURBANCE/NOISE values */
		Eigen::MatrixXd getMatrix(double Ts, TimeUpdateType type,
			VariableType inType, bool forcedOutput = false) const; /*!< Returns \f$\mathbf A_{bs} \f$, \f$\mathbf B_{bs} \f$, \f$\mathbf C_{bs} \f$, \f$\mathbf D_{bs} \f$ matrices  according to the measurement status and the forcedoutput flag */
		BaseSystem::BaseSystemPtr getBaseSystemPtr() const; /*!< BaseSystemPtr getter. */
		System::SystemPtr getPtr() const override;  /*!< SystemPtr getter. */
		bool isBaseSystem() const override; /*!< To check if it is for a basesystem or a sensor. */
	};

/*! \brief The class handles the properties of sensors
*
*/
	class SensorData : public SystemData {
		Sensor::SensorPtr ptr; /*!< Stores the shared pointer to the Sensor instance*/ 
	public:
		SensorData(Sensor::SensorPtr ptr_, const StatisticValue& noise_,
			const StatisticValue& disturbance_); /*!< Constructor without providing measurement result */
		SensorData(Sensor::SensorPtr ptr_, const StatisticValue& noise_,
			const StatisticValue& disturbance_, const StatisticValue& measurement_,
			MeasurementStatus measStatus_); /*!< Constructor. */
		Eigen::VectorXi depSensor(TimeUpdateType outType, VariableType type,
			bool forcedOutput = false) const; /*!< Returns if the nonlinear part of STATE_UPDATE/OUTPUT_UPDATE depends on the elements of sensor STATE or DISTURBANCE/NOISE values */
		Eigen::VectorXi depBaseSystem(TimeUpdateType outType, VariableType type,
			bool forcedOutput = false) const; /*!< Returns if the nonlinear part of STATE_UPDATE/OUTPUT_UPDATE depends on the elements of basesystem STATE or DISTURBANCE/NOISE values */
		Eigen::MatrixXd getMatrixBaseSystem(double Ts, TimeUpdateType type,
			VariableType inType, bool forcedOutput = false) const; /*!< Returns \f$\mathbf A'_{si} \f$, \f$\mathbf B'_{si} \f$, \f$\mathbf C'_{si} \f$, \f$\mathbf D'_{si} \f$ matrices  according to the measurement status and the forcedoutput flag */
		Eigen::MatrixXd getMatrixSensor(double Ts, TimeUpdateType type, VariableType inType,
			bool forcedOutput = false) const;  /*!< Returns \f$\mathbf A_{si} \f$, \f$\mathbf B_{si} \f$, \f$\mathbf C_{si} \f$, \f$\mathbf D_{si} \f$ matrices  according to the measurement status and the forcedoutput flag */
		Sensor::SensorPtr getSensorPtr() const; /*!< SensorPtr getter. */
		System::SystemPtr getPtr() const override; /*!< SystemPtr getter. */
		bool isBaseSystem() const override; /*!< To check if it is for a basesystem or a sensor. */
	};


	/*! \brief Constructor: a BaseSystemData class and its initial state, covariance matrix  must be given as inputs
*
*  The function performs System::Selftest on the system
*/ 
	SystemManager(const BaseSystemData& data, const StatisticValue& state_);

	/*! \brief Destructor.
	*
	*/ 
	~SystemManager();

	typedef std::shared_ptr<SystemManager> SystemManagerPtr; /*!< Shared pointer type for SystemManager class */

	/*! \brief Add a sensor: a SensorData class and its initial state, covariance matrix  must be given as inputs
	*
	* The function performs System::Selftest on the sensor, and check its compatibility to the basesystem
	*/
	void AddSensor(const SensorData& sensorData, const StatisticValue& sensorState);

/*! \brief Virtual function for filtering function including time update & measurement update
	*
	* It must call SystemManager::StepClock, SystemManager::PredictionDone, SystemManager::FilteringDone protected functions
	*/
	virtual void Step(TimeMicroSec Ts) = 0;

/*! \brief The function to inject data (meas. results, noise, disturbance value and/or variances)
	*
	*/
	virtual void SetProperty(const DataMsg& data);

	typedef std::function<void(const DataMsg& data)> Callback; /*!< Callback that can be set to handle results of time update & filtering update */

	void SetCallback(Callback callback_); /*!< Set callback that is called during time update & filtering update */

	void ClearCallback(); /*!< Remove callback that is called during time update & filtering update */

	size_t nSensors() const; /*!< Get number of sensor installed */

	const SystemData* SystemByID(unsigned int ID) const;  /*!< Get const pointer for a SystemData by ID */

/*! \brief Get the current number of states, outputs, noises or disturbances of the system 
	*
	* By using forcedOutput=true input, it assumes UPTODATE measurements
	*/
	size_t num(DataType type, bool forcedOutput = false) const;

/*! \brief Get the if the nonlinear part of STATE_UPDATE / OUTPUT_UPDATE depends on the elements of
* states, outputs, noises or disturbances of the system according to the measurement statuses of the systems
	*
	* By using forcedOutput=true input, it assumes UPTODATE measurements
	*/
	Eigen::VectorXi dep(TimeUpdateType outType, VariableType inType, bool forcedOutput = false) const;

/*! \brief Get the A,B, C,D matrices of the system according to the measurement statuses of the systems
	*
	* By using forcedOutput=true input, it assumes UPTODATE measurements
	*/
	void getMatrices(TimeUpdateType out_, double Ts, Eigen::MatrixXd& A,
		Eigen::MatrixXd& B, bool forcedOutput = false) const;

/*! \brief Get STATE, DISTURBANCE, measured OUTPUT, NOISE vectors of the system according to the measurement statuses of the systems
	*
	* By using forcedOutput=true input, it assumes UPTODATE measurements
	*/
	StatisticValue operator()(DataType type, bool forcedOutput = false) const;

	std::ostream & print(std::ostream & stream) const;  /*!<  Print the current status of the system. */

	/*! \brief Evaluate the nonlinear part of the STATE_UPDATE/OUTPUT_UPDATE with given Ts, state vector and disturbance/noise values according to the measurement statuses of the systems
	*
	* By using forcedOutput=true input, it assumes UPTODATE measurements
	*/
	Eigen::VectorXd EvalNonLinPart(double Ts, TimeUpdateType outType,
		const Eigen::VectorXd& state, const Eigen::VectorXd& in, bool forcedOutput = false) const;

/*! \brief Evaluate the STATE_UPDATE/OUTPUT_UPDATE with given Ts, state vector&variances and disturbance/noise vectors&variances according to the measurement statuses of the systems
	*
	* Returns the computed vector&variance, and it cross-variance matrices with the state (S_out_x) and the disturbance/noise (S_out_in)
	*
	* By using forcedOutput=true input, it assumes UPTODATE measurements
	*/
	StatisticValue Eval(TimeUpdateType outType, double Ts, const StatisticValue& state_, const StatisticValue& in,
		Eigen::MatrixXd& S_out_x, Eigen::MatrixXd& S_out_in, bool forcedOutput = false) const;

protected:
/*! \brief Simple class to fasten up the partitioning of state/output/disturbance/noise vectors and covariance matrixes for systems, sensors according to the measurement statuses of the systems
	*
	* By using forcedOutput=true input, it assumes UPTODATE measurements
	*/
	struct Partitioner {
		std::vector<size_t> nx;  /*!< Number of states of the systems */
		std::vector<size_t> nw;  /*!< Number of disturbances of the systems */
		std::vector<size_t> ny;  /*!< Number of outputs of the systems */
		std::vector<size_t> nv;  /*!< Number of noises of the systems */
		Partitioner(size_t N); /*!< Constructor. N is the number of systems */
		const std::vector<size_t>& n(DataType type) const; /*!< Get number of states/outputs/dist.-es/noises */
		Eigen::VectorBlock<Eigen::VectorXd> PartValue(DataType type,
			Eigen::VectorXd& value, int index) const;  /*!< Partitionate a vector */
		Eigen::VectorXd PartValue(DataType type,
			const Eigen::VectorXd& value, int index) const;  /*!< Partitionate a const vector */
		Eigen::Block<Eigen::MatrixXd> PartVariance(DataType type,
			Eigen::MatrixXd& value, int index1, int index2) const;   /*!< Partitionate a variance matrix */
		Eigen::MatrixXd PartVariance(DataType type,
			const Eigen::MatrixXd& value, int index1, int index2) const; /*!< Partitionate a const variance matrix */
		Eigen::Block<Eigen::MatrixXd> PartVariance(DataType type1, DataType type2,
			Eigen::MatrixXd& value, int index1, int index2) const; /*!< Partitionate a cross-variance matrix */
		StatisticValue PartStatisticValue(DataType type, const StatisticValue& value, int index) const; /*!< Constructor */
	};

	/*! \brief Get Partitioner struct according to the current measurement statuses of the systems
	*
	* By using forcedOutput=true input, it assumes UPTODATE measurements
	*/
	Partitioner getPartitioner(bool forcedOutput = false) const;

/*! \brief Get index for a (user defined) systemID. It returns -1 for the basesystem!
	*
	* 
	*/
	int _GetIndex(unsigned int ID) const;

	BaseSystemData & BaseSystem();   /*!< Returns a BaseSystemData ref for an index */

	const BaseSystemData & BaseSystem() const;   /*!< Returns a const BaseSystemData ref for an index */

	SensorData & Sensor(size_t index);   /*!< Returns a SensorData ref for an index */

	StatisticValue& State();   /*!< Returns ref of the state value  */

	SystemData* SystemByID(unsigned int ID);   /*!< Returns a SystemData* for an ID */

	const SensorData& Sensor(size_t index) const;   /*!< Returns a const SensorData ref for an index */

	bool isAvailable(int index) const;   /*!< Returns if the system with the given index is available */

	void resetMeasurement();   /*!< Set 'UPTODATE' meas. statuses to 'OBSOLETHE' */

	void PredictionDone(const StatisticValue& state, const StatisticValue& output) const;   /*!< Call it with predicition results to forward them to the set callback */

	void FilteringDone(const StatisticValue& state) const; /*!< Call it with filtering results to forward them to the set callback */

	void StepClock(TimeMicroSec dt); /*!< To step the inner clock in the overridden Step() function */

	void Call(const DataMsg& data) const; /*!< Call the callback if it is set */

private:
	BaseSystemData baseSystem; /*!< Stores the data related to the basesystem */
	std::vector<SensorData> sensorList;  /*!< Stores the data related to the sensors */
	StatisticValue state;  /*!< Stores the state of the system */
	TimeMicroSec time;  /*!< Time, initialized to zero, incremented by Step(Ts) via StepClock(Ts) */
	bool hasCallback;  /*!< If callback was set */
	Callback callback;  /*!< The callback if it was set */
};
#pragma once
#include "Eigen/Dense"

enum DataType {
	NOISE,
	DISTURBANCE,
	STATE,
	OUTPUT,
	INVALID_DATATYPE }; /*!< The relevant signal types */

enum TimeUpdateType {
	STATE_UPDATE,
	OUTPUT_UPDATE,
	INVALID_TIMEUPDATETYPE }; /*!< To identify if the STATE_UPDATE or the OUTPUT_UPDATE part of time update is considered */

enum OperationType {
	FILTER_TIME_UPDATE,
	FILTER_MEAS_UPDATE,
	SENSOR,
	FILTER_PARAM_ESTIMATION,
	GROUND_TRUTH,
	INVALID_OPERATIONTYPE }; /*!< The relevant sources of signals */

#include <chrono>
namespace SF {

	typedef std::chrono::time_point<std::chrono::system_clock> Time;

	typedef std::chrono::microseconds DTime;

	DTime duration_since_epoch(const Time& t);

	template<intmax_t a, intmax_t b>
	inline DTime duration_cast(const std::chrono::duration<long long, std::ratio<a, b>>& in);

	template<intmax_t a, intmax_t b>
	inline double duration_cast_to_sec(const std::chrono::duration<long long, std::ratio<a, b>>& in);

	Time InitFromDurationSinceEpochInMicroSec(const long long& value);

	DTime InitFromDurationInMicroSec(const long long& value);

	Time Now();

	template<intmax_t a, intmax_t b>
	inline DTime duration_cast(const std::chrono::duration<long long, std::ratio<a, b>>& in) {
		return std::chrono::duration_cast<DTime>(in);
	}

	template<intmax_t a, intmax_t b>
	inline double duration_cast_to_sec(const std::chrono::duration<long long, std::ratio<a, b>>& in) {
		return duration_cast(in).count() / 1e6;
	}
}
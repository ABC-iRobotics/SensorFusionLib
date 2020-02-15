#pragma once
#include "Eigen/Dense"
#include <chrono>

#include <type_traits>

namespace SF {

	enum DataType : unsigned char {
		NOISE = 1,
		DISTURBANCE,
		STATE,
		OUTPUT,
		INVALID_DATATYPE }; /*!< The relevant signal types */

	enum TimeUpdateType : unsigned char {
		STATE_UPDATE = 1,
		OUTPUT_UPDATE,
		INVALID_TIMEUPDATETYPE }; /*!< To identify if the STATE_UPDATE or the OUTPUT_UPDATE part of time update is considered */

	enum OperationType : unsigned char {
		FILTER_TIME_UPDATE = 1,
		FILTER_MEAS_UPDATE,
		SENSOR,
		FILTER_PARAM_ESTIMATION,
		GROUND_TRUTH,
		INVALID_OPERATIONTYPE }; /*!< The relevant sources of signals */

	template <typename E>
	constexpr auto to_underlying(E e) noexcept {
		return static_cast<std::underlying_type_t<E>>(e);
	}

	typedef std::chrono::time_point<std::chrono::system_clock> Time;

	typedef std::chrono::microseconds DTime;

	DTime duration_since_epoch(const Time& t);

	template<class _Rep, class _Period>
	inline DTime duration_cast(const std::chrono::duration<_Rep, _Period>& in);

	template<class _Rep, class _Period>
	inline double duration_cast_to_sec(const std::chrono::duration<_Rep, _Period>& in);

	Time InitFromDurationSinceEpochInMicroSec(const long long& value);

	DTime InitFromDurationInMicroSec(const long long& value);

	Time Now();

	template<class _Rep,class _Period>
	inline DTime duration_cast(const std::chrono::duration<_Rep, _Period>& in) {
		return std::chrono::duration_cast<DTime>(in);
	}

	template<class _Rep, class _Period>
	inline double duration_cast_to_sec(const std::chrono::duration<_Rep, _Period>& in) {
		return duration_cast(in).count() / 1e6;
	}
}
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

	inline std::string to_string(OperationType d) {
		switch (d)
		{
		case SF::FILTER_TIME_UPDATE:
			return "FILTER_TIME_UPDATE";
			break;
		case SF::FILTER_MEAS_UPDATE:
			return "FILTER_MEAS_UPDATE";
			break;
		case SF::SENSOR:
			return "SENSOR";
			break;
		case SF::FILTER_PARAM_ESTIMATION:
			return "FILTER_PARAM_ESTIMATION";
			break;
		case SF::GROUND_TRUTH:
			return "GROUND_TRUTH";
			break;
		case SF::INVALID_OPERATIONTYPE:
			return "INVALID_OPERATIONTYPE";
			break;
		default:
			break;
		}
	}

	inline std::string to_string(DataType d) {
		switch (d)
		{
		case SF::NOISE:
			return "NOISE";
			break;
		case SF::DISTURBANCE:
			return "DISTURBANCE";
			break;
		case SF::STATE:
			return "STATE";
			break;
		case SF::OUTPUT:
			return "OUTPUT";
			break;
		case SF::INVALID_DATATYPE:
			return "INVALID_DATATYPE";
			break;
		default:
			break;
		}
	}
}
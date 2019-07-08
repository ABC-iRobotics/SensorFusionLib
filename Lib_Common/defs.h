#pragma once
#include "Eigen/Dense"

enum DataType { NOISE, DISTURBANCE, STATE, OUTPUT }; /*!< The relevant signal types */

enum TimeUpdateType { STATE_UPDATE, OUTPUT_UPDATE }; /*!< To identify if the STATE_UPDATE or the OUTPUT_UPDATE part of time update is considered */

enum OperationType { FILTER_TIME_UPDATE, FILTER_MEAS_UPDATE, SENSOR, FILTER_PARAM_ESTIMATION, GROUND_TRUTH}; /*!< The relevant sources of signals */

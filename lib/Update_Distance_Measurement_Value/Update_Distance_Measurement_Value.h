/**
 ******************************************************************************
 * @file        distance_sensor.h
 * @brief       Interface for distance measurement (e.g., rear wall detection).
 *
 * @details     This header provides a function to perform distance measurement
 *              using an analog IR sensor. Raw values are filtered, converted
 *              to a physical distance [m], and validated for plausibility.
 *
 * @param[in]   b_Distance_Measurement_Active   Enables or disables measurement (e.g., based on state).
 *
 * @return      DistanceMeasurement              Structure containing the measured distance in meters
 *                                               and a validity flag (true = plausible).
 *
 * @note        Requires a globally defined `AnalogIn` object named `DistanceSensor`,
 *              e.g., declared in `main.cpp`.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24b - PM2
 * @date        2025-04-18
 ******************************************************************************
 */

 #ifndef DISTANCE_SENSOR_H_
 #define DISTANCE_SENSOR_H_
 
 #include "mbed.h"
 #include "AvgFilter.h"
 #include "Robot_Structs.h"
 #include "Robot_const.h"
 
 // =============================================================================
 // Configuration
 // =============================================================================
 
 /**
  * @brief       Size of the moving average filter.
  * @details     Number of samples used for smoothing the distance signal.
  */
 #define DISTANCE_AVG_FILTER_SIZE 10
 
 /**
  * @brief       Instance of the moving average filter for distance readings.
  * @note        Used to smooth sensor values before validation.
  */
 static AvgFilter distance_filter(AVG_FILTER_SIZE);
 
 // =============================================================================
 // Function Prototype
 // =============================================================================
 
 /**
  * @brief       Performs a distance measurement using an IR sensor.
  *
  * @param[in]   b_Distance_Measurement_Active   Indicates whether the measurement is active.
  *
  * @return      DistanceMeasurement              Structure with distance value in meters
  *                                               and validity status.
  */
 DistanceMeasurement distance_measurement(bool b_Distance_Measurement_Active);
 
 #endif // DISTANCE_SENSOR_H_
 
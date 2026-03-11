/**
 ******************************************************************************
 * @file        Update_Distance_Measurement_Value.h
 * @brief       Interface for IR distance measurement (e.g., front & rear detection).
 *
 * @details     This file provides a function to perform distance measurement using
 *              two analog IR sensors (front and rear). The raw values are filtered,
 *              converted into real distances [cm], and checked for plausibility.
 *
 * @param[in]   b_Distance_Measurement_Active   Enables or disables the measurement logic.
 *
 * @return      IRMeasurement                   Structure containing distance values in centimeters
 *                                              and validity flags for both sensors.
 *
 * @note        External IRSensor objects must be defined and initialized in `main.cpp`,
 *              e.g., `extern IRSensor IRFront;` and `extern IRSensor IRRear;`.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24b - PM2
 * @date        2025-05-02
 ******************************************************************************
 */

 #ifndef UPDATE_DISTANCE_MEASUREMENT_VALUE_H_
 #define UPDATE_DISTANCE_MEASUREMENT_VALUE_H_
 
 #include "System_Structs.h"
 #include "AvgFilter.h"
 
 // =============================================================================
 // Configuration
 // =============================================================================
 
 /**
  * @brief       Size of the moving average filter.
  * @details     Number of samples used for averaging.
  */
 #define IR_DISTANCE_AVG_FILTER_SIZE 10
 
 /**
  * @brief       Instance of a moving average filter for IR distance values.
  * @note        Declared static as it is only used within this header file.
  */
 static AvgFilter ir_distance_filter(IR_DISTANCE_AVG_FILTER_SIZE);
 
 // =============================================================================
 // Function Prototype
 // =============================================================================
 
 /**
  * @brief       Performs IR distance measurement using front and rear sensors.
  *
  * @param[in]   b_Distance_Measurement_Active   Indicates whether measurement should be executed.
  *
  * @return      IRMeasurement                   Structure containing distances [cm] and validity flags.
  */
 IRMeasurement IR_Sensor_Value(bool b_Distance_Measurement_Active);
 
 #endif // UPDATE_DISTANCE_MEASUREMENT_VALUE_H_
 
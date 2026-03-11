/**
 ******************************************************************************
 * @file        Update_Distance_Measurement_Value.cpp
 * @brief       Ultrasonic sensor – Reads distance and checks validity.
 *
 * @details     This function uses the integrated logic of the ultrasonic sensor
 *              to perform distance measurement. The return value of the `read()`
 *              method is evaluated and optionally passed on. The distance is 
 *              returned in centimeters. Invalid measurements are marked as -1.0f.
 *
 * @param[in]   b_Distance_Measurement_Active     Enables or disables measurement.
 *
 * @return      DistanceMeasurement                Structure containing distance [cm]
 *                                                 and a validity flag.
 *
 * @note        The sensor must be declared in the main program as 
 *              `extern UltrasonicSensor DistanceSensor`. The sensor logic already
 *              includes filtering and validation mechanisms. An optional moving 
 *              average filter is applied.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24b - PM2
 * @date        2025-04-18
 ******************************************************************************
 */

 #include "Update_Distance_Measurement_Value.h"
 #include "UltrasonicSensor.h"
 #include "AvgFilter.h"
 
 // =============================================================================
 // External Objects (must be defined in main.cpp)
 // =============================================================================
 
 extern UltrasonicSensor DistanceSensor;      ///< Global instance of ultrasonic sensor
 extern AvgFilter distance_filter;            ///< Moving average filter for distance values
 
 // =============================================================================
 // Main Function – Ultrasonic Distance Measurement
 // =============================================================================
 
 /**
  * @brief       Performs a distance measurement using the ultrasonic sensor.
  *
  * @details     This function reads the distance via `read()`. Return values
  *              below 10 cm or invalid results (< 0) are discarded. Valid
  *              values are additionally filtered using a moving average.
  *
  * @param[in]   b_Distance_Measurement_Active     Indicates whether measurement is active.
  *
  * @return      DistanceMeasurement                Structure with filtered distance [cm]
  *                                                 and a validity flag.
  */
 DistanceMeasurement distance_measurement(bool b_Distance_Measurement_Active)
 {
     DistanceMeasurement measured;  // Structure for returning the measurement result
 
     if (b_Distance_Measurement_Active)
     {
         // Read distance from sensor
         float us_distance_cm_candidate = DistanceSensor.read();
 
         if (us_distance_cm_candidate >= 10.0f)
         {
             float filtered = distance_filter.apply(us_distance_cm_candidate);
             measured.distance = filtered;
             measured.distance_valid = true;
         }
         else
         {
             measured.distance = -1.0f;
             measured.distance_valid = false;
         }
     }
     else
     {
         // Measurement disabled
         measured.distance = -1.0f;
         measured.distance_valid = false;
     }
 
     return measured;
 }
 
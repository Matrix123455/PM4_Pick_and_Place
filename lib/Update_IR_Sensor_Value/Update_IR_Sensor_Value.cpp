/**
 ******************************************************************************
 * @file        Update_Distance_Measurement_Value.cpp
 * @brief       IR Distance Measurement – Reads and validates sensor values.
 *
 * @details     This function processes two IR sensors (front and rear) to detect
 *              whether an object is present within a defined range (e.g., 4–10 cm).
 *              Sensor values are converted to centimeters using internal methods,
 *              and results are stored in an IRMeasurement structure.
 *
 * @param[in]   b_IR_Sensor_Active      Activation flag for distance measurement logic.
 *
 * @return      IRMeasurement           Structure containing detection and validity
 *                                      status for both front and rear sensors.
 *
 * @note        The IR sensor objects `IRFront` and `IRRear` must be externally defined
 *              and properly initialized in the main program (e.g., main.cpp).
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24b - PM2
 * @date        2025-04-18
 ******************************************************************************
 */

 #include "Update_Distance_Measurement_Value.h"
 #include "IRSensor.h"
 #include "AvgFilter.h"
 #include <stdio.h>
 
 // =============================================================================
 // External Objects (defined in main.cpp)
 // =============================================================================
 
 extern IRSensor IRFront;               ///< Front-facing IR sensor (external)
 extern IRSensor IRRear;                ///< Rear-facing IR sensor (external)
 extern AvgFilter distance_filter;      ///< Optional moving average filter
 
 // =============================================================================
 // Main Function – IR Distance Measurement
 // =============================================================================
 
 /**
  * @brief       Executes IR distance measurement using both front and rear sensors.
  *
  * @details     Uses the `readcm()` method of each IR sensor to determine distances.
  *              If a value lies within the valid range (4–10 cm), detection and
  *              validity flags are set accordingly. Otherwise, both flags remain false.
  *
  * @param[in]   b_IR_Sensor_Active      Indicates whether measurement should be active.
  *
  * @return      IRMeasurement           Structure with validity status and detection flags.
  */
 IRMeasurement IR_Sensor_Value(bool b_IR_Sensor_Active)
 {
     IRMeasurement measured;  // Return structure for sensor results
 
     if (b_IR_Sensor_Active)
     {
         // Direct measurement in centimeters
         float ir_distance_Front = IRFront.readcm();
         float ir_distance_Rear  = IRRear.readcm();
 
         // Range check for front sensor (valid range: 4–10 cm)
         if (ir_distance_Front >= 4.0f && ir_distance_Front <= 10.0f)
         {
             measured.distance_valid_Front = true;
             measured.plate_detected_Front = true;
         }
         else
         {
             measured.distance_valid_Front = false;
             measured.plate_detected_Front = false;
         }
 
         // Range check for rear sensor (valid range: 4–10 cm)
         if (ir_distance_Rear >= 4.0f && ir_distance_Rear <= 10.0f)
         {
             measured.distance_valid_Rear = true;
             measured.plate_detected_Rear = true;
         }
         else
         {
             measured.distance_valid_Rear = false;
             measured.plate_detected_Rear = false;
         }
     }
     else
     {
         // Measurement deactivated → Reset all flags
         measured.distance_valid_Front = false;
         measured.distance_valid_Rear = false;
         measured.plate_detected_Front = false;
         measured.plate_detected_Rear = false;
     }
 
     return measured;
 }
 
/**
 ******************************************************************************
 * @file        robot_structs.h
 * @brief       Structure definitions and global control flags for the PM2 robot.
 *
 * @details     This header file contains shared structures for return values
 *              (e.g. line following or distance measurement), as well as global
 *              control flags used for flow management in the main program.
 *
 *              Separating data structures from logic allows for clean
 *              modularization and easier maintenance or extension.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24b - PM2
 * @date        2025-04-18
 ******************************************************************************
 */

 #ifndef SYSTEM_STRUCTS_H_
 #define SYSTEM_STRUCTS_H_
 
 // =============================================================================
 // Structure Definitions
 // =============================================================================
 
 /**
  * @brief       Structure to return a single distance measurement.
  */
 struct DistanceMeasurement {
     float distance;                ///< Measured distance to an object [m].
     bool distance_valid;           ///< Validity flag (true = value is plausible).
 };
 
 /**
  * @brief       Structure to return IR sensor data (front and rear detection).
  */
 struct IRMeasurement {
     bool plate_detected_Front;     ///< Detection front: true = plate detected.
     bool plate_detected_Rear;      ///< Detection rear: true = plate detected.
     bool distance_valid_Front;     ///< Validity of the front IR measurement.
     bool distance_valid_Rear;      ///< Validity of the rear IR measurement.
 };
 
 /**
  * @brief       Structure to return line following information.
  */
 struct LineFollowerOut {
     float velocityright;           ///< Velocity for the right wheel [m/s].
     float velocityleft;            ///< Velocity for the left wheel [m/s].
     bool line_found;               ///< true = line detected.
     bool follow_active;            ///< true = line following active.
 };
 
 // =============================================================================
 // Global Control Flags (externally defined)
 // =============================================================================
 
 /**
  * @brief       Activation flag for the main task loop.
  */
 extern bool do_execute_main_task;
 
 /**
  * @brief       One-time reset flag on startup.
  * @details     Set to true once the main task is triggered for the first time.
  */
 extern bool do_reset_all_once;
 
 #endif // SYSTEM_STRUCTS_H_
 
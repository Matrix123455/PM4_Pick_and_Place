/**
 ******************************************************************************
 * @file        motor_control.h
 * @brief       Interface for speed control of the drive motors.
 *
 * @details     This header file provides the interface for controlling the two
 *              main drive motors. It allows target velocities to be passed to
 *              the motor objects `M1_DriveLeft` and `M1_DriveRight`, which must
 *              be defined externally (e.g., in main.cpp).
 *
 * @note        This function assumes that the DCMotor instances
 *              `M1_DriveLeft` and `M1_DriveRight` are globally available.
 *              It is typically called in the main program to control robot motion.
 *
 * @param[in]   f_M1_LeftVelocity     Target velocity for the left motor [m/s or RPS].
 * @param[in]   f_M1_RightVelocity    Target velocity for the right motor [m/s or RPS].
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24b - PM2
 * @date        2025-04-18
 ******************************************************************************
 */

 #ifndef MOTOR_CONTROL_H
 #define MOTOR_CONTROL_H
 
 // =============================================================================
 // Forward Declarations
 // =============================================================================
 
 /**
  * @brief       Forward declaration of the DCMotor class.
  * @note        Used to externally reference motor objects without full inclusion.
  */
 class DCMotor;
 
 // =============================================================================
 // Global Motor Objects (externally defined)
 // =============================================================================
 
 /**
  * @brief       Left drive motor – instance must be defined externally.
  */
 extern DCMotor M1_DriveLeft;
 
 /**
  * @brief       Right drive motor – instance must be defined externally.
  */
 extern DCMotor M1_DriveRight;
 
 // =============================================================================
 // Function Prototype
 // =============================================================================
 
 /**
  * @brief       Sets the target velocities of the left and right drive motors.
  *
  * @param[in]   f_M1_LeftVelocity      New target velocity for the left motor [m/s or RPS].
  * @param[in]   f_M1_RightVelocity     New target velocity for the right motor [m/s or RPS].
  *
  * @note        This function internally calls `setVelocity()` on each motor object.
  */
 void update_motor_values(float f_M1_LeftVelocity,
                          float f_M1_RightVelocity);
 
 #endif // MOTOR_CONTROL_H
 
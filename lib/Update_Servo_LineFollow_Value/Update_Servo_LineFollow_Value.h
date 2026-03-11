/**
 ******************************************************************************
 * @file        servo_control_linefollower.h
 * @brief       Interface for controlling the Linefollower servo.
 *
 * @details     This header file provides the interface for gradually adjusting
 *              the target angle of the Linefollower servo. It includes:
 *              - Definition of the incremental step size for angle updates,
 *              - External reference to the servo object,
 *              - Function prototype for periodic target angle update.
 *
 * @note        The corresponding implementation is located in
 *              Update_Servo_LineFollower_Value.cpp. The servo instance
 *              `Lift_Servo_LineFollower` must be defined externally
 *              (e.g., in main.cpp) and must be enabled.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24b - PM2
 * @date        2025-05-03
 ******************************************************************************
 */

 #ifndef SERVO_CONTROL_LINEFOLLOWER_H
 #define SERVO_CONTROL_LINEFOLLOWER_H
 
 #include <cmath>   ///< For mathematical functions such as ceilf()
 
 /**
  * @brief Increment step for gradual target angle adjustment.
  *
  * @details Added in each valid cycle to the current angle until
  *          the maximum value (1.0f) is reached.
  */
 #define SERVO_TARGET_INCREMENT 0.005f
 
 // =============================================================================
 // Forward declarations & global objects
 // =============================================================================
 
 /**
  * @brief Forward declaration of the Servo class.
  */
 class Servo;
 
 /**
  * @brief Global instance of the Linefollower servo (externally defined, e.g., in main.cpp).
  */
 extern Servo Lift_Servo_LineFollower;
 
 // =============================================================================
 // Function prototype
 // =============================================================================
 
 /**
  * @brief Periodically updates the target angle of the Linefollower servo.
  *
  * @param[in,out] f_Lift_Servo_LineFollower_TargetAngle   Target angle (range 0.0f – 1.0f)
  * @param[in]     main_task_period_ms                     Main loop cycle time in milliseconds
  *
  * @note The function sets the pulse width based on the provided angle.
  *       The target angle is only increased periodically.
  */
  void update_servo_LineFollower_values(float& f_Lift_Servo_LineFollower_TargetAngle,
                                        float main_task_period_ms);
 
 #endif // SERVO_CONTROL_LINEFOLLOWER_H
 
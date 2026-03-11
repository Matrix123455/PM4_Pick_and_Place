/**
 ******************************************************************************
 * @file        servo_control.h
 * @brief       Interface for controlling the lift servos.
 *
 * @details     This header file provides the interface for incremental control
 *              of the two lift servos. It includes:
 *              - Definition of the step size for increasing the target angle,
 *              - External references to global servo objects,
 *              - Function prototype for periodic target angle updates.
 *
 * @note        The implementation is provided in Update_Servo_Value.cpp.
 *              The servo objects must be defined externally (e.g., in main.cpp).
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24b - PM2
 * @date        2025-04-18
 ******************************************************************************
 */

 #ifndef SERVO_CONTROL_H
 #define SERVO_CONTROL_H
 
 #include <cmath>   ///< For mathematical functions like ceilf()
 
 /**
  * @brief Step size for incremental angle adjustment.
  *
  * @details This value is added to the current angle in each valid cycle
  *          until the maximum value (1.0f) is reached.
  */
 #define SERVO_TARGET_INCREMENT 0.005f
 
 // =============================================================================
 // Forward Declarations & Global Objects
 // =============================================================================
 
 /**
  * @brief Forward declaration of the Servo class.
  */
 class Servo;
 
 /**
  * @brief Global instance of the left lift servo (defined externally in main).
  */
 extern Servo Lift_Servo_Left;
 
 /**
  * @brief Global instance of the right lift servo (defined externally in main).
  */
 extern Servo Lift_Servo_Right;
 
 // =============================================================================
 // Function Prototype
 // =============================================================================
 
 /**
  * @brief Periodically updates the target angles of both lift servos.
  *
  * @param[in,out] f_Lift_Servo_Left_TargetAngle   Target angle for left servo (0.0f – 1.0f)
  * @param[in,out] f_Lift_Servo_Right_TargetAngle  Target angle for right servo (0.0f – 1.0f)
  * @param[in]     main_task_period_ms             Main loop period duration in milliseconds
  *
  * @note The function updates the pulse widths based on the given angle references.
  *       The angles are incremented periodically, not set directly.
  */
 void update_servo_values(float& f_Lift_Servo_Left_TargetAngle,
                          float& f_Lift_Servo_Right_TargetAngle,
                          float main_task_period_ms);
 
 #endif // SERVO_CONTROL_H
 
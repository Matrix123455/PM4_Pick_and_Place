/**
 ******************************************************************************
 * @file        Update_DCMotor_Value.cpp
 * @brief       Motor Control – Sets target velocities for the drive motors.
 *
 * @details     This function directly assigns new target velocities to the
 *              motor objects for the left and right main drives. These
 *              velocities are typically provided by the line follower,
 *              motion planner, or a state machine.
 *
 * @param[in]   f_M1_LeftVelocity     Target velocity for the left motor [m/s or RPS].
 * @param[in]   f_M1_RightVelocity    Target velocity for the right motor [m/s or RPS].
 *
 * @note        The actual motor control is handled internally by the motor
 *              object via its `setVelocity()` method. This function only
 *              sets the new target values.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24b - PM2
 * @date        2025-04-18
 ******************************************************************************
 */

 #include "Update_DCMotor_Value.h"
 #include "DCMotor.h"

 // =============================================================================
 // External Motor Instances (must be defined in main.cpp)
 // =============================================================================
 
 extern DCMotor M1_DriveLeft;    ///< Left main drive motor
 extern DCMotor M1_DriveRight;   ///< Right main drive motor
 
 // =============================================================================
 // Main Function – Set Target Velocities
 // =============================================================================
 
 /**
  * @brief       Sets the target velocities for both main drive motors.
  *
  * @param[in]   f_M1_LeftVelocity     Velocity for the left motor [m/s or RPS].
  * @param[in]   f_M1_RightVelocity    Velocity for the right motor [m/s or RPS].
  */
 void update_motor_values(float f_M1_LeftVelocity, float f_M1_RightVelocity)
 {
     M1_DriveLeft.setVelocity(f_M1_LeftVelocity);
     M1_DriveRight.setVelocity(f_M1_RightVelocity);
 }
 
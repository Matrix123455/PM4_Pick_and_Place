/**
 ******************************************************************************
 * @file        servo_motion_handler.cpp
 * @brief       Servo Control – Timed lift servo movement.
 *
 * @details     This function controls the lift servos as soon as both drive
 *              motors have stopped. A target angle for the left and right servo
 *              is set, and a timer is used to monitor when the movement is complete.
 *              The timer is triggered only once at the beginning of the movement.
 *
 * @param[in]   left_angle             Target angle for the left servo (0.0f–1.0f).
 * @param[in]   right_angle            Target angle for the right servo (0.0f–1.0f).
 * @param[in]   DriveLeft_Velocity     Velocity of the left drive motor.
 * @param[in]   DriveRight_Velocity    Velocity of the right drive motor.
 *
 * @return      true                   If the servo movement is complete.
 * @return      false                  If the movement is still ongoing.
 *
 * @note        The timer is automatically stopped and reset once the defined
 *              movement duration (`Servo_Move_Time`) has elapsed. The target
 *              angles are handled by separate functions like `update_servo_values()`.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24b - PM2
 * @date        2025-04-24
 ******************************************************************************
 */

 #include "Handle_Servo_Movement.h"
 #include "System_const.h"
 #include <cmath>
 #include <stdio.h>
 
 // =============================================================================
 // Global Variables for Servo Movement
 // =============================================================================
 
 bool b_Servo_Move_Active = false;              ///< Movement active flag
 float f_Lift_Servo_Left_TargetAngle = 0.0f;    ///< Target angle for the left servo
 float f_Lift_Servo_Right_TargetAngle = 0.0f;   ///< Target angle for the right servo
 
 // =============================================================================
 // Main Function – Executes servo motion when motors are stopped
 // =============================================================================
 
 /**
  * @brief       Executes a time-based lift servo movement.
  *
  * @details     This function monitors the drive motor speeds. When both motors
  *              are stopped, it initiates a servo movement and starts a timer.
  *              Once the defined movement duration is reached, it clears the flags
  *              and returns true.
  *
  * @param[in]   left_angle             Target angle for the left servo [0.0f–1.0f].
  * @param[in]   right_angle            Target angle for the right servo [0.0f–1.0f].
  * @param[in]   DriveLeft_Velocity     Current velocity of the left drive motor.
  * @param[in]   DriveRight_Velocity    Current velocity of the right drive motor.
  *
  * @return      true                   If the movement is finished.
  * @return      false                  If the movement is still in progress.
  */
 bool handle_servo_motion(float left_angle, float right_angle,
                          float DriveLeft_Velocity, float DriveRight_Velocity)
 {
     static bool timer_started = false;
     const float velocity_threshold = 0.01f; // Threshold for "stopped"
 
     // Check if both motors are stopped
     bool motors_stopped = (fabs(DriveLeft_Velocity) < velocity_threshold) &&
                           (fabs(DriveRight_Velocity) < velocity_threshold);
 
     // Start the movement only once when motors are stopped
     if (motors_stopped && !timer_started) {
         f_Lift_Servo_Left_TargetAngle  = left_angle - 0.01f;  // Optional offset
         f_Lift_Servo_Right_TargetAngle = right_angle;
 
         Servo_Move_Timer.stop();
         Servo_Move_Timer.reset();
         Servo_Move_Timer.start();
 
         b_Servo_Move_Active = true;
         timer_started = true;
     }
 
     // Check if movement duration has elapsed
     int elapsed_time_ms = Servo_Move_Timer.read_ms();
 
     if (b_Servo_Move_Active && elapsed_time_ms >= (SERVO_MOVE_TIME * 1000.0f)) {
         Servo_Move_Timer.stop();
         Servo_Move_Timer.reset();
 
         b_Servo_Move_Active = false;
         timer_started = false;
 
         return true;
     }
 
     return false;
 }
 
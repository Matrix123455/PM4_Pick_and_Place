/**
 ******************************************************************************
 * @file        Update_Servo_LineFollower_Value.cpp
 * @brief       Servo control – updates target angle & pulse width for the Linefollower servo.
 *
 * @details     This function gradually increases the target angle of the Linefollower
 *              servo up to a maximum of 1.0f. The update rate depends on the main loop
 *              period. The corresponding pulse width is continuously set based on the
 *              current target angle.
 *
 * @param[in,out] f_Lift_Servo_LineFollower_TargetAngle   Target angle of the servo (will be incremented).
 * @param[in]     main_task_period_ms                     Main loop cycle time in milliseconds.
 *
 * @note        This function uses a static counter for internal timing and assumes
 *              that the global servo object is properly defined and enabled.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24b - PM2
 * @date        2025-05-03
 ******************************************************************************
 */

 #include "Update_Servo_LineFollow_Value.h"
 #include "Servo.h"
 #include <algorithm>  // For std::min
 
 // =============================================================================
 // Global objects (externally defined)
 // =============================================================================
 extern Servo Lift_Servo_LineFollower;
 
 void update_servo_LineFollower_values(float& f_Lift_Servo_LineFollower_TargetAngle,
                                       float main_task_period_ms)
 {
     // Static counter – retains value between function calls
     static int dn_Servo_Counter_LineFollower = 0;
 
     // Calculate number of loop iterations per second
     const int Lift_Servo_loops_per_seconds = static_cast<int>(
         ceilf(1000.0f / main_task_period_ms)
     );
 
     /***********************************************************************
      * Update target angle periodically
      **********************************************************************/
     if ((f_Lift_Servo_LineFollower_TargetAngle < 1.0f) &&
         (dn_Servo_Counter_LineFollower % Lift_Servo_loops_per_seconds == 0) &&
         (dn_Servo_Counter_LineFollower != 0)) {
         f_Lift_Servo_LineFollower_TargetAngle = std::min(
             f_Lift_Servo_LineFollower_TargetAngle + SERVO_TARGET_INCREMENT, 1.0f);
     }
 
     // Increment counter (even if no update occurred)
     dn_Servo_Counter_LineFollower++;
 
     /***********************************************************************
      * Apply updated target angle to servo
      **********************************************************************/
     Lift_Servo_LineFollower.setPulseWidth(f_Lift_Servo_LineFollower_TargetAngle);
 }
 
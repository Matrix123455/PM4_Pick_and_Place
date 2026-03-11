/**
 ******************************************************************************
 * @file        Update_Servo_Value.cpp
 * @brief       Servo control – updates target angles and pulse widths.
 *
 * @details     This function gradually increases the target angles of the left
 *              and right lift servos up to a maximum of 1.0f, depending on the
 *              main loop period duration. The corresponding pulse widths are
 *              updated accordingly.
 *
 * @param[in,out] f_Lift_Servo_Left_TargetAngle   Target angle of the left servo (adjusted).
 * @param[in,out] f_Lift_Servo_Right_TargetAngle  Target angle of the right servo (adjusted).
 * @param[in]     main_task_period_ms             Main loop cycle time in milliseconds.
 *
 * @note        This function uses static counters for internal timing and
 *              assumes that the servo objects are globally defined and enabled.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24b - PM2
 * @date        2025-04-18
 ******************************************************************************
 */

 #include "Update_Servo_Value.h"
 #include "Servo.h"
 #include <algorithm>  // For std::min
 
 // =============================================================================
 // Global objects (defined externally)
 // =============================================================================
 extern Servo Lift_Servo_Left;
 extern Servo Lift_Servo_Right;
 
 void update_servo_values(float& f_Lift_Servo_Left_TargetAngle,
                          float& f_Lift_Servo_Right_TargetAngle,
                          float main_task_period_ms)
 {
     // Static counters for left and right servos – retained across function calls
     static int dn_Servo_Counter_Left = 0;
     static int dn_Servo_Counter_Right = 0;
 
     // Calculate how many main loop cycles correspond to one second
     const int Lift_Servo_loops_per_seconds = static_cast<int>(
         ceilf(1000.0f / main_task_period_ms)
     );
 
     /**************************************************************************
      * Helper function to update target angle periodically
      **************************************************************************/
     auto update_target_angle = [Lift_Servo_loops_per_seconds](float& angle, int& counter) {
         // Increase target angle by increment every second if below 1.0f
         if ((angle < 1.0f) &&
             (counter % Lift_Servo_loops_per_seconds == 0) &&
             (counter != 0)) {
             angle = std::min(angle + SERVO_TARGET_INCREMENT, 1.0f);
         }
 
         // Always increment counter, regardless of change
         counter++;
     };
 
     /**************************************************************************
      * Update left servo
      **************************************************************************/
     update_target_angle(f_Lift_Servo_Left_TargetAngle, dn_Servo_Counter_Left);
 
     /**************************************************************************
      * Update right servo
      **************************************************************************/
     update_target_angle(f_Lift_Servo_Right_TargetAngle, dn_Servo_Counter_Right);
 
     /**************************************************************************
      * Apply updated pulse widths to both servos
      **************************************************************************/
     Lift_Servo_Left.setPulseWidth(f_Lift_Servo_Left_TargetAngle);
     Lift_Servo_Right.setPulseWidth(f_Lift_Servo_Right_TargetAngle);
 }
 
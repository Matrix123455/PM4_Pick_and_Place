/**
 ******************************************************************************
 * @file        Update_Line_Follower_Value.cpp
 * @brief       Line Follower – calculates motor commands for line tracking.
 *
 * @details     This function acts as the interface to the LineFollower class.
 *              It checks if a line is detected, calculates the current angle
 *              deviation, and returns the corresponding wheel velocities.
 *              If no line is found or the deviation is too large, the robot
 *              drives straight at maximum velocity as a fallback.
 *
 * @param[in]   b_Check_Line_Follow   Enables line-following mode.
 *
 * @return      LineFollowerOut       Structure containing left/right wheel velocities
 *                                    (in RPS) and detection/activation status.
 *
 * @note        If line-following is disabled, both velocities are set to 0.0f.
 *              If the angle exceeds the allowed threshold, the robot drives straight.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24b - PM2
 * @date        2025-04-18
 ******************************************************************************
 */

 #include "Update_Line_Follower_Value.h"
 #include "LineFollower.h"
 
 // =============================================================================
 // Global objects (defined elsewhere)
 // =============================================================================
 
 extern LineFollower lineFollower;            ///< Global instance of the LineFollower object
 extern const float MAX_MOTOR_VELOCITY;         ///< Maximum velocity [RPS] for straight driving
 extern const float ROBOT_MAX_LINE_ANGLE_DEG; ///< Maximum allowed angle deviation from the line [°]
 
 // =============================================================================
 // Main function – Line following logic
 // =============================================================================
 
 LineFollowerOut line_follower(bool b_Check_Line_Follow)
 {
     LineFollowerOut linefollow = {0.0f, 0.0f, false, false};
 
     // If line-following is disabled → robot stands still
     if (!b_Check_Line_Follow) {
         return linefollow;
     }
 
     float angle = 0.0f;
 
     // Check if at least one LED on the sensor bar is active
     if (lineFollower.isLedActive()) {
         angle = lineFollower.getAngleDegrees();
         linefollow.line_found = true;
     }
 
     // Valid angle → activate line following and calculate wheel speeds
     if (fabs(angle) < ROBOT_MAX_LINE_ANGLE_DEG) {
         linefollow.follow_active = true;
         linefollow.velocityleft  = lineFollower.getLeftWheelVelocity();
         linefollow.velocityright = lineFollower.getRightWheelVelocity();
     } else {
         // Deviation too large → drive straight with max speed
         linefollow.velocityleft  = MAX_MOTOR_VELOCITY;
         linefollow.velocityright = MAX_MOTOR_VELOCITY;
     }
 
     return linefollow;
 }
 
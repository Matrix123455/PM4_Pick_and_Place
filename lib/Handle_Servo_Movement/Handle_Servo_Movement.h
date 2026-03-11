/**
 ******************************************************************************
 * @file        Handle_Servo_Movement.h
 * @brief       Interface for time-controlled servo movement.
 *
 * @details     This header provides the interface for coordinated control of
 *              the two lift servos. It includes:
 *              - External references to servo, motor, and timer objects,
 *              - A central control function using timer-based logic.
 *
 *              The function `handle_servo_motion()` sets target angles for
 *              the servos, starts a movement timer, and checks in each cycle
 *              whether the movement is complete. A state transition may follow.
 *
 *              Implementation is provided in `Handle_Servo_Movement.cpp`.
 *
 * @param[in]   left_angle             Target angle for the left servo (0.0f–1.0f).
 * @param[in]   right_angle            Target angle for the right servo (0.0f–1.0f).
 * @param[in]   DriveLeft_Velocity     Current velocity of the left motor.
 * @param[in]   DriveRight_Velocity    Current velocity of the right motor.
 *
 * @return      true                   If the servo movement is complete.
 * @return      false                  If the movement is still in progress.
 *
 * @note        Requires that global instances of servos, motors, timer, and
 *              state machine variables are available and defined externally.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24b - PM2
 * @date        2025-04-24
 ******************************************************************************
 */

 #ifndef SERVO_MOTION_HANDLER_H
 #define SERVO_MOTION_HANDLER_H
 
 #include "mbed.h"
 #include "System_State.h"
 
 // =============================================================================
 // Forward Declarations
 // =============================================================================
 
 class Servo;
 class DCMotor;
 
// =============================================================================
// External Declarations – Global Objects Used for Servo Movement Control
// =============================================================================

extern Timer Servo_Move_Timer;                    ///< Timer used to measure servo movement duration

extern float f_Lift_Servo_Left_TargetAngle;       ///< Target position for the left lift servo [0.0f–1.0f]
extern float f_Lift_Servo_Right_TargetAngle;      ///< Target position for the right lift servo [0.0f–1.0f]

extern bool b_Servo_Move_Active;                  ///< Flag indicating an active servo movement
extern bool b_Motor_Movement_Active;              ///< Flag indicating whether motors are currently moving

extern ExecuteState execute_state;                ///< Current sub-state of the execution state machine

extern Servo Lift_Servo_Left;                     ///< Global instance of the left lift servo
extern Servo Lift_Servo_Right;                    ///< Global instance of the right lift servo

extern DCMotor M1_DriveLeft;                      ///< Global instance of the left drive motor
extern DCMotor M1_DriveRight;                     ///< Global instance of the right drive motor

 
 // =============================================================================
 // Function Prototype
 // =============================================================================
 
 /**
  * @brief       Executes a time-controlled movement of the lift servos.
  *
  * @param[in]   left_angle             Target angle for the left servo (0.0f–1.0f).
  * @param[in]   right_angle            Target angle for the right servo (0.0f–1.0f).
  * @param[in]   DriveLeft_Velocity     Current velocity of the left motor [m/s].
  * @param[in]   DriveRight_Velocity    Current velocity of the right motor [m/s].
  *
  * @return      true                   If movement is finished.
  * @return      false                  If movement is still in progress.
  */
 bool handle_servo_motion(float left_angle,
                          float right_angle,
                          float DriveLeft_Velocity,
                          float DriveRight_Velocity);
 
 #endif // SERVO_MOTION_HANDLER_H
 
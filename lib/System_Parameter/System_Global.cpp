/**
 ******************************************************************************
 * @file        robot_globals.cpp
 * @brief       Definition of global hardware objects and states for the PM2 robot.
 *
 * @details     This file contains all definitions of global objects and constants
 *              used throughout the project. It provides concrete implementations
 *              for declarations found in the following headers:
 *              - robot_io.h:        I/O components like motors, servos, sensors
 *              - robot_state.h:     State machine for main and sub-tasks
 *              - robot_structs.h:   Global structures (e.g. LineFollowerOut)
 *              - robot_const.h:     Project-wide parameters and constants
 *
 * @note        This file must only be included once to avoid multiple definition
 *              errors at compile time.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24b - PM2
 * @date        2025-04-18
 ******************************************************************************
 */

 #include "PESBoardPinMap.h"

 // =============================================================================
 // Project Module Includes
 // =============================================================================
 #include "System_IO.h"
 #include "System_State.h"
 #include "System_Structs.h"
 #include "System_const.h"
 #include "System_Coordinate_System_Mapping.h"
 
 // =============================================================================
 // Main Task Timing & Encoder Configuration
 // =============================================================================
 const int   MAIN_TASK_PERIOD_MS            = 20;
 const int   TICKS_PER_ROTATION             = 360;
 const int   MAX_ROTATIONS                  = 10;
 const int   MAX_TICKS                      = MAX_ROTATIONS * TICKS_PER_ROTATION;
 const int   STATE_HISTORY_ARRAY_LENGTH     = 10;
 const float WAIT_START_TIME                = 2.0f;
 const float WAIT_STOP_TIME                 = 2.0f;
 
 // =============================================================================
 // Motor Parameters – Main Drive
 // =============================================================================
 const float M1_GEAR_RATIO                  = 78.125f;
 const float M1_KN                          = 140.0f / 12.0f;
 const float M1_VOLTAGE_MAX                 = 12.0f;
 
 const float M2_GEAR_RATIO                  = 78.125f;
 const float M2_KN                          = 140.0f / 12.0f;
 const float M2_VOLTAGE_MAX                 = 12.0f;
 
 // =============================================================================
 // Servo Configuration
 // =============================================================================
 const float LIFT_SERVO_LEFT_ANG_MIN                    = 0.0325f;
 const float LIFT_SERVO_LEFT_ANG_MAX                    = 0.1175f;
 const float LIFT_SERVO_LEFT_MAX_ACC                    = 0.3f;
 
 const float LIFT_SERVO_RIGHT_ANG_MIN                   = 0.0325f;
 const float LIFT_SERVO_RIGHT_ANG_MAX                   = 0.1175f;
 const float LIFT_SERVO_RIGHT_MAX_ACC                   = 0.3f;
 
 const float LIFT_SERVO_LINEFOLLOWER_ANG_MIN            = 0.03f;  //0.123
 const float LIFT_SERVO_LINEFOLLOWER_ANG_MAX            = 0.123f;    //0.03
 const float LIFT_SERVO_LINEFOLLOWER_MAX_ACC            = 0.8f;
 
 const float LIFT_SERVO_LOWER_POSITION                  = 1.0f;
 const float LIFT_SERVO_UPPER_POSITION                  = 0.0f;
 const float LIFT_SERVO_LIFT_POSITION                   = 0.4f;

 const float LIFT_SERVO_LINEFOLLOWER_LOWER_POSITION     = 0.63f;
 const float LIFT_SERVO_LINEFOLLOWER_MIDDLE_POSITION    = 0.4f;
 const float LIFT_SERVO_LINEFOLLOWER_UPPER_POSITION     = 0.0f;
 
 const float SERVO_MOVE_TIME                            = 3.0f;
 
 // =============================================================================
 // Line Follower Parameters
 // =============================================================================
 const float BAR_DIST                       = 0.120f;
 const float D_WHEEL                        = 0.04f;
 const float B_WHEEL                        = 0.095f;
 const float MAX_MOTOR_VELOCITY             = 0.5f;
 const float ROBOT_MAX_LINE_ANGLE_DEG       = 20.0f;
 const float KP                             = 10.5f;
 const float KP_NL                          = 20.0f;

 // =============================================================================
 // IR Sensor Parameters
 // =============================================================================
 const float IRSENSOR_CALIBRATION_MAX       = 11915.4582f;
 const float IRSENSOR_CALIBRATION_MIN       = 80.0f;
 
 // =============================================================================
 // Ultrasonic & Filter
 // =============================================================================
 const float AVG_FILTER_SIZE                = 3;
 
 // =============================================================================
 // Movement Parameters – End Position
 // =============================================================================
 const float END_PLATE_DISTANCE             = 20.0f;
 
 // =============================================================================
 // State Machine
 // =============================================================================
 SystemState    system_state             = STOPPED;
 ExecuteState   execute_state           = CHECK_FOR_LINE;
 ExecuteState   State_History[STATE_HISTORY_ARRAY_LENGTH];
 
 // =============================================================================
 // Control Flags
 // =============================================================================
 bool           do_execute_main_task    = false;
 bool           do_reset_all_once       = false;
 
 // =============================================================================
 // Motor Instances
 // =============================================================================
 DCMotor M1_DriveLeft(PB_PWM_M3, PB_ENC_A_M3, PB_ENC_B_M3, M1_GEAR_RATIO, M1_KN, M1_VOLTAGE_MAX);
 DCMotor M1_DriveRight(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, M2_GEAR_RATIO, M2_KN, M2_VOLTAGE_MAX);
 
 // =============================================================================
 // Servo Instances
 // =============================================================================
 Servo Lift_Servo_Left(PB_D0);
 Servo Lift_Servo_Right(PB_D1);
 Servo Lift_Servo_LineFollower(PB_D2);
 
 // =============================================================================
 // Sensors & I/O
 // =============================================================================
 LineFollower     lineFollower(PB_9, PB_8, BAR_DIST, D_WHEEL, B_WHEEL, M1_DriveLeft.getMaxPhysicalVelocity());
 UltrasonicSensor DistanceSensor(PB_D3);
 IRSensor         IRFront(PC_2);
 IRSensor         IRRear(PC_3);
 
 // =============================================================================
 // Inputs / Outputs & User LED
 // =============================================================================
 DebounceIn Start_Button(BUTTON1);
 DigitalOut Enable_DCMotor(PB_ENABLE_DCMOTORS);
 DigitalOut PES_Led1(LED1);
 
 // =============================================================================
 // Timers
 // =============================================================================
 Timer main_task_timer;
 Timer Servo_Move_Timer;
 Timer wait_start_timer;
 Timer wait_stop_timer;
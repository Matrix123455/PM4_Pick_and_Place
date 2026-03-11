/**
 ******************************************************************************
 * @file        robot_io.h
 * @brief       Global I/O components – Declarations for motors, servos, sensors & I/Os.
 *
 * @details     This file contains all external declarations for hardware objects
 *              used in the robot project. These include motors, sensors, switches,
 *              LEDs, and timers. The actual definitions are located in exactly one
 *              implementation file (e.g. `robot_globals.cpp`).
 *
 * @note        All declared objects must be defined in exactly one source file
 *              (no multiple definitions allowed).
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24b - PM2
 * @date        2025-04-18
 ******************************************************************************
 */

 #ifndef SYSTEM_IO_H_
 #define SYSTEM_IO_H_
 
 #include "mbed.h"
 #include "DCMotor.h"
 #include "Servo.h"
 #include "LineFollower.h"
 #include "DebounceIn.h"
 #include "UltrasonicSensor.h"
 #include "IRSensor.h"
 
 // =============================================================================
 // Motors
 // =============================================================================
 
 /**
  * @brief       Main drive motor – left side.
  */
 extern DCMotor M1_DriveLeft;
 
 /**
  * @brief       Main drive motor – right side.
  */
 extern DCMotor M1_DriveRight;
 
 // =============================================================================
 // Servos
 // =============================================================================
 
 /**
  * @brief       Lift servo – left side.
  */
 extern Servo Lift_Servo_Left;
 
 /**
  * @brief       Lift servo – right side.
  */
 extern Servo Lift_Servo_Right;
 
 /**
  * @brief       Servo for adjusting the height of the line follower.
  */
 extern Servo Lift_Servo_LineFollower;
 
 // =============================================================================
 // Sensors
 // =============================================================================
 
 /**
  * @brief       Line follower sensor.
  */
 extern LineFollower lineFollower;
 
 /**
  * @brief       Ultrasonic sensor for distance measurement.
  */
 extern UltrasonicSensor DistanceSensor;
 
 /**
  * @brief       IR sensor – front mounted.
  */
 extern IRSensor IRFront;
 
 /**
  * @brief       IR sensor – rear mounted.
  */
 extern IRSensor IRRear;
 
 // =============================================================================
 // Inputs (buttons, switches, etc.)
 // =============================================================================
 
 /**
  * @brief       Start button with debounce.
  */
 extern DebounceIn Start_Button;
 
 // =============================================================================
 // Outputs (e.g. LEDs)
 // =============================================================================
 
 /**
  * @brief       Enable signal for motor control.
  */
 extern DigitalOut Enable_DCMotor;
 
 /**
  * @brief       User LED on the PES board.
  */
 extern DigitalOut PES_Led1;
 
 // =============================================================================
 // Timers
 // =============================================================================
 
 /**
  * @brief       Cycle timer for the main task.
  */
 extern Timer main_task_timer;
 extern Timer wait_start_timer;
 extern Timer wait_stop_timer;
 #endif // SYSTEM_IO_H_
 
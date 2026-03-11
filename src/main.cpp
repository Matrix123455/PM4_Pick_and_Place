/**
 ******************************************************************************
 *              Main Project for PM4 Project -> Pick and Place System
 *
 * @file        main.cpp
 * @brief       Main control program for the PM4 system.
 *
 * @details     This file contains the full control logic for the robot,
 *              including initialization, state machine execution, and the
 *              coordination of sensors, actuators, and motion behavior.
 *              The structure is modular and executes state-specific logic
 *              in a defined sequence.
 *
 * @note        Changes and expansions are documented in the changelog below.
 *
 * @author      C. Meier, N. Bickel, L. Huber, A. von Aarburg, M. Hurschler
 *              ZHAW School of Engineering, ST24a - PM4
 * @date        2026-03-11
 ******************************************************************************
 *
 * @par Change Log
 * Date       | Author  | Changes
 * :--------: | :-----: | :---------------------------------------------------:
 * 11.03.2026 | meierc05| Init Project (created github repository, added libaries)
 * 09.03.2025 | meierc05| Ad
 ******************************************************************************
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "mbed.h"
#include "ThisThread.h"

/******************************************************************************
 * ETO Project Module Includes
 *****************************************************************************/
    #include "Robot_IO.h"                          ///< Global objects like motors, sensors, etc.
    #include "Robot_State.h"                       ///< Main and sub-states
    #include "Robot_Structs.h"                     ///< Structs like LineFollowerOut, etc.
    #include "Robot_const.h"                       ///< Central constants and parameters

    // Motor-related modules
    #include "Update_Servo_Value.h"
    #include "Update_DCMotor_Value.h"
    #include "Handle_Servo_Movement.h"
    #include "Update_Servo_LineFollow_Value.h"

    // Sensor-related modules
    #include "Update_Line_Follower_Value.h"
    #include "Update_Distance_Measurement_Value.h"
    #include "Update_IR_Sensor_Value.h"

/******************************************************************************
 * Local Function Prototypes
 *****************************************************************************/
void toggle_do_execute_main_fcn();
int  push_state_history(ExecuteState new_state, int history_idx);

/** **************************************************************************
 * @brief Main
 * @param :
 * **************************************************************************/
int main()
{
    /******************************************************************************
    * Initialization
    ******************************************************************************/

    // Behavior control flags
    bool         b_Check_Line_Follow                   = false;    ///< Flag to check if line is found
    bool         b_Distance_Measurement_Active         = false;    ///< Flag to enable distance measurement
    bool         b_Servo_Move_Active                   = false;    ///< Flag indicating active servo movement
    bool         b_IR_Sensor_Active                    = false;    ///< Flag to activate IR sensor reading
    bool         b_Rear_Sensor_Plate_detected          = false;    ///< Flag for rear plate detection
    bool         b_Program_Cycle_done                  = false;    ///< Program cycle completion flag
    static bool  b_first_entry                         = true;     ///< Flag for first-time entry in a state

    // State history index
    int State_History_Index                            = 0;

    // Motor velocity values left
    float f_M1_LeftVelocity                            = 0.0f;     ///< Current velocity of left motor
    float f_M1_LeftVelocityold                         = 0.0f;     ///< Previous velocity of left motor

    // Motor velocity values right
    float f_M1_RightVelocity                           = 0.0f;     ///< Current velocity of right motor
    float f_M1_RightVelocityold                        = 0.0f;     ///< Previous velocity of right motor

    // Target angles for lift servos left
    float f_Lift_Servo_Left_TargetAngle                = 0.0f;     ///< Target angle of left lift servo
    float f_Lift_Servo_Left_TargetAngle_old            = 0.1f;     ///< Previous angle of left lift servo

    // Target angles for lift servos right
    float f_Lift_Servo_Right_TargetAngle               = 0.0f;     ///< Target angle of right lift servo
    float f_Lift_Servo_Right_TargetAngle_old           = 0.1f;     ///< Previous angle of right lift servo

    // Target angles for line follower servo
    float f_Lift_Servo_LineFollower_TargetAngle        = 0.0f;     ///< Target angle of line follower servo
    float f_Lift_Servo_LineFollower_TargetAngle_old    = 0.1f;     ///< Previous angle of line follower servo

    // Timer stop and reset
    wait_start_timer.stop();
    wait_start_timer.reset();

    wait_stop_timer.stop();
    wait_stop_timer.reset();

    // State history helper
    ExecuteState temp_thirdlast_state;

    // Sensor/actuator output structs
    LineFollowerOut     linefollowout;                             ///< Output from line follower
    DistanceMeasurement distancemeasurement;                       ///< Output from distance sensor
    IRMeasurement       ir_measurement;                            ///< Output from IR sensor

    /******************************************************************************
     * Motor Initialization
     ******************************************************************************/

    // Enable motion planners for smooth transitions
    M1_DriveLeft.enableMotionPlanner();                                            ///< Enable motion planning for left motor
    M1_DriveRight.enableMotionPlanner();                                           ///< Enable motion planning for right motor

    // Configure acceleration limits (60% of maximum)
    M1_DriveLeft.setMaxAcceleration( M1_DriveLeft.getMaxAcceleration() * 0.6f);    ///< Limit acceleration left
    M1_DriveRight.setMaxAcceleration(M1_DriveRight.getMaxAcceleration() * 0.6f);   ///< Limit acceleration right

    // Configure velocity limits (30% of maximum)
    M1_DriveLeft.setMaxVelocity( M1_DriveLeft.getMaxVelocity() * 0.3f);            ///< Limit velocity left
    M1_DriveRight.setMaxVelocity(M1_DriveRight.getMaxVelocity() * 0.3f);           ///< Limit velocity right


    /******************************************************************************
     * Servo Initialization
     ******************************************************************************/

    // Calibrate servo pulse ranges (min/max values)
    Lift_Servo_Left.calibratePulseMinMax(LIFT_SERVO_LEFT_ANG_MIN,LIFT_SERVO_LEFT_ANG_MAX);                          ///< Calibrate left lift servo
    Lift_Servo_Right.calibratePulseMinMax(LIFT_SERVO_RIGHT_ANG_MIN,LIFT_SERVO_RIGHT_ANG_MAX);                       ///< Calibrate right lift servo
    Lift_Servo_LineFollower.calibratePulseMinMax(LIFT_SERVO_LINEFOLLOWER_ANG_MIN,LIFT_SERVO_LINEFOLLOWER_ANG_MAX);  ///< Calibrate line follower servo

    // Set maximum servo acceleration
    Lift_Servo_Left.setMaxAcceleration(LIFT_SERVO_LEFT_MAX_ACC);                    ///< Max acceleration left
    Lift_Servo_Right.setMaxAcceleration(LIFT_SERVO_RIGHT_MAX_ACC);                  ///< Max acceleration right
    Lift_Servo_LineFollower.setMaxAcceleration(LIFT_SERVO_LINEFOLLOWER_MAX_ACC);    ///< Max acceleration line follower

    /******************************************************************************
     * Sensor Calibration
     ******************************************************************************/

    // Calibrate IR sensors (front and rear)
    IRFront.setCalibration( IRSENSOR_CALIBRATION_MAX, IRSENSOR_CALIBRATION_MIN);    ///< Calibrate front IR sensor
    IRRear.setCalibration(  IRSENSOR_CALIBRATION_MAX, IRSENSOR_CALIBRATION_MIN);    ///< Calibrate rear IR sensor

    /******************************************************************************
     * Line Follower Calibration
     ******************************************************************************/

    // Calibrate Line Follower
    lineFollower.setRotationalVelocityGain(KP, KP_NL);

    /******************************************************************************
     * Button Initialization
     ******************************************************************************/
    Start_Button.fall(&toggle_do_execute_main_fcn);
        

    /******************************************************************************
    * Main task execute                     (Always executed also if start button is not pressed)
    *****************************************************************************/

    // start main task timer
    main_task_timer.start();                                        // start main task timer

    // this loop will run forever
    while (true) {
        main_task_timer.reset();
        printf("Main Task\n");

        if (do_execute_main_task) {

            switch(robot_state) {
                case RobotState::STOPPED: {
                    printf("Robot State: STOPPED\n");
                
                    // Transition to RESETTING state to prepare for next run
                    robot_state = RobotState::RESETTING;
                    break;
                }
                
                case RobotState::RESETTING: {
                    printf("Robot State: RESETTING\n");
                
                    // Reset visual indicators and entry flag
                    PES_Led1       = 0;
                    b_first_entry  = true;
                
                    // Reset sensor output structure
                    linefollowout  = {0.0f, 0.0f, false, false};
                
                    // Disable motor output
                    Enable_DCMotor = false;
                
                    // Disable all servos if currently enabled
                    if (Lift_Servo_Left.isEnabled()){
                        Lift_Servo_Left.disable();
                    }
                    if (Lift_Servo_Right.isEnabled()){
                        Lift_Servo_Right.disable();
                    }
                    if (Lift_Servo_LineFollower.isEnabled()){
                        Lift_Servo_LineFollower.disable();
                    }
                
                    // Clear execution state history
                    for (int i = 0; i < STATE_HISTORY_ARRAY_LENGTH; ++i)
                        State_History[i] = ExecuteState::ABORTED_Execute;
                    State_History_Index = 0;
                
                    // Transition to IDLE state to wait for user input
                    robot_state = RobotState::IDLE;
                    break;
                }
                
                case RobotState::IDLE: {
                    printf("Robot State: IDLE\n");
                
                    // Wait for user input via start button to begin operation
                    if ((Start_Button == true) && (!b_Program_Cycle_done)) {
                        robot_state = RobotState::STARTING;
                    }
                    break;
                }
                
                case RobotState::STARTING: {
                    printf("Robot State: STARTING\n");
                
                    // Enable motor driver control
                    Enable_DCMotor = true;
                
                    // Activate all servos if not already enabled
                    if (!Lift_Servo_Left.isEnabled()){
                        Lift_Servo_Left.enable();
                    }
                    if (!Lift_Servo_Right.isEnabled()){
                        Lift_Servo_Right.enable();
                    }
                    if (!Lift_Servo_LineFollower.isEnabled()){
                        Lift_Servo_LineFollower.enable();
                    }

                    // Move the line follower to its lowered position
                    f_Lift_Servo_LineFollower_TargetAngle = LIFT_SERVO_LINEFOLLOWER_LOWER_POSITION;

                    // Start Timer
                    wait_start_timer.start();
                    int elapsed_time_ms = wait_start_timer.read_ms();

                    if (elapsed_time_ms >= (WAIT_START_TIME * 1000.0f)) {

                        wait_start_timer.stop();
                        wait_start_timer.reset();

                        // Transition to EXECUTE state to begin task sequence
                        robot_state = RobotState::EXECUTE;
                        break;
                    }

                    if (f_Lift_Servo_LineFollower_TargetAngle_old != f_Lift_Servo_LineFollower_TargetAngle) {
                        update_servo_LineFollower_values(f_Lift_Servo_LineFollower_TargetAngle, MAIN_TASK_PERIOD_MS);
                        f_Lift_Servo_LineFollower_TargetAngle_old = f_Lift_Servo_LineFollower_TargetAngle;
                    }

                    break;
                }
                
                case RobotState::EXECUTE: {
                    printf("Robot State: EXECUTE\n");
                
                    // Execute state machine logic
                    switch (execute_state) {
                        case ExecuteState::CHECK_FOR_LINE: {
                            printf("Execute State: CHECK_FOR_LINE\n");
                
                            /*************************************************************
                             *   Initial line detection sequence
                             *************************************************************/

                            // Store encoder values on first entry to track distance
                            static int start_ticks_left  = 0;
                            static int start_ticks_right = 0;

                            if (b_first_entry) {
                                start_ticks_left  = M1_DriveLeft.getEncoderCount();   // Save initial encoder count (left)
                                start_ticks_right = M1_DriveRight.getEncoderCount();  // Save initial encoder count (right)
                                b_first_entry     = false;
                            }

                            // Check if line follower LED indicates line detection
                            if (lineFollower.isLedActive()) {
                                printf("Line detected!\n");
                                update_motor_values(0.0f, 0.0f);           // Stop both motors
                                b_Check_Line_Follow = true;               // Enable line following
                            }

                            // If no line detected, continue moving forward until max ticks reached
                            if (!b_Check_Line_Follow) {
                                int delta_left  = abs(M1_DriveLeft.getEncoderCount()  - start_ticks_left);
                                int delta_right = abs(M1_DriveRight.getEncoderCount() - start_ticks_right);

                                if (delta_left > MAX_TICKS || delta_right > MAX_TICKS) {
                                    printf("Maximum ticks reached. Abort!\n");
                                    update_motor_values(0.0f, 0.0f);       // Stop motors for safety
                                }

                                update_motor_values(0.3f, 0.3f);           // Keep driving forward to search for the line
                            }

                            if (b_Check_Line_Follow) {
                                State_History_Index = push_state_history(ExecuteState::CHECK_FOR_LINE, State_History_Index);
                                execute_state = ExecuteState::FOLLOW_LINE;
                            }
                            break;
                        }
                        case ExecuteState::FOLLOW_LINE: {
                            printf("Execute State: FOLLOW_LINE\n");
                
                            /*************************************************************
                             *   Line following logic
                             *************************************************************/

                            // Activate distance measurement during line following
                            b_Distance_Measurement_Active = true;

                            // Perform line following based on current sensor input
                            linefollowout = line_follower(b_Check_Line_Follow);

                            // Update motor velocities from line follower output
                            f_M1_LeftVelocity  = (linefollowout.velocityleft * 0.3f);
                            f_M1_RightVelocity = (linefollowout.velocityright * 0.3f);

                            // Activate IR sensor to monitor plate detection
                            b_IR_Sensor_Active = true;
                            ir_measurement     = IR_Sensor_Value(b_IR_Sensor_Active);

                            // If the front IR sensor no longer detects the plate, transition to END_OF_LINE
                            if (!ir_measurement.plate_detected_Front) {
                                f_Lift_Servo_LineFollower_TargetAngle = LIFT_SERVO_LINEFOLLOWER_MIDDLE_POSITION;

                                State_History_Index = push_state_history(ExecuteState::FOLLOW_LINE, State_History_Index);
                                execute_state       = ExecuteState::END_OF_LINE;
                            }
                            break;
                        }
                        case ExecuteState::END_OF_LINE: {
                            printf("Execute State: END_OF_LINE\n");
                
                            /*************************************************************
                             *   Check if plate has reached the end of the line
                             *************************************************************/

                            // Adjust line follower servo to middle position
                            f_Lift_Servo_LineFollower_TargetAngle = LIFT_SERVO_LINEFOLLOWER_MIDDLE_POSITION;

                            // Read updated IR sensor values
                            ir_measurement = IR_Sensor_Value(true);

                            // If the front sensor no longer sees the plate and the rear does, continue driving slowly
                            if (!ir_measurement.plate_detected_Front && ir_measurement.plate_detected_Rear) {
                                f_M1_LeftVelocity  = 0.2f;
                                f_M1_RightVelocity = 0.2f;
                            } else {
                                // Stop motors and transition to the next state
                                f_M1_LeftVelocity  = 0.0f;
                                f_M1_RightVelocity = 0.0f;

                                State_History_Index = push_state_history(ExecuteState::END_OF_LINE, State_History_Index);
                                execute_state       = ExecuteState::LOWER_WEIGHT;
                            }
                            break;
                        }
                        case ExecuteState::LOWER_WEIGHT: {
                            printf("Execute State: LOWER_WEIGHT\n");
                
                            /*************************************************************
                             *   Lower the lifting arms
                             *************************************************************/

                            // Read current motor velocities for servo movement coordination
                            float currentLeftVelocity  = M1_DriveLeft.getVelocity();
                            float currentRightVelocity = M1_DriveRight.getVelocity();

                            // Set target angles for lifting servos to lower position
                            f_Lift_Servo_Left_TargetAngle  = LIFT_SERVO_LOWER_POSITION;
                            f_Lift_Servo_Right_TargetAngle = LIFT_SERVO_LOWER_POSITION;

                            // Perform servo motion and check if movement is complete
                            b_Servo_Move_Active = handle_servo_motion(f_Lift_Servo_Left_TargetAngle,
                                                                    f_Lift_Servo_Right_TargetAngle,
                                                                    currentLeftVelocity,
                                                                    currentRightVelocity);

                            // If movement is completed, update history and proceed to next state
                            if (b_Servo_Move_Active) {
                                State_History_Index = push_state_history(ExecuteState::LOWER_WEIGHT, State_History_Index);
                                execute_state       = ExecuteState::ROPE_MOVEMENT;
                            }
                            break;
                        }
                        case ExecuteState::ROPE_MOVEMENT: {
                            printf("Execute State: ROPE_MOVEMENT\n");
                
                            /*************************************************************
                             *   Move rope forward until plate is detected at front
                             *************************************************************/

                            // Read current IR sensor status
                            ir_measurement = IR_Sensor_Value(true);

                            // If plate is not yet detected at the front, keep moving forward
                            if (!ir_measurement.plate_detected_Front) {
                                f_M1_LeftVelocity  = 0.4f;
                                f_M1_RightVelocity = 0.4f;
                            } else {
                                // Stop motors and advance to next state
                                f_M1_LeftVelocity  = 0.0f;
                                f_M1_RightVelocity = 0.0f;

                                State_History_Index = push_state_history(ExecuteState::ROPE_MOVEMENT, State_History_Index);
                                execute_state       = ExecuteState::LIFT_WEIGHT;
                            }
                            break;
                        }
                        case ExecuteState::LIFT_WEIGHT: {
                            printf("Execute State: LIFT_WEIGHT\n");
                
                            /*************************************************************
                             *   Lift the plate using the lift servos
                             *************************************************************/

                            // Read current motor velocities
                            float f_currentLeftVelocity  = M1_DriveLeft.getVelocity();
                            float f_currentRightVelocity = M1_DriveRight.getVelocity();

                            // Set target angles for lifting
                            f_Lift_Servo_Left_TargetAngle  = LIFT_SERVO_LIFT_POSITION;
                            f_Lift_Servo_Right_TargetAngle = LIFT_SERVO_LIFT_POSITION;

                            // Trigger servo movement
                            b_Servo_Move_Active = handle_servo_motion(f_Lift_Servo_Left_TargetAngle,
                                                                      f_Lift_Servo_Right_TargetAngle,
                                                                      f_currentLeftVelocity,
                                                                      f_currentRightVelocity
                                                                      );

                            // Once motion is complete, determine next state based on history
                            if (b_Servo_Move_Active) {
                                temp_thirdlast_state = State_History[(State_History_Index - 2 + STATE_HISTORY_ARRAY_LENGTH) % STATE_HISTORY_ARRAY_LENGTH];

                                State_History_Index = push_state_history(ExecuteState::LIFT_WEIGHT, State_History_Index);
                                
                                if (temp_thirdlast_state == ExecuteState::END_OF_LINE) {
                                    execute_state = ExecuteState::MOVE_BAR;
                                } else {
                                    execute_state = ExecuteState::MOVE_PLATE;
                                }
                            }
                            break;
                        }
                        case ExecuteState::MOVE_BAR: {
                            printf("Execute State: MOVE_BAR\n");
                
                            /*************************************************************
                             *   Move bar until the rear sensor confirms plate detection
                             *************************************************************/

                            // Update IR sensor measurement
                            ir_measurement = IR_Sensor_Value(true);

                            // Set flag if plate is detected at rear
                            if (ir_measurement.plate_detected_Rear) {
                                b_Rear_Sensor_Plate_detected = true;
                            }

                            // If plate was previously detected and is now gone, continue moving
                            if (!ir_measurement.plate_detected_Rear && b_Rear_Sensor_Plate_detected) {
                                f_M1_LeftVelocity  = 0.0f;
                                f_M1_RightVelocity = 0.0f;

                                f_Lift_Servo_LineFollower_TargetAngle = LIFT_SERVO_LINEFOLLOWER_UPPER_POSITION;
                                b_Rear_Sensor_Plate_detected = false;

                                State_History_Index = push_state_history(ExecuteState::MOVE_BAR, State_History_Index);
                                execute_state = ExecuteState::LOWER_WEIGHT;

                            } else {
                                if(ir_measurement.plate_detected_Rear && ir_measurement.plate_detected_Front){
                                    f_Lift_Servo_LineFollower_TargetAngle = LIFT_SERVO_LINEFOLLOWER_UPPER_POSITION;

                                    State_History_Index = push_state_history(ExecuteState::MOVE_BAR, State_History_Index);
                                    execute_state = ExecuteState::MOVE_PLATE;
                                    break;

                                } else {
                                    f_M1_LeftVelocity  = 0.3f;
                                    f_M1_RightVelocity = 0.3f;
                                }
                            }
                            break;
                        }
                        case ExecuteState::MOVE_PLATE: {
                            printf("Execute State: MOVE_PLATE\n");
                
                            /*************************************************************
                             *   Move plate forward until both sensors confirm end position
                             *************************************************************/

                            // Read current distance and IR sensor values
                            b_Distance_Measurement_Active = true;
                            distancemeasurement = distance_measurement(b_Distance_Measurement_Active);
                            float tempDistance = distancemeasurement.distance;
                            ir_measurement     = IR_Sensor_Value(true);

                            printf("Distance: %f\n", tempDistance);

                            // Check if both sensors detect the plate at the end
                            if (ir_measurement.plate_detected_Front && ir_measurement.plate_detected_Rear) {
                                if ((tempDistance <= END_PLATE_DISTANCE) && (tempDistance > 10.0f)) {
                                    // Stop movement if close enough to the end
                                    f_M1_LeftVelocity  = 0.0f;
                                    f_M1_RightVelocity = 0.0f;

                                    State_History_Index = push_state_history(ExecuteState::MOVE_PLATE, State_History_Index);
                                    execute_state       = ExecuteState::ABORTED_Execute;

                                } else {
                                    // Continue moving until within the desired distance
                                    f_M1_LeftVelocity  = 0.5f;
                                    f_M1_RightVelocity = 0.5f;
                                }
                            } else {
                                // Continue moving until both sensors detect the plate
                                f_M1_LeftVelocity  = 0.3f;
                                f_M1_RightVelocity = 0.3f;
                            }
                            break;
                        }
                        case ExecuteState::ABORTED_Execute: {
                            printf("Execute State: ABORTED_Execute\n");

                            /*************************************************************
                             *   Abort current operation and prepare to complete
                             *************************************************************/

                            // Save aborted state and transition to completing sequence
                            State_History_Index = push_state_history(ExecuteState::ABORTED_Execute, State_History_Index);
                            robot_state         = RobotState::COMPLETING;
                            break;
                        }
                    }
                
                    /*************************************************************
                     *   Update motors and servos if values changed
                     *************************************************************/

                    // Update motor velocities if changed
                    if (f_M1_LeftVelocityold  != f_M1_LeftVelocity  ||
                        f_M1_RightVelocityold != f_M1_RightVelocity) {
                        update_motor_values(f_M1_LeftVelocity, f_M1_RightVelocity);
                        f_M1_LeftVelocityold  = f_M1_LeftVelocity;
                        f_M1_RightVelocityold = f_M1_RightVelocity;
                    }

                    // Update lift servos if target angle changed
                    if (f_Lift_Servo_Left_TargetAngle_old  != f_Lift_Servo_Left_TargetAngle ||
                        f_Lift_Servo_Right_TargetAngle_old != f_Lift_Servo_Right_TargetAngle) {
                        update_servo_values(f_Lift_Servo_Left_TargetAngle, f_Lift_Servo_Right_TargetAngle, MAIN_TASK_PERIOD_MS);
                        f_Lift_Servo_Left_TargetAngle_old  = f_Lift_Servo_Left_TargetAngle;
                        f_Lift_Servo_Right_TargetAngle_old = f_Lift_Servo_Right_TargetAngle;
                    }
                // Update lift servos LineFollower if target angle changed
                    if (f_Lift_Servo_LineFollower_TargetAngle_old != f_Lift_Servo_LineFollower_TargetAngle) {
                        update_servo_LineFollower_values(f_Lift_Servo_LineFollower_TargetAngle, MAIN_TASK_PERIOD_MS);
                        f_Lift_Servo_LineFollower_TargetAngle_old = f_Lift_Servo_LineFollower_TargetAngle;
                    }
                    break;
                }
                case RobotState::COMPLETING: {
                    printf("Robot State: COMPLETING\n");

                    // Oscillation cycle parameters
                    static int   cycle     = 0;
                    const int   max_cycles = static_cast<int>(1000.0f / MAIN_TASK_PERIOD_MS);  // 20 seconds duration

                    // Check if cycle duration is complete
                    if (++cycle >= max_cycles) {
                        // Stop motors and reset servos
                        f_M1_LeftVelocity               = 0.0f;
                        f_M1_RightVelocity              = 0.0f;
                        f_Lift_Servo_Left_TargetAngle  = LIFT_SERVO_UPPER_POSITION;
                        f_Lift_Servo_Right_TargetAngle = LIFT_SERVO_UPPER_POSITION;
                        f_Lift_Servo_LineFollower_TargetAngle = LIFT_SERVO_LINEFOLLOWER_UPPER_POSITION;

                        // Reset cycle state
                        cycle     = 0;

                        // Transition to COMPLETE state
                        robot_state = RobotState::COMPLETE;
                    }

                    /*************************************************************
                     *   Update motors and servos if values changed
                     *************************************************************/

                    // Update motor velocities if changed
                    if (f_M1_LeftVelocityold  != f_M1_LeftVelocity  ||
                        f_M1_RightVelocityold != f_M1_RightVelocity) {
                        update_motor_values(f_M1_LeftVelocity, f_M1_RightVelocity);
                        f_M1_LeftVelocityold  = f_M1_LeftVelocity;
                        f_M1_RightVelocityold = f_M1_RightVelocity;
                    }

                    // Update lift servos if target angle changed
                    if (f_Lift_Servo_Left_TargetAngle_old  != f_Lift_Servo_Left_TargetAngle ||
                        f_Lift_Servo_Right_TargetAngle_old != f_Lift_Servo_Right_TargetAngle) {
                        update_servo_values(f_Lift_Servo_Left_TargetAngle, f_Lift_Servo_Right_TargetAngle, MAIN_TASK_PERIOD_MS);
                        f_Lift_Servo_Left_TargetAngle_old  = f_Lift_Servo_Left_TargetAngle;
                        f_Lift_Servo_Right_TargetAngle_old = f_Lift_Servo_Right_TargetAngle;
                    }

                    // Update line follower servo if needed
                    if (f_Lift_Servo_LineFollower_TargetAngle_old != f_Lift_Servo_LineFollower_TargetAngle) {
                        update_servo_LineFollower_values(f_Lift_Servo_LineFollower_TargetAngle, MAIN_TASK_PERIOD_MS);
                        f_Lift_Servo_LineFollower_TargetAngle_old = f_Lift_Servo_LineFollower_TargetAngle;
                    }
                    break;
                }
                case RobotState::COMPLETE: {
                    printf("Robot State: COMPLETE\n");
        
                    wait_stop_timer.start();
                    int elapsed_time_ms = wait_stop_timer.read_ms();

                    if (elapsed_time_ms >= (WAIT_STOP_TIME * 1000.0f)) {

                        wait_stop_timer.stop();
                        wait_stop_timer.reset();

                        b_Program_Cycle_done = true;  // Set flag to indicate program completion

                    // Automatically transition back to RESETTING for next run
                        robot_state = RobotState::RESETTING;
                        break;
                    }
                    break;
                }
                case RobotState::ABORTING: {
                    printf("Robot State: ABORTING\n");
        
                    // Handle any critical shutdown logic before aborting
                    robot_state = RobotState::ABORTED;
                    break;
                }
                case RobotState::ABORTED: {
                    printf("Robot State: ABORTED\n");
        
                    // Remain in ABORTED state until manually reset or reinitialized
                    // robot_state = RobotState::RESETTING;
                    break;
                }
                default: {
                    // Catch-all fallback; should not be reached under normal operation
                    break;
                }
            }
        } else {
            // This block executes only once after toggling the main task ON
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // Reset relevant variables or system components here if needed
            }
        }
        // Toggle the user LED for visual feedback
        PES_Led1 = 1;

        // Calculate remaining time and sleep to maintain fixed main loop period
        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();

        if (MAIN_TASK_PERIOD_MS - main_task_elapsed_time_ms < 0) {
            printf("Warning: Main task took longer than MAIN_TASK_PERIOD_MS\n");
        } else {
            thread_sleep_for(MAIN_TASK_PERIOD_MS - main_task_elapsed_time_ms);
        }
    }
}

/**
 ******************************************************************************
 * @brief Toggle execution of the main control loop.
 *
 * This function is connected to the user button interrupt. When the button is
 * pressed, it toggles the execution state of the main task. If the task is
 * being activated, a reset is triggered on the next cycle.
 ******************************************************************************
 */
void toggle_do_execute_main_fcn()
{
    // Toggle main task execution state
    do_execute_main_task = !do_execute_main_task;

    // If activated, trigger one-time reset
    if (do_execute_main_task)
        do_reset_all_once = true;
}

/**
 ******************************************************************************
 * @brief Push a new execution state to the state history buffer.
 *
 * Updates the circular buffer that tracks previously executed states, enabling
 * conditional logic based on past transitions. Older entries are overwritten
 * once the buffer limit is reached.
 *
 * @param new_state              The execution state to add.
 * @param state_history_index    Current index in the history buffer.
 * @return int                   Updated index after storing the new state.
 ******************************************************************************
 */
int push_state_history(ExecuteState new_state, int state_history_index)
{
    state_history_index = (state_history_index + 1) % STATE_HISTORY_ARRAY_LENGTH;
    State_History[state_history_index] = new_state;
    return state_history_index;
}


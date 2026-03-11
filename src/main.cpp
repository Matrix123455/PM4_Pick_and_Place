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
 * 11.03.2026 | meierc05| Prepared main Program and created Coordinate System Mapping module
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
#include "System_IO.h"                          ///< Global objects like motors, sensors, etc.
#include "System_State.h"                       ///< Main and sub-states
#include "System_Structs.h"                     ///< Structs like LineFollowerOut, etc.
#include "System_const.h"                       ///< Central constants and parameters
#include "System_Coordinate_System_Mapping.h"   ///< Coordinate mapping and position generation

// Motor-related modules
#include "Update_Servo_Value.h"
#include "Update_DCMotor_Value.h"
#include "Handle_Servo_Movement.h"

// Sensor-related modules
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
    bool b_Program_Cycle_done = false;    ///< Program cycle completion flag

    // State history index
    int State_History_Index = 0;

    // Motor velocity values left
    float f_M1_LeftVelocity = 0.0f;       ///< Current velocity of left motor
    float f_M1_LeftVelocityold = 0.0f;    ///< Previous velocity of left motor

    // Motor velocity values right
    float f_M1_RightVelocity = 0.0f;      ///< Current velocity of right motor
    float f_M1_RightVelocityold = 0.0f;   ///< Previous velocity of right motor

    // Target angles for lift servos left
    float f_Lift_Servo_Left_TargetAngle = 0.0f;      ///< Target angle of left lift servo
    float f_Lift_Servo_Left_TargetAngle_old = 0.1f;  ///< Previous angle of left lift servo

    // Target angles for lift servos right
    float f_Lift_Servo_Right_TargetAngle = 0.0f;      ///< Target angle of right lift servo
    float f_Lift_Servo_Right_TargetAngle_old = 0.1f;  ///< Previous angle of right lift servo

    // Timer stop and reset
    wait_start_timer.stop();
    wait_start_timer.reset();

    wait_stop_timer.stop();
    wait_stop_timer.reset();

    // State history helper
    ExecuteState temp_thirdlast_state;

    // Sensor/actuator output structs
    DistanceMeasurement distancemeasurement;   ///< Output from distance sensor
    IRMeasurement ir_measurement;              ///< Output from IR sensor

    /******************************************************************************
     * Motor Initialization
     ******************************************************************************/


    /******************************************************************************
     * Servo Initialization
     ******************************************************************************/


    /******************************************************************************
     * Sensor Calibration
     ******************************************************************************/


    /******************************************************************************
     * Button Initialization
     ******************************************************************************/
    Start_Button.fall(&toggle_do_execute_main_fcn);

    /******************************************************************************
     * Main task execute (Always executed also if start button is not pressed)
     ******************************************************************************/

    // start main task timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();
        printf("Main Task\n");

        if (do_execute_main_task) {

            switch (system_state) {

                case SystemState::STOPPED: {
                    printf("Robot State: STOPPED\n");

                    system_state = SystemState::RESETTING;
                    break;
                }

                case SystemState::RESETTING: {
                    printf("Robot State: RESETTING\n");

                    system_state = SystemState::IDLE;
                    break;
                }

                case SystemState::IDLE: {
                    printf("Robot State: IDLE\n");

                    // Wait for user input via start button to begin operation
                    if ((Start_Button == true) && (!b_Program_Cycle_done)) {
                        system_state = SystemState::STARTING;
                    }
                    break;
                }

                case SystemState::STARTING: {
                    printf("Robot State: STARTING\n");

                    system_state = SystemState::EXECUTE;
                    break;
                }

                case SystemState::EXECUTE: {
                    printf("Robot State: EXECUTE\n");

                    // Execute state machine logic
                    switch (execute_state) {

                        case ExecuteState::HOME: {
                            printf("Execute State: HOME\n");

                            State_History_Index = push_state_history(ExecuteState::HOME, State_History_Index);
                            execute_state = ExecuteState::RUN_TO_WAIT_PICK;
                            break;
                        }

                        case ExecuteState::RUN_TO_WAIT_PICK: {
                            printf("Execute State: RUN_TO_WAIT_PICK\n");

                            State_History_Index = push_state_history(ExecuteState::RUN_TO_WAIT_PICK, State_History_Index);
                            execute_state = ExecuteState::RUN_TO_PICK;
                            break;
                        }

                        case ExecuteState::RUN_TO_PICK: {
                            printf("Execute State: RUN_TO_PICK\n");

                            State_History_Index = push_state_history(ExecuteState::RUN_TO_PICK, State_History_Index);
                            execute_state = ExecuteState::PICK;
                            break;
                        }

                        case ExecuteState::PICK: {
                            printf("Execute State: PICK\n");

                            State_History_Index = push_state_history(ExecuteState::PICK, State_History_Index);
                            execute_state = ExecuteState::RUN_TO_WAIT_PLACE;
                            break;
                        }

                        case ExecuteState::RUN_TO_WAIT_PLACE: {
                            printf("Execute State: RUN_TO_WAIT_PLACE\n");

                            State_History_Index = push_state_history(ExecuteState::RUN_TO_WAIT_PLACE, State_History_Index);
                            execute_state = ExecuteState::RUN_TO_PLACE;
                            break;
                        }

                        case ExecuteState::RUN_TO_PLACE: {
                            printf("Execute State: RUN_TO_PLACE\n");

                            State_History_Index = push_state_history(ExecuteState::RUN_TO_PLACE, State_History_Index);
                            execute_state = ExecuteState::PLACE;
                            break;
                        }

                        case ExecuteState::PLACE: {
                            printf("Execute State: PLACE\n");

                            State_History_Index = push_state_history(ExecuteState::PLACE, State_History_Index);
                            execute_state = ExecuteState::MOVE_OUT;
                            break;
                        }

                        case ExecuteState::MOVE_OUT: {
                            printf("Execute State: MOVE_OUT\n");

                            State_History_Index = push_state_history(ExecuteState::MOVE_OUT, State_History_Index);
                            execute_state = ExecuteState::MOVE_IN;
                            break;
                        }

                        case ExecuteState::MOVE_IN: {
                            printf("Execute State: MOVE_IN\n");

                            State_History_Index = push_state_history(ExecuteState::MOVE_IN, State_History_Index);
                            execute_state = ExecuteState::OPEN_FLAP;
                            break;
                        }

                        case ExecuteState::OPEN_FLAP: {
                            printf("Execute State: OPEN_FLAP\n");

                            State_History_Index = push_state_history(ExecuteState::OPEN_FLAP, State_History_Index);
                            execute_state = ExecuteState::CLOSE_FLAP;
                            break;
                        }

                        case ExecuteState::CLOSE_FLAP: {
                            printf("Execute State: CLOSE_FLAP\n");

                            State_History_Index = push_state_history(ExecuteState::CLOSE_FLAP, State_History_Index);
                            execute_state = ExecuteState::OPEN_GRIPPER;
                            break;
                        }

                        case ExecuteState::OPEN_GRIPPER: {
                            printf("Execute State: OPEN_GRIPPER\n");

                            State_History_Index = push_state_history(ExecuteState::OPEN_GRIPPER, State_History_Index);
                            execute_state = ExecuteState::CLOSE_GRIPPER;
                            break;
                        }

                        case ExecuteState::CLOSE_GRIPPER: {
                            printf("Execute State: CLOSE_GRIPPER\n");

                            State_History_Index = push_state_history(ExecuteState::CLOSE_GRIPPER, State_History_Index);
                            execute_state = ExecuteState::CLOSE_GRIPPER;
                            break;
                        }

                        default: {
                            break;
                        }
                    }

                    break;
                }

                case SystemState::COMPLETING: {
                    printf("Robot State: COMPLETING\n");

                    system_state = SystemState::COMPLETE;
                    break;
                }

                case SystemState::COMPLETE: {
                    printf("Robot State: COMPLETE\n");

                    wait_stop_timer.start();
                    int elapsed_time_ms = wait_stop_timer.read_ms();

                    if (elapsed_time_ms >= (WAIT_STOP_TIME * 1000.0f)) {
                        wait_stop_timer.stop();
                        wait_stop_timer.reset();

                        b_Program_Cycle_done = true;  // Set flag to indicate program completion
                        system_state = SystemState::RESETTING;
                    }

                    break;
                }

                case SystemState::ABORTING: {
                    printf("Robot State: ABORTING\n");

                    system_state = SystemState::ABORTED;
                    break;
                }

                case SystemState::ABORTED: {
                    printf("Robot State: ABORTED\n");

                    system_state = SystemState::RESETTING;
                    break;
                }

                default: {
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
        int main_task_elapsed_time_ms =
            duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();

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
    if (do_execute_main_task) {
        do_reset_all_once = true;
    }
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
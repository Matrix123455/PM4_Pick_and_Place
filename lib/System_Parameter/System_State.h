/**
 ******************************************************************************
 * @file        robot_state.h
 * @brief       State definitions for the PM2 robot control system.
 *
 * @details     This file contains the enumerations and global state variables
 *              used to manage both the main and sub state machines.
 *              It serves as the foundation for the execution logic
 *              within the robot's state machine.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24b - PM2
 * @date        2025-04-18
 ******************************************************************************
 */

 #ifndef SYSTEM_STATE_H_
 #define SYSTEM_STATE_H_
 
 // =============================================================================
 // Top-Level States – SystemState (Main FSM)
 // =============================================================================
 
 /**
  * @brief       Main state machine for the robot.
  */
 enum SystemState {
     STOPPED,        ///< System is stopped (initial state).
     RESETTING,      ///< All variables and hardware are reset.
     IDLE,           ///< Waiting state (e.g. until start button is pressed).
     STARTING,       ///< Initialization, actuator activation.
     EXECUTE,        ///< Active task execution sequence.
     COMPLETING,     ///< Final phase after task execution.
     COMPLETE,       ///< Tasks completed successfully.
     ABORTED,        ///< Operation was aborted.
     ABORTING        ///< Abort process active (e.g. due to timeout).
 };
 
 /**
  * @brief       Global variable holding the current top-level robot state.
  */
 extern SystemState system_state;
 
 // =============================================================================
 // Substates – ExecuteState (Active during EXECUTE)
 // =============================================================================
 
 /**
  * @brief       Sub-state machine used during EXECUTE phase.
  */
 enum ExecuteState {
     HOME,
     RUN_TO_WAIT_PICK,
     RUN_TO_PICK,
     PICK,
     RUN_TO_WAIT_PLACE,
     RUN_TO_PLACE,
     PLACE,
     MOVE_OUT,
     MOVE_IN,
     OPEN_FLAP,
     CLOSE_FLAP,
     OPEN_GRIPPER,
     CLOSE_GRIPPER,
 };
 
 /**
  * @brief       Current sub-state during EXECUTE.
  */
 extern ExecuteState execute_state;
 
 /**
  * @brief       History array to store previous EXECUTE substates.
  * @note        Must be defined and initialized in a global source file.
  */
 extern ExecuteState State_History[];
 
 #endif // SYSTEM_STATE_H_
 
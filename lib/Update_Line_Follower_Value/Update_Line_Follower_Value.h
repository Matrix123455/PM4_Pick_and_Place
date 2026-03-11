/**
 ******************************************************************************
 * @file        Update_Line_Follower_Value.h
 * @brief       Interface for line-following control.
 *
 * @details     Declares the interface for computing motor commands based on
 *              line position. Returns a structure containing left/right wheel
 *              velocities and detection status.
 *
 * @note        The implementation is provided in the corresponding .cpp file.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24b - PM2
 * @date        2025-04-18
 ******************************************************************************
 */

 #ifndef UPDATE_LINE_FOLLOWER_VALUE_H
 #define UPDATE_LINE_FOLLOWER_VALUE_H
 
 #include <stdbool.h>       ///< For boolean types
 #include <cstdint>         ///< For standard types (e.g., float)
 #include "Robot_Structs.h" ///< For LineFollowerOut structure
 
 // =============================================================================
 // Forward declarations & external references
 // =============================================================================
 
 /**
  * @brief Maximum velocity for fallback behavior (e.g., drive straight).
  *
  * @note  These constants are typically defined in main.cpp or robot_const.h.
  */
 extern const float MAX_MOTOR_VELOCITY;
 extern const float MAX_LINE_ANGLE_DEGREES;
 
 /**
  * @brief Global instance of the LineFollower object.
  *
  * @note This must be initialized in the main application (e.g., main.cpp).
  */
 class LineFollower;
 extern LineFollower lineFollower;
 
 // =============================================================================
 // Function prototype
 // =============================================================================
 
 /**
  * @brief Executes one iteration of the line-following logic.
  *
  * @param[in] b_Check_Line_Follow   Enables the line-following algorithm.
  * @return    LineFollowerOut       Structure with velocity commands and status flags.
  */
 LineFollowerOut line_follower(bool b_Check_Line_Follow);
 
 #endif // UPDATE_LINE_FOLLOWER_VALUE_H
 
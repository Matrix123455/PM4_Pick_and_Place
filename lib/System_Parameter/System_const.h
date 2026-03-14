/**
 ******************************************************************************
 * @file        Robot_const.h
 * @brief       Global constants and hardware parameters for the PM4 robot.
 *
 * @details     This header file contains all relevant constants for the central
 *              configuration of the robot. It includes parameters for:
 *              - Timing (main loop)
 *              - Motors (gear ratios, voltages)
 *              - Servos (limits, positions)
 *              - Line follower (sensor position, max speed)
 *
 *              This structure allows centralized maintenance and consistent
 *              parameter use across all software modules.
 *
 * @note        All constants are declared as **extern** and must be defined in
 *              a corresponding `.cpp` file (e.g. `Robot_const.cpp`).
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24a - PM4
 * @date        2026-03-11
 ******************************************************************************
 */

 #ifndef SYSTEM_CONST_H_
 #define SYSTEM_CONST_H_
 
 // =============================================================================
 // Timing – Main Task Period
 // =============================================================================
 
 /** @brief Cycle period of the main loop [ms]. */
 extern const int MAIN_TASK_PERIOD_MS;
 extern const float WAIT_START_TIME;
 extern const float WAIT_STOP_TIME;
 
 /** @brief Length of the state history array. */
 extern const int STATE_HISTORY_ARRAY_LENGTH;
 
 // =============================================================================
 // Encoder Configuration
 // =============================================================================
 
 extern const int TICKS_PER_ROTATION;            ///< Number of ticks per wheel rotation.
 extern const int MAX_ROTATIONS;                 ///< Maximum allowed wheel rotations.
 extern const int MAX_TICKS;                     ///< Max ticks during line search drive.
 
 // =============================================================================
 // Motor Parameters x - Axis, y - Axis, z - Axis
 // =============================================================================
 
 extern const float Ax_GEAR_RATIO;               ///< Gear ratio for Ax.
 extern const float Ax_KN;                       ///< Motor constant for Ax [RPM/V].
 extern const float Ax_VOLTAGE_MAX;              ///< Max voltage for Ax [V].
 
 extern const float Ay_GEAR_RATIO;               ///< Gear ratio for Ay.
 extern const float Ay_KN;                       ///< Motor constant for Ay [RPM/V].
 extern const float Ay_VOLTAGE_MAX;              ///< Max voltage for Ay [V].

 extern const float Az_GEAR_RATIO;               ///< Gear ratio for Az.
 extern const float Az_KN;                       ///< Motor constant for Az [RPM/V].
 extern const float Az_VOLTAGE_MAX;              ///< Max voltage for Az [V].
 
 // =============================================================================
 // Servo Parameters
 // =============================================================================
 
 // Servo open top flap
 extern const float OPEN_FLAP_SERVO_LEFT_ANG_MIN;               ///< Minimum pulse width.
 extern const float OPEN_FLAP_SERVO_LEFT_ANG_MAX;               ///< Maximum pulse width.
 extern const float OPEN_FLAP_SERVO_LEFT_MAX_ACC;               ///< Max acceleration.
 
 // Servo gripper
 extern const float GRIPPER_SERVO_RIGHT_ANG_MIN;                ///< Minimum pulse width.
 extern const float GRIPPER_SERVO_RIGHT_ANG_MAX;                ///< Maximum pulse width.
 extern const float GRIPPER_SERVO_RIGHT_MAX_ACC;                ///< Max acceleration.
 
 // Common Target Positions
 extern const float OPEN_FLAP_SERVO_CLOSE_POSITION;             ///< Fully closed (0.0f).
 extern const float OPEN_FLAP_SERVO_OPEN_POSITION;              ///< Fully open (1.0f).
 extern const float GRIPPER_SERVO_OPEN_POSITION;                ///< Linefollower fully lowered.
 extern const float GRIPPER_SERVO_CLOSE_POSITION;               ///< Linefollower middle position.

 /** @brief Duration for full servo movement [s]. */
 extern const float SERVO_MOVE_TIME;
 
 // =============================================================================
 // SCARA Kinematics Parameters
 // =============================================================================

 extern const float SCARA_LINK_1_LENGTH_MM;       ///< Length of arm 1 [mm]
 extern const float SCARA_LINK_2_LENGTH_MM;       ///< Length of arm 2 [mm]

 extern const float SCARA_AXIS_1_MIN_DEG;         ///< Minimum angle axis 1 [deg]
 extern const float SCARA_AXIS_1_MAX_DEG;         ///< Maximum angle axis 1 [deg]

 extern const float SCARA_AXIS_2_MIN_DEG;         ///< Minimum angle axis 2 [deg]
 extern const float SCARA_AXIS_2_MAX_DEG;         ///< Maximum angle axis 2 [deg]

 extern const float SCARA_AXIS_2_BLOCK_MIN_DEG;   ///< Forbidden range start [deg]
 extern const float SCARA_AXIS_2_BLOCK_MAX_DEG;   ///< Forbidden range end [deg]

 // =============================================================================
 // IR Sensor – Sensor Parameters
 // =============================================================================
 
 extern const float IRSENSOR_CALIBRATION_MAX;    ///< IR sensor calibration constant for maximum distance (e.g., close object) [a.u. or mm].
 extern const float IRSENSOR_CALIBRATION_MIN;    ///< IR sensor calibration constant for minimum distance (e.g., far object) [a.u. or mm].
 
 // =============================================================================
 // Other Sensor Parameters
 // =============================================================================
 
 extern const float AVG_FILTER_SIZE;             ///< Window size for moving average filter.
 
 // =============================================================================
 // Distance Parameters for Motion Logic
 // =============================================================================
 
 extern const float END_PLATE_DISTANCE;          ///< Distance from end of plate to back wall [mm].
 
 #endif // SYSTEM_CONST_H_
/**
 ******************************************************************************
 * @file        System_Kinematik_PathPlaning.h
 * @brief       Path planning for SCARA axis 2 motion.
 *
 * @details     This header file contains the data types and public function
 *              declarations for the path planning of the second SCARA axis.
 *
 *              The module evaluates whether a motion from a given start angle
 *              to a target angle is possible while respecting:
 *              - the configured global axis 2 limits
 *              - the configured forbidden blocked range
 *
 *              The planner can evaluate both motion directions:
 *              - clockwise (CW)
 *              - counter-clockwise (CCW)
 *
 *              If necessary, the motion may pass over the 0° / 360° boundary
 *              in order to avoid the blocked range.
 *
 * @note        All angles handled by this module are interpreted in degree.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24a - PM4
 * @date        2026-03-14
 ******************************************************************************
 */

#ifndef SYSTEM_KINEMATIK_PATHPLANING_H_
#define SYSTEM_KINEMATIK_PATHPLANING_H_

#include <stdbool.h>

// =============================================================================
// Data Types
// =============================================================================

/**
 * @brief Possible motion directions for axis 2.
 */
typedef enum
{
    AXIS_2_PATH_NONE = 0,   ///< No valid motion path found
    AXIS_2_PATH_CW,         ///< Clockwise motion
    AXIS_2_PATH_CCW         ///< Counter-clockwise motion

} Axis2PathDirection_t;

/**
 * @brief Result reason for axis 2 path planning.
 */
typedef enum
{
    AXIS_2_PATH_REASON_NONE = 0,              ///< No reason assigned
    AXIS_2_PATH_REASON_START_INVALID,         ///< Start angle invalid
    AXIS_2_PATH_REASON_TARGET_INVALID,        ///< Target angle invalid
    AXIS_2_PATH_REASON_BOTH_PATHS_BLOCKED,    ///< Neither CW nor CCW path valid
    AXIS_2_PATH_REASON_CW_SELECTED_SHORTER,   ///< Both valid, CW selected because shorter
    AXIS_2_PATH_REASON_CCW_SELECTED_SHORTER,  ///< Both valid, CCW selected because shorter
    AXIS_2_PATH_REASON_ONLY_CW_ALLOWED,       ///< Only clockwise path valid
    AXIS_2_PATH_REASON_ONLY_CCW_ALLOWED       ///< Only counter-clockwise path valid

} Axis2PathReason_t;

/**
 * @brief Planned motion result for SCARA axis 2.
 */
typedef struct
{
    float start_deg;                   ///< Start angle [deg]
    float target_deg;                  ///< Target angle [deg]

    bool valid_start;                  ///< true if start angle is valid
    bool valid_target;                 ///< true if target angle is valid
    bool motion_possible;              ///< true if a valid motion path exists

    Axis2PathDirection_t direction;    ///< Selected motion direction
    float delta_deg;                   ///< Angular travel of selected motion [deg]

    Axis2PathReason_t reason;          ///< Reason for selected result

} Axis2MotionPlan_t;

// =============================================================================
// Function Prototypes
// =============================================================================

/**
 * @brief Plans a valid motion for SCARA axis 2.
 *
 * @details This function checks whether a motion from the given start angle
 *          to the target angle is possible while avoiding the forbidden
 *          blocked range.
 *
 *          The function evaluates:
 *          - start angle validity
 *          - target angle validity
 *          - clockwise motion path
 *          - counter-clockwise motion path
 *
 *          If both directions are possible, the shorter one is selected.
 *
 * @param[in] start_deg    Start angle of axis 2 [deg]
 * @param[in] target_deg   Target angle of axis 2 [deg]
 *
 * @return Axis2MotionPlan_t
 */
Axis2MotionPlan_t SystemKinematik_PlanAxis2Motion(float start_deg, float target_deg);

#endif // SYSTEM_KINEMATIK_PATHPLANING_H_
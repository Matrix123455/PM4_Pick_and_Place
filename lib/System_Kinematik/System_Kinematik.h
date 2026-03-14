/**
 ******************************************************************************
 * @file        System_Kinematik.h
 * @brief       Inverse kinematics for the 2-axis SCARA robot.
 *
 * @details     This header file contains the data types and public function
 *              declarations for the inverse kinematics of a planar 2-axis
 *              SCARA robot.
 *
 *              Input:
 *              - Cartesian target position in mm
 *
 *              Output:
 *              - Axis 1 angle in degree
 *              - Axis 2 angle in degree
 *              - Valid flag indicating whether the target position is reachable
 *                and inside the configured axis limits
 *              - Reason code indicating why a solution was selected or rejected
 *
 *              The inverse kinematics module evaluates:
 *              - geometric workspace reachability
 *              - configured axis 1 limits
 *              - configured global axis 2 limits
 *              - configured forbidden blocked range for axis 2
 *
 * @note        All angles used by this module are interpreted in degree and
 *              normalized internally to the range 0.0° ... 360.0°.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24a - PM4
 * @date        2026-03-14
 ******************************************************************************
 */

#ifndef SYSTEM_KINEMATIK_H_
#define SYSTEM_KINEMATIK_H_

#include <stdbool.h>
#include "System_Structs.h"
#include "System_Coordinate_System_Mapping.h"

// =============================================================================
// Data Types
// =============================================================================

/**
 * @brief Result reason for SCARA inverse kinematics evaluation.
 */
typedef enum
{
    SCARA_IK_REASON_NONE = 0,                  ///< No reason assigned
    SCARA_IK_REASON_SOLUTION_A_SELECTED,       ///< Solution A selected as valid result
    SCARA_IK_REASON_SOLUTION_B_SELECTED,       ///< Solution B selected as valid result
    SCARA_IK_REASON_TARGET_OUTSIDE_WORKSPACE,  ///< Target lies outside the reachable workspace
    SCARA_IK_REASON_NO_VALID_SOLUTION,         ///< No valid inverse kinematic solution found
    SCARA_IK_REASON_AXIS_1_LIMIT_VIOLATION_A,  ///< Solution A violates axis 1 limits
    SCARA_IK_REASON_AXIS_2_LIMIT_VIOLATION_A,  ///< Solution A violates global axis 2 limits
    SCARA_IK_REASON_AXIS_2_BLOCKED_RANGE_A,    ///< Solution A lies inside blocked axis 2 range
    SCARA_IK_REASON_AXIS_1_LIMIT_VIOLATION_B,  ///< Solution B violates axis 1 limits
    SCARA_IK_REASON_AXIS_2_LIMIT_VIOLATION_B,  ///< Solution B violates global axis 2 limits
    SCARA_IK_REASON_AXIS_2_BLOCKED_RANGE_B     ///< Solution B lies inside blocked axis 2 range

} ScaraIkReason_t;

/**
 * @brief Calculated SCARA axis target position.
 */
typedef struct
{
    float axis_1_deg;         ///< Calculated target angle for axis 1 [deg]
    float axis_2_deg;         ///< Calculated target angle for axis 2 [deg]
    bool  valid;              ///< true = valid solution found, false = invalid target

    ScaraIkReason_t reason;   ///< Reason for selected result or rejection

    float q1_a_deg;           ///< Calculated axis 1 angle of solution A [deg]
    float q2_a_deg;           ///< Calculated axis 2 angle of solution A [deg]
    float q1_b_deg;           ///< Calculated axis 1 angle of solution B [deg]
    float q2_b_deg;           ///< Calculated axis 2 angle of solution B [deg]

} ScaraAxisPosition_t;

// =============================================================================
// Function Prototypes
// =============================================================================

/**
 * @brief Calculates the inverse kinematics for a SCARA target position.
 *
 * @details This function calculates the two axis angles required to reach
 *          the given cartesian target position in the XY plane.
 *
 *          The function checks:
 *          - workspace reachability
 *          - axis 1 limits
 *          - axis 2 global limits
 *          - axis 2 blocked forbidden range
 *
 *          If no valid solution exists, the return value contains
 *          valid = false and a corresponding reason code.
 *
 * @param[in] target   Cartesian target position [mm]
 *
 * @return ScaraAxisPosition_t
 */
ScaraAxisPosition_t SystemKinematik_GetAxisPosition(CartesianPosition_t target);

#endif // SYSTEM_KINEMATIK_H_
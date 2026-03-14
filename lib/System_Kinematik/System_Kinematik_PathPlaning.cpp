/**
 ******************************************************************************
 * @file        System_Kinematik_PathPlaning.cpp
 * @brief       Path planning for SCARA axis 2 motion.
 *
 * @details     This source file implements the path planning for the second
 *              rotational axis of the SCARA robot.
 *
 *              The module determines whether a motion from a given start angle
 *              to a target angle is possible while respecting:
 *              - the configured global axis 2 limits
 *              - the configured forbidden collision range
 *
 *              The planner evaluates both possible motion directions:
 *              - clockwise (CW)
 *              - counter-clockwise (CCW)
 *
 *              If both directions are possible, the shorter path is selected.
 *              If only one direction is possible, this direction is returned.
 *              If necessary, the motion may pass over the 0° / 360° boundary
 *              in order to avoid the blocked range.
 *
 * @note        All angles are internally normalized to the range
 *              0.0° ... 360.0°.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24a - PM4
 * @date        2026-03-14
 ******************************************************************************
 */

#include "System_Kinematik_PathPlaning.h"
#include "System_const.h"
#include <math.h>

// =============================================================================
// Local Function Prototypes
// =============================================================================

/**
 * @brief Normalizes an angle to the range 0...360 degree.
 *
 * @param[in] angle_deg   Input angle in degree
 *
 * @return Normalized angle in degree
 */
static float NormalizeAngle360(float angle_deg);

/**
 * @brief Checks whether an angle lies inside a circular interval.
 *
 * @details Supports:
 *          - normal ranges, e.g. 20° ... 150°
 *          - wrap-around ranges, e.g. 300° ... 40°
 *          - full-circle range, e.g. 0° ... 360°
 *
 * @param[in] angle_deg   Angle to check [deg]
 * @param[in] min_deg     Range start [deg]
 * @param[in] max_deg     Range end [deg]
 *
 * @return true if angle lies inside the interval, otherwise false
 */
static bool AngleInRange360(float angle_deg, float min_deg, float max_deg);

/**
 * @brief Checks whether an axis 2 angle is valid.
 *
 * @details The angle is valid when:
 *          - it lies inside the configured global axis 2 range
 *          - it does not lie inside the forbidden blocked range
 *
 * @param[in] angle_deg   Axis 2 angle [deg]
 *
 * @return true if valid, otherwise false
 */
static bool Axis2AngleValid(float angle_deg);

/**
 * @brief Calculates clockwise angular travel from start to target.
 *
 * @param[in] start_deg    Start angle [deg]
 * @param[in] target_deg   Target angle [deg]
 *
 * @return Clockwise angular distance [deg]
 */
static float ForwardDelta(float start_deg, float target_deg);

/**
 * @brief Calculates counter-clockwise angular travel from start to target.
 *
 * @param[in] start_deg    Start angle [deg]
 * @param[in] target_deg   Target angle [deg]
 *
 * @return Counter-clockwise angular distance [deg]
 */
static float BackwardDelta(float start_deg, float target_deg);

/**
 * @brief Checks whether the clockwise path is completely valid.
 *
 * @details The path is sampled in 1° steps. Every intermediate angle must
 *          remain valid and outside the forbidden blocked range.
 *
 * @param[in] start_deg    Start angle [deg]
 * @param[in] target_deg   Target angle [deg]
 *
 * @return true if the complete clockwise path is valid, otherwise false
 */
static bool PathValidCW(float start_deg, float target_deg);

/**
 * @brief Checks whether the counter-clockwise path is completely valid.
 *
 * @details The path is sampled in 1° steps. Every intermediate angle must
 *          remain valid and outside the forbidden blocked range.
 *
 * @param[in] start_deg    Start angle [deg]
 * @param[in] target_deg   Target angle [deg]
 *
 * @return true if the complete counter-clockwise path is valid, otherwise false
 */
static bool PathValidCCW(float start_deg, float target_deg);

// =============================================================================
// Public Functions
// =============================================================================

Axis2MotionPlan_t SystemKinematik_PlanAxis2Motion(float start_deg, float target_deg)
{
    Axis2MotionPlan_t movePlan;

    // Normalize input angles to the internal 0...360° representation
    start_deg  = NormalizeAngle360(start_deg);
    target_deg = NormalizeAngle360(target_deg);

    // Initialize return structure
    movePlan.start_deg       = start_deg;
    movePlan.target_deg      = target_deg;
    movePlan.valid_start     = Axis2AngleValid(start_deg);
    movePlan.valid_target    = Axis2AngleValid(target_deg);
    movePlan.motion_possible = false;
    movePlan.direction       = AXIS_2_PATH_NONE;
    movePlan.delta_deg       = 0.0f;
    movePlan.reason          = AXIS_2_PATH_REASON_NONE;

    // -------------------------------------------------------------------------
    // Validate start and target angle
    // -------------------------------------------------------------------------

    if (!movePlan.valid_start)
    {
        movePlan.reason = AXIS_2_PATH_REASON_START_INVALID;
        return movePlan;
    }

    if (!movePlan.valid_target)
    {
        movePlan.reason = AXIS_2_PATH_REASON_TARGET_INVALID;
        return movePlan;
    }

    // -------------------------------------------------------------------------
    // Check both motion directions
    // -------------------------------------------------------------------------

    const bool cw_possible  = PathValidCW(start_deg, target_deg);
    const bool ccw_possible = PathValidCCW(start_deg, target_deg);

    const float cw_delta  = ForwardDelta(start_deg, target_deg);
    const float ccw_delta = BackwardDelta(start_deg, target_deg);

    // -------------------------------------------------------------------------
    // If both paths are valid, select the shorter one
    // -------------------------------------------------------------------------

    if (cw_possible && ccw_possible)
    {
        movePlan.motion_possible = true;

        if (cw_delta <= ccw_delta)
        {
            movePlan.direction = AXIS_2_PATH_CW;
            movePlan.delta_deg = cw_delta;
            movePlan.reason    = AXIS_2_PATH_REASON_CW_SELECTED_SHORTER;
        }
        else
        {
            movePlan.direction = AXIS_2_PATH_CCW;
            movePlan.delta_deg = ccw_delta;
            movePlan.reason    = AXIS_2_PATH_REASON_CCW_SELECTED_SHORTER;
        }

        return movePlan;
    }

    // -------------------------------------------------------------------------
    // Only clockwise path valid
    // -------------------------------------------------------------------------

    if (cw_possible)
    {
        movePlan.motion_possible = true;
        movePlan.direction       = AXIS_2_PATH_CW;
        movePlan.delta_deg       = cw_delta;
        movePlan.reason          = AXIS_2_PATH_REASON_ONLY_CW_ALLOWED;
        return movePlan;
    }

    // -------------------------------------------------------------------------
    // Only counter-clockwise path valid
    // -------------------------------------------------------------------------

    if (ccw_possible)
    {
        movePlan.motion_possible = true;
        movePlan.direction       = AXIS_2_PATH_CCW;
        movePlan.delta_deg       = ccw_delta;
        movePlan.reason          = AXIS_2_PATH_REASON_ONLY_CCW_ALLOWED;
        return movePlan;
    }

    // -------------------------------------------------------------------------
    // No valid path found
    // -------------------------------------------------------------------------

    movePlan.reason = AXIS_2_PATH_REASON_BOTH_PATHS_BLOCKED;
    return movePlan;
}

// =============================================================================
// Local Functions
// =============================================================================

static float NormalizeAngle360(float angle_deg)
{
    float normalized = fmodf(angle_deg, 360.0f);

    if (normalized < 0.0f)
    {
        normalized += 360.0f;
    }

    return normalized;
}

static bool AngleInRange360(float angle_deg, float min_deg, float max_deg)
{
    angle_deg = NormalizeAngle360(angle_deg);

    const float raw_span = max_deg - min_deg;

    // Full circle case, e.g. 0° ... 360°
    if (fabsf(raw_span) >= 360.0f)
    {
        return true;
    }

    min_deg = NormalizeAngle360(min_deg);
    max_deg = NormalizeAngle360(max_deg);

    if (min_deg <= max_deg)
    {
        return (angle_deg >= min_deg) && (angle_deg <= max_deg);
    }
    else
    {
        return (angle_deg >= min_deg) || (angle_deg <= max_deg);
    }
}

static bool Axis2AngleValid(float angle_deg)
{
    angle_deg = NormalizeAngle360(angle_deg);

    // Check whether the angle is inside the global axis 2 limits
    const bool valid_global = AngleInRange360(
        angle_deg,
        SCARA_AXIS_2_MIN_DEG,
        SCARA_AXIS_2_MAX_DEG);

    // Check whether the angle lies inside the forbidden blocked range
    const bool in_blocked = AngleInRange360(
        angle_deg,
        SCARA_AXIS_2_BLOCK_MIN_DEG,
        SCARA_AXIS_2_BLOCK_MAX_DEG);

    return valid_global && (!in_blocked);
}

static float ForwardDelta(float start_deg, float target_deg)
{
    return NormalizeAngle360(target_deg - start_deg);
}

static float BackwardDelta(float start_deg, float target_deg)
{
    return NormalizeAngle360(start_deg - target_deg);
}

static bool PathValidCW(float start_deg, float target_deg)
{
    const float step_deg = 1.0f;
    const float delta = ForwardDelta(start_deg, target_deg);

    for (float s = 0.0f; s <= delta; s += step_deg)
    {
        const float a = NormalizeAngle360(start_deg + s);

        if (!Axis2AngleValid(a))
        {
            return false;
        }
    }

    return true;
}

static bool PathValidCCW(float start_deg, float target_deg)
{
    const float step_deg = 1.0f;
    const float delta = BackwardDelta(start_deg, target_deg);

    for (float s = 0.0f; s <= delta; s += step_deg)
    {
        const float a = NormalizeAngle360(start_deg - s);

        if (!Axis2AngleValid(a))
        {
            return false;
        }
    }

    return true;
}
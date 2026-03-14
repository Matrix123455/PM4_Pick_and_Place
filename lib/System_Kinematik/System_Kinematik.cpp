/**
 ******************************************************************************
 * @file        System_Kinematik.cpp
 * @brief       Inverse kinematics for the 2-axis SCARA robot.
 *
 * @details     This source file implements the inverse kinematics calculation
 *              for a planar 2-axis SCARA robot.
 *
 *              The calculation is based on:
 *              - link length of axis 1
 *              - link length of axis 2
 *              - cartesian target position X/Y
 *
 *              Two possible inverse kinematic solutions are calculated:
 *              - solution A
 *              - solution B
 *
 *              Both solutions are checked against:
 *              - the configured axis 1 limits
 *              - the configured global axis 2 limits
 *              - the configured forbidden blocked range of axis 2
 *
 *              All calculated angles are normalized to the range
 *              0.0° ... 360.0°.
 *
 *              The first valid solution is returned.
 *
 * @note        The Z coordinate of the cartesian input is ignored in this
 *              module because the current implementation only calculates the
 *              planar XY kinematics.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24a - PM4
 * @date        2026-03-14
 ******************************************************************************
 */

#include "System_Kinematik.h"
#include "System_const.h"
#include <math.h>

// =============================================================================
// Local Constants
// =============================================================================

/** @brief Constant value of pi as float. */
static const float PI_F = 3.14159265358979323846f;

// =============================================================================
// Local Function Prototypes
// =============================================================================

/**
 * @brief Converts an angle from radians to degree.
 *
 * @param[in] rad   Angle in radians
 *
 * @return Angle in degree
 */
static float RadToDeg(float rad);

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
 *          - full-circle ranges, e.g. 0° ... 360°
 *
 * @param[in] angle_deg   Angle to check [deg]
 * @param[in] min_deg     Range start [deg]
 * @param[in] max_deg     Range end [deg]
 *
 * @return true if angle lies inside the interval, otherwise false
 */
static bool AngleInRange360(float angle_deg, float min_deg, float max_deg);

/**
 * @brief Checks whether axis 1 angle is inside the configured limits.
 *
 * @param[in] q1_deg   Axis 1 angle [deg]
 *
 * @return true if valid, otherwise false
 */
static bool IsAxis1Valid(float q1_deg);

/**
 * @brief Checks whether axis 2 angle is inside the configured global range.
 *
 * @param[in] q2_deg   Axis 2 angle [deg]
 *
 * @return true if valid, otherwise false
 */
static bool IsAxis2InGlobalRange(float q2_deg);

/**
 * @brief Checks whether axis 2 angle lies inside the forbidden blocked range.
 *
 * @param[in] q2_deg   Axis 2 angle [deg]
 *
 * @return true if blocked, otherwise false
 */
static bool IsAxis2Blocked(float q2_deg);

/**
 * @brief Checks whether a complete SCARA axis solution is valid.
 *
 * @details A solution is valid when:
 *          - axis 1 lies inside the configured limits
 *          - axis 2 lies inside the configured global range
 *          - axis 2 does not lie inside the forbidden blocked range
 *
 * @param[in] q1_deg   Axis 1 angle [deg]
 * @param[in] q2_deg   Axis 2 angle [deg]
 *
 * @return true if valid, otherwise false
 */
static bool AxisLimitsValid(float q1_deg, float q2_deg);

// =============================================================================
// Public Functions
// =============================================================================

ScaraAxisPosition_t SystemKinematik_GetAxisPosition(CartesianPosition_t target)
{
    ScaraAxisPosition_t result;

    result.axis_1_deg = 0.0f;
    result.axis_2_deg = 0.0f;
    result.valid      = false;
    result.reason     = SCARA_IK_REASON_NONE;

    result.q1_a_deg = 0.0f;
    result.q2_a_deg = 0.0f;
    result.q1_b_deg = 0.0f;
    result.q2_b_deg = 0.0f;

    const float x  = target.x_mm;
    const float y  = target.y_mm;
    const float L1 = SCARA_LINK_1_LENGTH_MM;
    const float L2 = SCARA_LINK_2_LENGTH_MM;

    // -------------------------------------------------------------------------
    // Workspace check using the cosine law
    // -------------------------------------------------------------------------
    //
    // The SCARA arm consists of two rigid links:
    //   L1 = length of arm 1
    //   L2 = length of arm 2
    //
    // The target position forms a triangle together with L1 and L2.
    //
    // According to the law of cosines:
    //
    //   r² = L1² + L2² + 2 * L1 * L2 * cos(q2)
    //
    // Rearranged:
    //
    //   cos(q2) = (r² - L1² - L2²) / (2 * L1 * L2)
    //
    // If |cos(q2)| > 1, the target lies outside the reachable workspace.
    // -------------------------------------------------------------------------

    /** @brief Squared distance from robot base to target position [mm²]. */
    const float r2 = (x * x) + (y * y);

    /** @brief Cosine of the axis 2 angle from the cosine law. */
    float c2 = (r2 - (L1 * L1) - (L2 * L2)) / (2.0f * L1 * L2);

    if ((c2 < -1.0f) || (c2 > 1.0f))
    {
        result.reason = SCARA_IK_REASON_TARGET_OUTSIDE_WORKSPACE;
        return result;
    }

    // Numerical safety clamp
    if (c2 > 1.0f)  c2 = 1.0f;
    if (c2 < -1.0f) c2 = -1.0f;

    // -------------------------------------------------------------------------
    // Two inverse kinematic solutions
    // -------------------------------------------------------------------------
    //
    // Every reachable target normally has two possible joint configurations:
    //
    //   solution A: q2 = +acos(c2)
    //   solution B: q2 = -acos(c2)
    //
    // Both solutions must be checked because mechanical limits may allow only one.
    // -------------------------------------------------------------------------

    /** @brief Axis 2 angle of inverse kinematic solution A [rad]. */
    const float q2_a = acosf(c2);

    /** @brief Axis 2 angle of inverse kinematic solution B [rad]. */
    const float q2_b = -acosf(c2);

    // -------------------------------------------------------------------------
    // Calculate q1 for solution A
    // -------------------------------------------------------------------------
    //
    // q1 is calculated from:
    //
    //   q1 = atan2(y, x) - atan2(k2, k1)
    //
    // with:
    //
    //   k1 = L1 + L2 * cos(q2)
    //   k2 = L2 * sin(q2)
    // -------------------------------------------------------------------------

    /** @brief Intermediate x projection for solution A. */
    const float k1_a = L1 + L2 * cosf(q2_a);

    /** @brief Intermediate y projection for solution A. */
    const float k2_a = L2 * sinf(q2_a);

    /** @brief Axis 1 angle of inverse kinematic solution A [rad]. */
    const float q1_a = atan2f(y, x) - atan2f(k2_a, k1_a);

    // -------------------------------------------------------------------------
    // Calculate q1 for solution B
    // -------------------------------------------------------------------------

    /** @brief Intermediate x projection for solution B. */
    const float k1_b = L1 + L2 * cosf(q2_b);

    /** @brief Intermediate y projection for solution B. */
    const float k2_b = L2 * sinf(q2_b);

    /** @brief Axis 1 angle of inverse kinematic solution B [rad]. */
    const float q1_b = atan2f(y, x) - atan2f(k2_b, k1_b);

    // -------------------------------------------------------------------------
    // Convert to degree and normalize to 0...360
    // -------------------------------------------------------------------------

    /** @brief Axis 1 angle of solution A [deg]. */
    const float q1_a_deg = NormalizeAngle360(RadToDeg(q1_a));

    /** @brief Axis 2 angle of solution A [deg]. */
    const float q2_a_deg = NormalizeAngle360(RadToDeg(q2_a));

    /** @brief Axis 1 angle of solution B [deg]. */
    const float q1_b_deg = NormalizeAngle360(RadToDeg(q1_b));

    /** @brief Axis 2 angle of solution B [deg]. */
    const float q2_b_deg = NormalizeAngle360(RadToDeg(q2_b));

    result.q1_a_deg = q1_a_deg;
    result.q2_a_deg = q2_a_deg;
    result.q1_b_deg = q1_b_deg;
    result.q2_b_deg = q2_b_deg;

    // -------------------------------------------------------------------------
    // Validate both solutions
    // -------------------------------------------------------------------------

    const bool valid_a = AxisLimitsValid(q1_a_deg, q2_a_deg);
    const bool valid_b = AxisLimitsValid(q1_b_deg, q2_b_deg);

    // -------------------------------------------------------------------------
    // Return first valid solution
    // -------------------------------------------------------------------------

    if (valid_a)
    {
        result.axis_1_deg = q1_a_deg;
        result.axis_2_deg = q2_a_deg;
        result.valid      = true;
        result.reason     = SCARA_IK_REASON_SOLUTION_A_SELECTED;
        return result;
    }

    if (valid_b)
    {
        result.axis_1_deg = q1_b_deg;
        result.axis_2_deg = q2_b_deg;
        result.valid      = true;
        result.reason     = SCARA_IK_REASON_SOLUTION_B_SELECTED;
        return result;
    }

    // -------------------------------------------------------------------------
    // Determine rejection reason for solution A or B
    // -------------------------------------------------------------------------

    if (!IsAxis1Valid(q1_a_deg))
    {
        result.reason = SCARA_IK_REASON_AXIS_1_LIMIT_VIOLATION_A;
        return result;
    }

    if (!IsAxis2InGlobalRange(q2_a_deg))
    {
        result.reason = SCARA_IK_REASON_AXIS_2_LIMIT_VIOLATION_A;
        return result;
    }

    if (IsAxis2Blocked(q2_a_deg))
    {
        result.reason = SCARA_IK_REASON_AXIS_2_BLOCKED_RANGE_A;
        return result;
    }

    if (!IsAxis1Valid(q1_b_deg))
    {
        result.reason = SCARA_IK_REASON_AXIS_1_LIMIT_VIOLATION_B;
        return result;
    }

    if (!IsAxis2InGlobalRange(q2_b_deg))
    {
        result.reason = SCARA_IK_REASON_AXIS_2_LIMIT_VIOLATION_B;
        return result;
    }

    if (IsAxis2Blocked(q2_b_deg))
    {
        result.reason = SCARA_IK_REASON_AXIS_2_BLOCKED_RANGE_B;
        return result;
    }

    result.reason = SCARA_IK_REASON_NO_VALID_SOLUTION;
    return result;
}

// =============================================================================
// Local Functions
// =============================================================================

static float RadToDeg(float rad)
{
    return rad * 180.0f / PI_F;
}

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

static bool IsAxis1Valid(float q1_deg)
{
    return AngleInRange360(q1_deg, SCARA_AXIS_1_MIN_DEG, SCARA_AXIS_1_MAX_DEG);
}

static bool IsAxis2InGlobalRange(float q2_deg)
{
    return AngleInRange360(q2_deg, SCARA_AXIS_2_MIN_DEG, SCARA_AXIS_2_MAX_DEG);
}

static bool IsAxis2Blocked(float q2_deg)
{
    return AngleInRange360(q2_deg, SCARA_AXIS_2_BLOCK_MIN_DEG, SCARA_AXIS_2_BLOCK_MAX_DEG);
}

static bool AxisLimitsValid(float q1_deg, float q2_deg)
{
    const bool valid_q1 = IsAxis1Valid(q1_deg);
    const bool valid_q2 = IsAxis2InGlobalRange(q2_deg) && (!IsAxis2Blocked(q2_deg));

    return valid_q1 && valid_q2;
}
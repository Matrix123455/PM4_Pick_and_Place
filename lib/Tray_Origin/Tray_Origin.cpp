/**
 ******************************************************************************
 * @file        Tray_Origin.cpp
 * @brief       Coordinate mapping for tray and holder positions.
 *
 * @details     This module provides helper functions for calculating absolute
 *              Cartesian positions of tray slots and the sample holder.
 *
 *              The tray positions are not stored individually. Instead, they
 *              are generated from:
 *              - tray origin coordinates
 *              - row/column offsets
 *              - pick and safe heights
 *
 *              This simplifies calibration and allows easy adjustment if tray
 *              geometry changes.
 *
 * @note        The corresponding declarations are provided in Tray_Origin.h.
 *              All constants used here must be defined externally.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24a - PM4
 * @date        2026-03-11
 ******************************************************************************
 */

#include "Tray_Origin.h"

// =============================================================================
// Internal Helper Function for Tray Origin
// =============================================================================
static CartesianPosition_t GetTrayOrigin(TrayType_t tray)
{
    CartesianPosition_t origin;

    switch (tray)
    {
        case INPUT_TRAY:
            origin.x_mm = INPUT_TRAY_ORIGIN_X_MM;
            origin.y_mm = INPUT_TRAY_ORIGIN_Y_MM;
            origin.z_mm = INPUT_TRAY_ORIGIN_Z_MM;
            break;

        case OUTPUT_TRAY:
            origin.x_mm = OUTPUT_TRAY_ORIGIN_X_MM;
            origin.y_mm = OUTPUT_TRAY_ORIGIN_Y_MM;
            origin.z_mm = OUTPUT_TRAY_ORIGIN_Z_MM;
            break;

        case OUTPUT_BAD_TRAY:
            origin.x_mm = OUTPUT_BAD_TRAY_ORIGIN_X_MM;
            origin.y_mm = OUTPUT_BAD_TRAY_ORIGIN_Y_MM;
            origin.z_mm = OUTPUT_BAD_TRAY_ORIGIN_Z_MM;
            break;

        default:
            origin.x_mm = 0.0f;
            origin.y_mm = 0.0f;
            origin.z_mm = 0.0f;
            break;
    }

    return origin;
}

// =============================================================================
// Coordinate Mapping Functions
// =============================================================================
CartesianPosition_t GetTrayPickPosition(TrayType_t tray, int row, int column)
{
    CartesianPosition_t pos = {0.0f, 0.0f, 0.0f};

    if (!IsValidTrayIndex(row, column))
    {
        return pos;
    }

    CartesianPosition_t origin = GetTrayOrigin(tray);

    pos.x_mm = origin.x_mm + (static_cast<float>(column) * TRAY_OFFSET_X_MM);
    pos.y_mm = origin.y_mm + (static_cast<float>(row)    * TRAY_OFFSET_Y_MM);
    pos.z_mm = origin.z_mm + TRAY_PICK_HEIGHT_MM;

    return pos;
}

CartesianPosition_t GetTraySafePosition(TrayType_t tray, int row, int column)
{
    CartesianPosition_t pos = {0.0f, 0.0f, 0.0f};

    if (!IsValidTrayIndex(row, column))
    {
        return pos;
    }

    CartesianPosition_t origin = GetTrayOrigin(tray);

    pos.x_mm = origin.x_mm + (static_cast<float>(column) * TRAY_OFFSET_X_MM);
    pos.y_mm = origin.y_mm + (static_cast<float>(row)    * TRAY_OFFSET_Y_MM);
    pos.z_mm = origin.z_mm + TRAY_SAFE_HEIGHT_MM;

    return pos;
}

CartesianPosition_t GetHolderPlacePosition(void)
{
    CartesianPosition_t pos;

    pos.x_mm = HOLDER_PLACE_X_MM;
    pos.y_mm = HOLDER_PLACE_Y_MM;
    pos.z_mm = HOLDER_PLACE_Z_MM;

    return pos;
}

CartesianPosition_t GetHolderSafePosition(void)
{
    CartesianPosition_t pos;

    pos.x_mm = HOLDER_PLACE_X_MM;
    pos.y_mm = HOLDER_PLACE_Y_MM;
    pos.z_mm = HOLDER_SAFE_HEIGHT_MM;

    return pos;
}

bool IsValidTrayIndex(int row, int column)
{
    return ((row >= 0) && (row < TRAY_ROWS) &&
            (column >= 0) && (column < TRAY_COLUMNS));
}
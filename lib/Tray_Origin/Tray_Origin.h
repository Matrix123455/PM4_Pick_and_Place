/**
 ******************************************************************************
 * @file        Tray_Origin.h
 * @brief       Interface for tray origin and coordinate mapping.
 *
 * @details     This header provides the interface for calculating tray based
 *              coordinates used by the pick-and-place system.
 *
 *              The tray positions are generated dynamically using:
 *              - tray origin coordinates
 *              - row and column offsets
 *              - defined pick and safe heights
 *
 *              The actual implementation of the coordinate calculations is
 *              provided in Tray_Origin.cpp.
 *
 * @note        Only function prototypes and extern constants are defined here.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24a - PM4
 * @date        2026-03-11
 ******************************************************************************
 */

#ifndef TRAY_ORIGIN_H
#define TRAY_ORIGIN_H

#include <stdint.h>

// =============================================================================
// Data Types
// =============================================================================

/**
 * @brief Cartesian coordinate representation in millimeters.
 */
struct CartesianPosition_t
{
    float x_mm;
    float y_mm;
    float z_mm;
};

/**
 * @brief Tray index representing a specific slot in the tray.
 */
struct TrayIndex_t
{
    int row;
    int column;
};

/**
 * @brief Tray type selection.
 */
enum TrayType_t
{
    INPUT_TRAY,
    OUTPUT_TRAY,
    OUTPUT_BAD_TRAY
};

// =============================================================================
// Global Tray Configuration (extern definitions)
// =============================================================================

extern const int TRAY_ROWS;
extern const int TRAY_COLUMNS;

extern const float INPUT_TRAY_ORIGIN_X_MM;
extern const float INPUT_TRAY_ORIGIN_Y_MM;
extern const float INPUT_TRAY_ORIGIN_Z_MM;

extern const float OUTPUT_TRAY_ORIGIN_X_MM;
extern const float OUTPUT_TRAY_ORIGIN_Y_MM;
extern const float OUTPUT_TRAY_ORIGIN_Z_MM;

extern const float OUTPUT_BAD_TRAY_ORIGIN_X_MM;
extern const float OUTPUT_BAD_TRAY_ORIGIN_Y_MM;
extern const float OUTPUT_BAD_TRAY_ORIGIN_Z_MM;

extern const float TRAY_OFFSET_X_MM;
extern const float TRAY_OFFSET_Y_MM;

extern const float TRAY_PICK_HEIGHT_MM;
extern const float TRAY_SAFE_HEIGHT_MM;

// =============================================================================
// Sample Holder Mapping
// =============================================================================

extern const float HOLDER_PLACE_X_MM;
extern const float HOLDER_PLACE_Y_MM;
extern const float HOLDER_PLACE_Z_MM;

extern const float HOLDER_SAFE_HEIGHT_MM;

// =============================================================================
// Motion Safety
// =============================================================================

extern const float GLOBAL_SAFE_HEIGHT_MM;

// =============================================================================
// Function Prototypes
// =============================================================================

/**
 * @brief Returns the pick position of a vial in a tray.
 *
 * @param tray    Tray type (input/output/bad)
 * @param row     Row index
 * @param column  Column index
 * @return CartesianPosition_t Absolute XYZ position
 */
CartesianPosition_t GetTrayPickPosition(TrayType_t tray,
                                         int row,
                                         int column);

/**
 * @brief Returns the safe position above a tray slot.
 *
 * @param tray    Tray type
 * @param row     Row index
 * @param column  Column index
 * @return CartesianPosition_t Absolute XYZ position
 */
CartesianPosition_t GetTraySafePosition(TrayType_t tray,
                                         int row,
                                         int column);

/**
 * @brief Returns the place position of the sample holder.
 */
CartesianPosition_t GetHolderPlacePosition(void);

/**
 * @brief Returns the safe position above the sample holder.
 */
CartesianPosition_t GetHolderSafePosition(void);

/**
 * @brief Checks if a tray index is valid.
 *
 * @param row     Row index
 * @param column  Column index
 * @return true if valid
 */
bool IsValidTrayIndex(int row, int column);

#endif // TRAY_ORIGIN_H
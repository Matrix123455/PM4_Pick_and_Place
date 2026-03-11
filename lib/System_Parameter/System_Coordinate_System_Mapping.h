/**
 ******************************************************************************
 * @file        System_Coordinate_System_Mapping.h
 * @brief       Coordinate system mapping and position generation for the PM4 system.
 *
 * @details     This module contains all constants and helper functions required
 *              for coordinate mapping of the pick-and-place system.
 *
 *              The tray positions are not stored as fixed absolute coordinates.
 *              Instead, they are calculated from:
 *              - one tray origin
 *              - row/column offsets
 *              - defined pick/place heights
 *
 *              This makes calibration easier and allows simple adaptation if
 *              the tray geometry changes.
 *
 * @note        All constants are declared as extern and must be defined in
 *              System_Coordinate_System_Mapping.cpp.
 *
 * @author      Christof Meier, ZHAW School of Engineering, ST24a - PM4
 * @date        2026-03-11
 ******************************************************************************
 */

#ifndef SYSTEM_COORDINATE_SYSTEM_MAPPING_H_
#define SYSTEM_COORDINATE_SYSTEM_MAPPING_H_

#include <stdint.h>

// =============================================================================
// Data Types
// =============================================================================

struct CartesianPosition_t
{
    float x_mm;   ///< X position in mm
    float y_mm;   ///< Y position in mm
    float z_mm;   ///< Z position in mm
};

struct TrayIndex_t
{
    uint8_t row;      ///< Tray row index
    uint8_t column;   ///< Tray column index
};

// =============================================================================
// Global Coordinate System
// =============================================================================

extern const float BASE_ORIGIN_X_MM;            ///< Global base origin X [mm]
extern const float BASE_ORIGIN_Y_MM;            ///< Global base origin Y [mm]
extern const float BASE_ORIGIN_Z_MM;            ///< Global base origin Z [mm]

// =============================================================================
// Tray Geometry and Mapping
// =============================================================================

extern const uint8_t TRAY_ROWS;                 ///< Number of tray rows
extern const uint8_t TRAY_COLUMNS;              ///< Number of tray columns

extern const float TRAY_ORIGIN_X_MM;            ///< X coordinate of tray reference position [mm]
extern const float TRAY_ORIGIN_Y_MM;            ///< Y coordinate of tray reference position [mm]
extern const float TRAY_ORIGIN_Z_MM;            ///< Z coordinate of tray top reference [mm]

extern const float TRAY_OFFSET_X_MM;            ///< Column spacing in X direction [mm]
extern const float TRAY_OFFSET_Y_MM;            ///< Row spacing in Y direction [mm]

extern const float TRAY_PICK_HEIGHT_MM;         ///< Z height for picking vial [mm]
extern const float TRAY_SAFE_HEIGHT_MM;         ///< Safe Z height above tray [mm]

// =============================================================================
// Sample Holder / Device Mapping
// =============================================================================

extern const float HOLDER_PLACE_X_MM;           ///< X coordinate of sample holder place position [mm]
extern const float HOLDER_PLACE_Y_MM;           ///< Y coordinate of sample holder place position [mm]
extern const float HOLDER_PLACE_Z_MM;           ///< Z coordinate of sample holder place position [mm]

extern const float HOLDER_SAFE_HEIGHT_MM;       ///< Safe Z height above sample holder [mm]

// =============================================================================
// Global Motion Safety Parameters
// =============================================================================

extern const float GLOBAL_SAFE_HEIGHT_MM;       ///< General safe travel height [mm]

// =============================================================================
// Other Sensor Parameters
// =============================================================================

extern const float AVG_FILTER_SIZE;             ///< Window size for moving average filter.

// =============================================================================
// Distance Parameters for Motion Logic
// =============================================================================

extern const float END_PLATE_DISTANCE;          ///< Distance from end of plate to back wall [mm]

// =============================================================================
// Function Prototypes
// =============================================================================

/**
 * @brief   Returns the absolute pick position of a vial in the tray.
 *
 * @param   row     Tray row index
 * @param   column  Tray column index
 * @return  CartesianPosition_t with absolute XYZ coordinates
 */
CartesianPosition_t GetTrayPickPosition(uint8_t row, uint8_t column);

/**
 * @brief   Returns the safe position above a vial in the tray.
 *
 * @param   row     Tray row index
 * @param   column  Tray column index
 * @return  CartesianPosition_t with absolute XYZ coordinates
 */
CartesianPosition_t GetTraySafePosition(uint8_t row, uint8_t column);

/**
 * @brief   Returns the place position of the sample holder.
 *
 * @return  CartesianPosition_t with absolute XYZ coordinates
 */
CartesianPosition_t GetHolderPlacePosition(void);

/**
 * @brief   Returns the safe position above the sample holder.
 *
 * @return  CartesianPosition_t with absolute XYZ coordinates
 */
CartesianPosition_t GetHolderSafePosition(void);

/**
 * @brief   Checks whether the tray index is valid.
 *
 * @param   row     Tray row index
 * @param   column  Tray column index
 * @return  true if valid, otherwise false
 */
bool IsValidTrayIndex(uint8_t row, uint8_t column);

#endif // SYSTEM_COORDINATE_SYSTEM_MAPPING_H_
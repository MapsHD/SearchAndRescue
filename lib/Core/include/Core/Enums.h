#pragma once

#include <cstdint>

// Camera projection type : perspective (fov based) or orthographic (parallel, box frustum)
enum class ProjectionType : int
{
    PROJECTION_TYPE_PERSPECTIVE  = 0,
    PROJECTION_TYPE_ORTHOGRAPHIC = 1
};

enum class CameraMode : int
{
    CAMERA_MODE_FREE_ORBIT = 0,
    CAMERA_MODE_AXIS_X     = 1,
    CAMERA_MODE_AXIS_NX    = 2,
    CAMERA_MODE_AXIS_Y     = 3,
    CAMERA_MODE_AXIS_NY    = 4,
    CAMERA_MODE_AXIS_Z     = 5,
    CAMERA_MODE_AXIS_NZ    = 6,
    CAMERA_MODE_LOCAL_X    = 7,
    CAMERA_MODE_LOCAL_NX   = 8,
    CAMERA_MODE_LOCAL_Y    = 9,
    CAMERA_MODE_LOCAL_NY   = 10,
    CAMERA_MODE_LOCAL_Z    = 11,
    CAMERA_MODE_LOCAL_NZ   = 12
};

enum class ViewportCount : int
{
    VIEWPORT_COUNT_ONE  = 1,
    VIEWPORT_COUNT_TWO  = 2,
    VIEWPORT_COUNT_FOUR = 4
};

enum class PointCloudDisplayMode : int32_t
{
    POINT_CLOUD_DISPLAY_MODE_INTENSITY                          = 0,
    POINT_CLOUD_DISPLAY_MODE_COLOR_MAP                          = 1,
    POINT_CLOUD_DISPLAY_MODE_COLOR_MAP_TIMES_INTENSITY          = 2,
    POINT_CLOUD_DISPLAY_MODE_COLOR_MAP_POSITION                 = 3,
    POINT_CLOUD_DISPLAY_MODE_COLOR_MAP_POSITION_TIMES_INTENSITY = 4
};

enum class ColorMapType : int32_t
{
    COLOR_MAP_TYPE_TURBO   = 0,
    COLOR_MAP_TYPE_VIRIDIS = 1,
    COLOR_MAP_TYPE_PLASMA  = 2,
    COLOR_MAP_TYPE_MAGMA   = 3,
    COLOR_MAP_TYPE_INFERNO = 4
};

enum class TrajectoryDisplayMode : int32_t
{
    TRAJECTORY_DISPLAY_MODE_LINE_STRIP = 0,
    TRAJECTORY_DISPLAY_MODE_POINTS     = 1
};

// How the radius of a pick tolerance is defined
enum class PickToleranceMode : int32_t
{
    PICK_TOLERANCE_MODE_WORLD   = 0, // fixed radius in metres
    PICK_TOLERANCE_MODE_SCREEN  = 1, // radius in screen pixels, grows with the view depth
    PICK_TOLERANCE_MODE_LARGEST = 2  // the larger of the two
};

enum class TrajectoryCsvLayout : int
{
    TRAJECTORY_CSV_LAYOUT_UNKNOWN            = 0,
    TRAJECTORY_CSV_LAYOUT_MAT33_2_TIMESTAMPS = 1, // TS1, TS2, x, y, z, r00..r22  (14 columns)
    TRAJECTORY_CSV_LAYOUT_MAT33_1_TIMESTAMP  = 2, // TS1,      x, y, z, r00..r22  (13 columns)
    TRAJECTORY_CSV_LAYOUT_QUAT_2_TIMESTAMPS  = 3, // TS1, TS2, x, y, z, qx,qy,qz,qw (9 columns)
    TRAJECTORY_CSV_LAYOUT_QUAT_1_TIMESTAMP   = 4  // TS1,      x, y, z, qx,qy,qz,qw (8 columns)
};

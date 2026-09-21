#pragma once

#include "rayneo_api.h"

inline const char* RayneoExampleDeviceName(const RAYNEO_VidPid& device)
{
    // Air 3S Pro and Air 4 Pro share this USB identity.
    if (device.vid == RAYNEO_AIR_3S_PRO_VID && device.pid == RAYNEO_AIR_3S_PRO_PID)
        return "RayNeo Air 3S Pro / Air 4 Pro";
    if (device.vid == RAYNEO_GT_VID && device.pid == RAYNEO_GT_PID)
        return "RayNeo GT";
    return "Unknown RayNeo model";
}

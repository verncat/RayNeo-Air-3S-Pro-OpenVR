// RayNeo C API
// Public low-level interface for building and sending 64-byte protocol frames,
// managing a background service thread, and receiving events via callback or polling.
// Versioning: MAJOR changes break ABI, MINOR are backward compatible additions.
// Packed version format: (MAJOR << 16) | (MINOR & 0xFFFF)

#pragma once

#ifdef _WIN32
 #ifdef RAYNEO_BUILD
  #define RAYNEO_API __declspec(dllexport)
 #else
  #define RAYNEO_API __declspec(dllimport)
 #endif
#else
 #define RAYNEO_API
#endif

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Versioning ----
#define RAYNEO_API_VERSION_MAJOR 1
// Minor 1: added structured fields to RAYNEO_DeviceInfoMini (backward compatible: raw[] still first)
// Minor 2: added RAYNEO_EVENT_NOTIFY (sleep/wake notifications) and notify union member
#define RAYNEO_API_VERSION_MINOR 2 // 1.2: added RAYNEO_EVENT_NOTIFY (sleep/wake) and notify union member
#define RAYNEO_API_VERSION ((RAYNEO_API_VERSION_MAJOR << 16) | (RAYNEO_API_VERSION_MINOR & 0xFFFF))

RAYNEO_API unsigned int Rayneo_GetApiVersion(void); // returns packed version

typedef struct RayneoContext__* RAYNEO_Context; // opaque handle

typedef enum RAYNEO_Result {
    RAYNEO_OK = 0,
    RAYNEO_ERR_GENERAL      = -1,
    RAYNEO_ERR_NO_DEVICE    = -2,
    RAYNEO_ERR_TIMEOUT      = -3,
    RAYNEO_ERR_IO           = -4,
    RAYNEO_ERR_BUSY         = -5,
    RAYNEO_ERR_UNSUPPORTED  = -6,
    RAYNEO_ERR_INVALID_ARG  = -7,
    RAYNEO_ERR_QUEUE_FULL   = -8
} RAYNEO_Result;

// Service flags (bitmask)
typedef enum RAYNEO_ServiceFlags {
    RAYNEO_SERVICE_FLAG_AUTODETECT = 1u << 0,
    RAYNEO_SERVICE_FLAG_ASYNC_IMU  = 1u << 1
} RAYNEO_ServiceFlags;

// Event types produced by the service thread.
typedef enum RAYNEO_EventType {
    RAYNEO_EVENT_DEVICE_ATTACHED = 0,
    RAYNEO_EVENT_DEVICE_DETACHED = 1,
    RAYNEO_EVENT_IMU_SAMPLE      = 2,
    RAYNEO_EVENT_DEVICE_INFO     = 3,
    RAYNEO_EVENT_ERROR           = 4,
    RAYNEO_EVENT_LOG             = 5,
    RAYNEO_EVENT_NOTIFY          = 6,  // new in 1.2 (e.g. sleep/wake)
    RAYNEO_NOTIFY_SLEEP          = 7,
    RAYNEO_NOTIFY_WAKE           = 8,
    RAYNEO_NOTIFY_BUTTON         = 9
} RAYNEO_EventType;

typedef struct RAYNEO_ImuSample {
    float    acc[3];
    float    gyroDps[3];
    float    gyroRad[3];
    float    magnet[3];
    float    temperature;
    float    psensor;
    float    lsensor;
    uint32_t tick;
    uint32_t count;
    uint8_t  flag;
    uint8_t  checksum;
    uint8_t  valid;      // 1 if filled
    uint8_t  reserved;   // future use
} RAYNEO_ImuSample;

// Device info (mini) returned from command 0x00 (type 0xC8 ack). Originally only raw[60] was exposed.
// Minor version 1 adds decoded fields after the original prefix for convenience.
typedef struct vyt  {
    uint8_t  raw[60];      // raw payload bytes (unchanged for backward compatibility)
    uint8_t  valid;        // 1 if filled
    uint8_t  reserved[3];  // kept for alignment with original 1.0 layout
    // --- Decoded fields (added in 1.1). If parsing failed they remain zeroed. ---
    uint32_t tick;
    uint8_t  value;
    uint8_t  cpuid[12];
    uint8_t  board_id;
    uint8_t  sensor_on;
    uint8_t  support_fov;
    char     date[13];     // null-terminated
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  glasses_fps;
    uint8_t  luminance;
    uint8_t  volume;
    uint8_t  side_by_side;
    uint8_t  psensor_enable;
    uint8_t  audio_mode;
    uint8_t  dp_status;
    uint8_t  status3;
    uint8_t  psensor_valid;
    uint8_t  lsensor_valid;
    uint8_t  gyro_valid;
    uint8_t  magnet_valid;
    float    reserve1;
    float    reserve2;
    uint8_t  max_luminance;
    uint8_t  max_volume;
    uint8_t  support_panel_color_adjust;
    uint8_t  flag;
} RAYNEO_DeviceInfoMini;

typedef struct RAYNEO_Event {
    RAYNEO_EventType type;
    uint64_t         seq; // monotonically increasing sequence number
    union {
        RAYNEO_ImuSample      imu;
        RAYNEO_DeviceInfoMini info;
        struct { int code; }  error;
        struct { int level; char message[96]; } log;
        struct { int code; char message[96]; } notify; // new in 1.2
    } data;
} RAYNEO_Event;

typedef void (*RAYNEO_EventCallback)(const RAYNEO_Event* evt, void* user);

// Human-readable string for a result code.
RAYNEO_API const char* Rayneo_ResultToString(RAYNEO_Result r);

// Lifecycle
RAYNEO_API RAYNEO_Result Rayneo_Create(RAYNEO_Context* outCtx);
RAYNEO_API void          Rayneo_Destroy(RAYNEO_Context ctx);

// Configuration (call before Start). If unset, defaults may be used.
RAYNEO_API RAYNEO_Result Rayneo_SetTargetVidPid(RAYNEO_Context ctx, uint16_t vid, uint16_t pid);
RAYNEO_API RAYNEO_Result Rayneo_SetEventCallback(RAYNEO_Context ctx, RAYNEO_EventCallback cb, void* user);
RAYNEO_API RAYNEO_Result Rayneo_SetLogLevel(int level); // global/simple; 0=errors .. 3=debug

// Service control
RAYNEO_API RAYNEO_Result Rayneo_Start(RAYNEO_Context ctx, uint32_t serviceFlags);
RAYNEO_API RAYNEO_Result Rayneo_Stop(RAYNEO_Context ctx);

// Optional polling (alternative to callback). timeoutMs=0 => non-blocking.
RAYNEO_API RAYNEO_Result Rayneo_PollEvent(RAYNEO_Context ctx, RAYNEO_Event* outEvent, uint32_t timeoutMs);

// Raw protocol I/O (64-byte frames)
RAYNEO_API RAYNEO_Result Rayneo_SendRaw(RAYNEO_Context ctx, const uint8_t frame64[64]);
RAYNEO_API RAYNEO_Result Rayneo_SendCommand(RAYNEO_Context ctx,
                                            uint8_t command, uint8_t value,
                                            const void* payload, size_t payloadLen);
RAYNEO_API RAYNEO_Result Rayneo_RoundTrip(RAYNEO_Context ctx,
                                          uint8_t command, uint8_t value,
                                          const void* payload, size_t payloadLen,
                                          uint32_t timeoutMs,
                                          uint8_t outFrame[64]);

// Convenience commands (thin wrappers around SendCommand)
RAYNEO_API RAYNEO_Result Rayneo_EnableImu(RAYNEO_Context ctx);
RAYNEO_API RAYNEO_Result Rayneo_DisableImu(RAYNEO_Context ctx);
RAYNEO_API RAYNEO_Result Rayneo_RequestDeviceInfo(RAYNEO_Context ctx);
RAYNEO_API RAYNEO_Result Rayneo_DisplaySet3D(RAYNEO_Context ctx);
RAYNEO_API RAYNEO_Result Rayneo_DisplaySet2D(RAYNEO_Context ctx);

// Snapshots of last parsed data
RAYNEO_API RAYNEO_Result Rayneo_GetLastImu(RAYNEO_Context ctx, RAYNEO_ImuSample* out);
RAYNEO_API RAYNEO_Result Rayneo_GetDeviceInfo(RAYNEO_Context ctx, RAYNEO_DeviceInfoMini* out);

#ifdef __cplusplus
} // extern "C"
// Layout sanity check to ensure new decoded fields start after 64 bytes (60 raw + 4 status bytes)
static_assert(offsetof(RAYNEO_DeviceInfoMini, tick) == 64, "RAYNEO_DeviceInfoMini unexpected layout (tick offset must be 64)");
#endif

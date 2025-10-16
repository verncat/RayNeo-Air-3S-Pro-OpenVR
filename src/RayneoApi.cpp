#define RAYNEO_BUILD
#include "rayneo_api.h"

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <string>
#include <condition_variable>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <cstdlib>

#ifdef __APPLE__
#include <IOKit/hid/IOHIDManager.h>
#include <IOKit/hid/IOHIDKeys.h>
#include <CoreFoundation/CoreFoundation.h>
#else
#include <libusb.h>
#endif

struct RayneoContext__
{
    uint16_t vid{0};
    uint16_t pid{0};
    std::atomic<bool> running{false};
    RAYNEO_EventCallback cb{nullptr};
    void *cbUser{nullptr};
    int logLevel{2};
    uint64_t seq{0};
    // Simple last snapshots
    RAYNEO_ImuSample lastImu{};
    RAYNEO_DeviceInfoMini lastInfo{};
    // Minimal event queue for PollEvent (single-producer single-consumer style)
    std::mutex qMtx;
    std::condition_variable qCv;
    std::vector<RAYNEO_Event> queue; // naive FIFO
    size_t maxQueue{128};
    // --- transport (non-Apple libusb) ---
#ifndef __APPLE__
    libusb_context *usbCtx{nullptr};
    libusb_device_handle *handle{nullptr};
    int interfaceNumber{-1};
    uint8_t epIn{0};
    uint8_t epOut{0};
    bool hasInterrupt{false};
    // async interrupt transfer state
    libusb_transfer *inTransfer{nullptr};
    std::atomic<bool> transferActive{false};
    std::atomic<bool> transferDone{false};
    std::atomic<bool> transferResubmit{false};
#else
    IOHIDManagerRef hidManager{nullptr};
    IOHIDDeviceRef hidDevice{nullptr};
    CFRunLoopRef hidRunLoop{nullptr};
    uint8_t *hidInputBuffer{nullptr};
    size_t hidReportSize{64};
    std::mutex hidMtx;
    std::mutex macReadyMtx;
    std::condition_variable macReadyCv;
    bool macReady{false};
    RAYNEO_Result macReadyStatus{RAYNEO_OK};
    std::atomic<bool> macDeviceOnline{false};
#endif
    // worker
    std::thread worker;
    std::atomic<bool> stopWorker{false};
    std::atomic<bool> stopIssued{false};
    // RoundTrip sync
    std::mutex rtMtx;
    std::condition_variable rtCv;
    uint8_t lastFrame[64]{}; // last raw frame
    std::atomic<bool> haveNewFrame{false};
    std::atomic<uint8_t> lastType{0};
};

// Internal protocol message / command identifiers (mirrors RayNeoSimpleClient)
enum : uint8_t
{
    RAYNEO_PROTO_CMD_DEVICE_INFO = 0x00, // Outgoing command to request device info
    RAYNEO_PROTO_ACK_COMMAND = 0xC8,     // Generic command ack / may carry device info
    RAYNEO_PROTO_ACK_IMU_DATA = 0x65,    // IMU sample frame
    RAYNEO_PROTO_ACK_TRACE = 0xCA,       // Trace report
    RAYNEO_PROTO_NOTIFY_SLEEP = 0x41,    // Glasses sleep notify
    RAYNEO_PROTO_NOTIFY_WAKE = 0x42,     // Glasses wake notify
    RAYNEO_PROTO_NOTIFY_FAIL = 0x02
};

static void enqueueEvent(RayneoContext__ *ctx, const RAYNEO_Event &evt);

static void processInboundFrame(RayneoContext__ *ctx, const uint8_t *buf, size_t len)
{
    if (!ctx || !buf || len < 2)
        return;
    if (len < 64 || buf[0] != 0x99)
        return;

    uint8_t type = buf[1];
    ctx->lastType.store(type);
    {
        std::lock_guard<std::mutex> lk(ctx->rtMtx);
        std::memcpy(ctx->lastFrame, buf, 64);
        ctx->haveNewFrame.store(true);
    }
    ctx->rtCv.notify_all();

    auto rdF = [&](int off)
    {
        float f = 0.f;
        if (off + 4 <= static_cast<int>(len))
            std::memcpy(&f, buf + off, 4);
        return f;
    };
    auto rdU32 = [&](int off)
    {
        if (off + 4 > static_cast<int>(len))
            return uint32_t{0};
        return (uint32_t)buf[off] | ((uint32_t)buf[off + 1] << 8) | ((uint32_t)buf[off + 2] << 16) | ((uint32_t)buf[off + 3] << 24);
    };

    if (type == RAYNEO_PROTO_ACK_IMU_DATA)
    {
        RAYNEO_ImuSample sample{};
        sample.valid = 1;
        for (int i = 0; i < 3; i++)
            sample.acc[i] = rdF(4 + i * 4);
        for (int i = 0; i < 3; i++)
            sample.gyroDps[i] = rdF(16 + i * 4);
        sample.temperature = rdF(28);
        sample.magnet[0] = rdF(32);
        sample.magnet[1] = rdF(36);
        sample.tick = rdU32(40);
        sample.psensor = rdF(44);
        sample.lsensor = rdF(48);
        sample.magnet[2] = rdF(52);
        if (len > 56)
            sample.flag = buf[56];
        if (len > 57)
            sample.checksum = buf[57];
        for (int i = 0; i < 3; i++)
            sample.gyroRad[i] = sample.gyroDps[i] * 0.0174532925f;
        ctx->lastImu = sample;
        RAYNEO_Event evt{};
        evt.type = RAYNEO_EVENT_IMU_SAMPLE;
        evt.seq = ++ctx->seq;
        evt.data.imu = sample;
        enqueueEvent(ctx, evt);
    }
    else if (type == RAYNEO_PROTO_ACK_COMMAND)
    {
        RAYNEO_DeviceInfoMini info{};
        info.valid = 1;
        if (len >= 64)
            std::memcpy(info.raw, buf + 4, 60);
        const uint8_t *p = info.raw;
        size_t off = 0;
        auto need = [&](size_t b)
        { return (off + b) <= 60; };
        auto rdU32p = [&]()
        {
            uint32_t v = 0;
            if (need(4))
                v = (uint32_t)p[off] | ((uint32_t)p[off + 1] << 8) | ((uint32_t)p[off + 2] << 16) | ((uint32_t)p[off + 3] << 24);
            off += 4;
            return v;
        };
        auto rdU16p = [&]()
        {
            uint16_t v = 0;
            if (need(2))
                v = (uint16_t)p[off] | ((uint16_t)p[off + 1] << 8);
            off += 2;
            return v;
        };
        auto rdBp = [&]()
        {
            uint8_t v = 0;
            if (need(1))
                v = p[off];
            off += 1;
            return v;
        };
        auto rdFp = [&]()
        {
            float f = 0;
            if (need(4))
                std::memcpy(&f, p + off, 4);
            off += 4;
            return f;
        };
        info.tick = rdU32p();
        info.value = rdBp();
        if (need(12))
        {
            std::memcpy(info.cpuid, p + off, 12);
            off += 12;
        }
        info.board_id = rdBp();
        info.sensor_on = rdBp();
        info.support_fov = rdBp();
        if (need(12))
        {
            std::memcpy(info.date, p + off, 12);
            info.date[12] = '\0';
            off += 12;
        }
        info.year = rdU16p();
        info.month = rdBp();
        info.day = rdBp();
        info.glasses_fps = rdBp();
        info.luminance = rdBp();
        info.volume = rdBp();
        info.side_by_side = rdBp();
        info.psensor_enable = rdBp();
        info.audio_mode = rdBp();
        info.dp_status = rdBp();
        info.status3 = rdBp();
        info.psensor_valid = rdBp();
        info.lsensor_valid = rdBp();
        info.gyro_valid = rdBp();
        info.magnet_valid = rdBp();
        info.reserve1 = rdFp();
        info.reserve2 = rdFp();
        info.max_luminance = rdBp();
        info.max_volume = rdBp();
        info.support_panel_color_adjust = rdBp();
        info.flag = rdBp();
        bool emit = (!ctx->lastInfo.valid) || (ctx->lastInfo.tick != info.tick);
        ctx->lastInfo = info;
        if (emit)
        {
            RAYNEO_Event evt{};
            evt.type = RAYNEO_EVENT_DEVICE_INFO;
            evt.seq = ++ctx->seq;
            evt.data.info = info;
            enqueueEvent(ctx, evt);
        }
    }
    else if (type == RAYNEO_PROTO_NOTIFY_SLEEP || type == RAYNEO_PROTO_NOTIFY_WAKE)
    {
        RAYNEO_Event evt{};
        evt.type = RAYNEO_EVENT_LOG;
        evt.seq = ++ctx->seq;
        evt.data.log.level = 1;
        std::snprintf(evt.data.log.message, sizeof(evt.data.log.message), "%s (type=0x%02X)", (type == RAYNEO_PROTO_NOTIFY_SLEEP) ? "Sleep notify" : "Wakeup notify", type);
        enqueueEvent(ctx, evt);
    }
}

#ifdef __APPLE__
static void macSignalReady(RayneoContext__ *ctx, RAYNEO_Result status)
{
    if (!ctx)
        return;
    {
        std::lock_guard<std::mutex> lk(ctx->macReadyMtx);
        ctx->macReadyStatus = status;
        ctx->macReady = true;
    }
    ctx->macReadyCv.notify_all();
}

static int macGetDeviceInt(IOHIDDeviceRef dev, CFStringRef key, int fallback = 0)
{
    if (!dev)
        return fallback;
    CFTypeRef prop = IOHIDDeviceGetProperty(dev, key);
    if (prop && CFGetTypeID(prop) == CFNumberGetTypeID())
    {
        int value = fallback;
        CFNumberGetValue((CFNumberRef)prop, kCFNumberIntType, &value);
        return value;
    }
    return fallback;
}

static void macDeviceMatchingCallback(void *context, IOReturn result, void *, IOHIDDeviceRef device)
{
    auto *ctx = static_cast<RayneoContext__ *>(context);
    if (!ctx || result != kIOReturnSuccess || !device)
        return;
    int vid = macGetDeviceInt(device, CFSTR(kIOHIDVendorIDKey));
    int pid = macGetDeviceInt(device, CFSTR(kIOHIDProductIDKey));
    if (vid == ctx->vid && pid == ctx->pid)
    {
        std::lock_guard<std::mutex> lk(ctx->hidMtx);
        if (!ctx->hidDevice)
        {
            ctx->hidDevice = device;
            CFRetain(device);
        }
    }
}

static void macDeviceRemovalCallback(void *context, IOReturn result, void *, IOHIDDeviceRef device)
{
    auto *ctx = static_cast<RayneoContext__ *>(context);
    if (!ctx || result != kIOReturnSuccess || !device)
        return;
    bool emitDetach = false;
    {
        std::lock_guard<std::mutex> lk(ctx->hidMtx);
        if (ctx->hidDevice == device && ctx->hidDevice)
        {
            IOHIDDeviceClose(ctx->hidDevice, kIOHIDOptionsTypeNone);
            CFRelease(ctx->hidDevice);
            ctx->hidDevice = nullptr;
            delete[] ctx->hidInputBuffer;
            ctx->hidInputBuffer = nullptr;
            ctx->macDeviceOnline.store(false);
            emitDetach = true;
        }
    }
    if (emitDetach && !ctx->stopWorker.load())
    {
        RAYNEO_Event evt{};
        evt.type = RAYNEO_EVENT_DEVICE_DETACHED;
        evt.seq = ++ctx->seq;
        enqueueEvent(ctx, evt);
    }
}

static void macInputReportCallback(void *context, IOReturn result, void *, IOHIDReportType, uint32_t, uint8_t *report, CFIndex reportLength)
{
    auto *ctx = static_cast<RayneoContext__ *>(context);
    if (!ctx || result != kIOReturnSuccess || !report)
        return;
    processInboundFrame(ctx, report, static_cast<size_t>(reportLength));
}

static bool macEnsureDeviceSelected(RayneoContext__ *ctx)
{
    if (!ctx || !ctx->hidManager)
        return false;

    CFSetRef set = IOHIDManagerCopyDevices(ctx->hidManager);
    if (!set)
        return false;
    CFIndex count = CFSetGetCount(set);
    std::vector<IOHIDDeviceRef> devices(static_cast<size_t>(count));
    CFSetGetValues(set, reinterpret_cast<const void **>(devices.data()));
    for (auto dev : devices)
    {
        int vid = macGetDeviceInt(dev, CFSTR(kIOHIDVendorIDKey));
        int pid = macGetDeviceInt(dev, CFSTR(kIOHIDProductIDKey));
        if (vid == ctx->vid && pid == ctx->pid)
        {
            std::lock_guard<std::mutex> lk(ctx->hidMtx);
            if (!ctx->hidDevice)
            {
                ctx->hidDevice = dev;
                CFRetain(dev);
            }
            CFRelease(set);
            return true;
        }
    }
    CFRelease(set);
    return false;
}

static void macReleaseDevice(RayneoContext__ *ctx)
{
    if (!ctx)
        return;
    std::lock_guard<std::mutex> lk(ctx->hidMtx);
    if (ctx->hidDevice)
    {
        IOHIDDeviceClose(ctx->hidDevice, kIOHIDOptionsTypeNone);
        CFRelease(ctx->hidDevice);
        ctx->hidDevice = nullptr;
    }
    delete[] ctx->hidInputBuffer;
    ctx->hidInputBuffer = nullptr;
    ctx->macDeviceOnline.store(false);
}

static void macCleanupManager(RayneoContext__ *ctx)
{
    if (!ctx)
        return;
    if (ctx->hidManager)
    {
        if (ctx->hidRunLoop)
            IOHIDManagerUnscheduleFromRunLoop(ctx->hidManager, ctx->hidRunLoop, kCFRunLoopDefaultMode);
        IOHIDManagerClose(ctx->hidManager, kIOHIDOptionsTypeNone);
        CFRelease(ctx->hidManager);
        ctx->hidManager = nullptr;
    }
    ctx->hidRunLoop = nullptr;
}

static void macWorkerThread(RayneoContext__ *ctx)
{
    if (!ctx)
        return;
    ctx->hidRunLoop = CFRunLoopGetCurrent();
    ctx->macDeviceOnline.store(false);
    bool signaled = false;
    auto signalOnce = [&](RAYNEO_Result status)
    {
        if (!signaled)
        {
            macSignalReady(ctx, status);
            signaled = true;
        }
    };

    ctx->hidManager = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone);
    if (!ctx->hidManager)
    {
        signalOnce(RAYNEO_ERR_GENERAL);
        return;
    }

    CFMutableDictionaryRef dict = CFDictionaryCreateMutable(kCFAllocatorDefault, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    if (dict)
    {
        CFNumberRef vidNum = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &ctx->vid);
        CFNumberRef pidNum = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &ctx->pid);
        if (vidNum && pidNum)
        {
            CFDictionarySetValue(dict, CFSTR(kIOHIDVendorIDKey), vidNum);
            CFDictionarySetValue(dict, CFSTR(kIOHIDProductIDKey), pidNum);
        }
        if (vidNum)
            CFRelease(vidNum);
        if (pidNum)
            CFRelease(pidNum);
    }
    if (dict)
        IOHIDManagerSetDeviceMatching(ctx->hidManager, dict);
    if (dict)
        CFRelease(dict);

    IOHIDManagerRegisterDeviceMatchingCallback(ctx->hidManager, macDeviceMatchingCallback, ctx);
    IOHIDManagerRegisterDeviceRemovalCallback(ctx->hidManager, macDeviceRemovalCallback, ctx);

    IOReturn rc = IOHIDManagerOpen(ctx->hidManager, kIOHIDOptionsTypeNone);
    if (rc != kIOReturnSuccess)
    {
        signalOnce(RAYNEO_ERR_GENERAL);
        macCleanupManager(ctx);
        return;
    }

    IOHIDManagerScheduleWithRunLoop(ctx->hidManager, ctx->hidRunLoop, kCFRunLoopDefaultMode);

    auto waitForDevice = [&](double seconds, bool allowUnfiltered)
    {
        auto start = std::chrono::steady_clock::now();
        while (!ctx->stopWorker.load() && !ctx->hidDevice && std::chrono::steady_clock::now() - start < std::chrono::duration<double>(seconds))
        {
            CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.05, false);
            if (!ctx->hidDevice)
                macEnsureDeviceSelected(ctx);
        }
        if (!ctx->hidDevice && allowUnfiltered)
        {
            IOHIDManagerSetDeviceMatching(ctx->hidManager, nullptr);
            start = std::chrono::steady_clock::now();
            while (!ctx->stopWorker.load() && !ctx->hidDevice && std::chrono::steady_clock::now() - start < std::chrono::duration<double>(seconds))
            {
                CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.05, false);
                if (!ctx->hidDevice)
                    macEnsureDeviceSelected(ctx);
            }
        }
        return ctx->hidDevice != nullptr;
    };

    if (!waitForDevice(2.0, true))
    {
        signalOnce(RAYNEO_ERR_NO_DEVICE);
        macCleanupManager(ctx);
        return;
    }

    IOReturn openRc = kIOReturnError;
    {
        std::lock_guard<std::mutex> lk(ctx->hidMtx);
        if (ctx->hidDevice)
        {
            openRc = IOHIDDeviceOpen(ctx->hidDevice, kIOHIDOptionsTypeNone);
            if (openRc != kIOReturnSuccess)
                openRc = IOHIDDeviceOpen(ctx->hidDevice, kIOHIDOptionsTypeSeizeDevice);
        }
    }
    if (openRc != kIOReturnSuccess)
    {
        signalOnce(RAYNEO_ERR_IO);
        macReleaseDevice(ctx);
        macCleanupManager(ctx);
        return;
    }

    {
        std::lock_guard<std::mutex> lk(ctx->hidMtx);
        if (ctx->hidDevice)
        {
            CFTypeRef prop = IOHIDDeviceGetProperty(ctx->hidDevice, CFSTR(kIOHIDMaxInputReportSizeKey));
            if (prop && CFGetTypeID(prop) == CFNumberGetTypeID())
            {
                int sz = 0;
                CFNumberGetValue((CFNumberRef)prop, kCFNumberIntType, &sz);
                if (sz > 0)
                    ctx->hidReportSize = static_cast<size_t>(sz);
            }
            delete[] ctx->hidInputBuffer;
            ctx->hidInputBuffer = new uint8_t[ctx->hidReportSize];
            IOHIDDeviceRegisterInputReportCallback(ctx->hidDevice, ctx->hidInputBuffer, ctx->hidReportSize, macInputReportCallback, ctx);
        }
    }

    ctx->macDeviceOnline.store(true);
    signalOnce(RAYNEO_OK);

    while (!ctx->stopWorker.load())
    {
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.05, false);
    }

    macReleaseDevice(ctx);
    macCleanupManager(ctx);
}
#endif

static void enqueueEvent(RayneoContext__ *ctx, const RAYNEO_Event &evt)
{
    if (!ctx)
        return;
    if (ctx->cb)
    {
        ctx->cb(&evt, ctx->cbUser);
        return;
    }
    std::unique_lock<std::mutex> lk(ctx->qMtx);
    if (ctx->queue.size() >= ctx->maxQueue)
    {
        // drop oldest
        ctx->queue.erase(ctx->queue.begin());
    }
    ctx->queue.push_back(evt);
    lk.unlock();
    ctx->qCv.notify_one();
}

const char *Rayneo_ResultToString(RAYNEO_Result r)
{
    switch (r)
    {
    case RAYNEO_OK:
        return "OK";
    case RAYNEO_ERR_GENERAL:
        return "GENERAL";
    case RAYNEO_ERR_NO_DEVICE:
        return "NO_DEVICE";
    case RAYNEO_ERR_TIMEOUT:
        return "TIMEOUT";
    case RAYNEO_ERR_IO:
        return "IO";
    case RAYNEO_ERR_BUSY:
        return "BUSY";
    case RAYNEO_ERR_UNSUPPORTED:
        return "UNSUPPORTED";
    case RAYNEO_ERR_INVALID_ARG:
        return "INVALID_ARG";
    case RAYNEO_ERR_QUEUE_FULL:
        return "QUEUE_FULL";
    default:
        return "UNKNOWN";
    }
}

unsigned int Rayneo_GetApiVersion(void)
{
    return RAYNEO_API_VERSION;
}

RAYNEO_Result Rayneo_Create(RAYNEO_Context *outCtx)
{
    if (!outCtx)
        return RAYNEO_ERR_INVALID_ARG;
    auto *c = new (std::nothrow) RayneoContext__();
    if (!c)
        return RAYNEO_ERR_GENERAL;
    *outCtx = c;
    return RAYNEO_OK;
}

void Rayneo_Destroy(RAYNEO_Context ctx)
{
    if (!ctx)
        return;
    Rayneo_Stop(ctx); // ensure stopped
    delete ctx;
}

RAYNEO_Result Rayneo_SetTargetVidPid(RAYNEO_Context ctx, uint16_t vid, uint16_t pid)
{
    if (!ctx)
        return RAYNEO_ERR_INVALID_ARG;
    if (ctx->running.load())
        return RAYNEO_ERR_BUSY;
    ctx->vid = vid;
    ctx->pid = pid;
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_SetEventCallback(RAYNEO_Context ctx, RAYNEO_EventCallback cb, void *user)
{
    if (!ctx)
        return RAYNEO_ERR_INVALID_ARG;
    if (ctx->running.load())
        return RAYNEO_ERR_BUSY;
    ctx->cb = cb;
    ctx->cbUser = user;
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_SetLogLevel(int level)
{
    (void)level; // global not stored yet
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_Start(RAYNEO_Context ctx, uint32_t /*serviceFlags*/)
{
    if (!ctx)
        return RAYNEO_ERR_INVALID_ARG;
    bool expected = false;
    if (!ctx->running.compare_exchange_strong(expected, true))
        return RAYNEO_ERR_BUSY;
    ctx->stopIssued.store(false);
    ctx->stopWorker.store(false);
        // Initialize transport
#ifndef __APPLE__
    if (libusb_init(&ctx->usbCtx) != 0)
    {
        ctx->running.store(false);
        return RAYNEO_ERR_GENERAL;
    }
    libusb_device **list = nullptr;
    ssize_t n = libusb_get_device_list(ctx->usbCtx, &list);
    libusb_device *target = nullptr;
    for (ssize_t i = 0; i < n; i++)
    {
        libusb_device *dev = list[i];
        libusb_device_descriptor desc{};
        if (libusb_get_device_descriptor(dev, &desc) == 0)
        {
            if (desc.idVendor == ctx->vid && desc.idProduct == ctx->pid)
            {
                target = dev;
                break;
            }
        }
    }
    if (!target)
    {
        libusb_free_device_list(list, 1);
        ctx->running.store(false);
        return RAYNEO_ERR_NO_DEVICE;
    }
    if (libusb_open(target, &ctx->handle) != 0 || !ctx->handle)
    {
        libusb_free_device_list(list, 1);
        ctx->running.store(false);
        return RAYNEO_ERR_IO;
    }
    libusb_free_device_list(list, 1);
    // Find first interface with HID class (0x03) or fallback first
    libusb_config_descriptor *cfg = nullptr; // libusb_get_active_config_descriptor expects non-const**
    if (libusb_get_active_config_descriptor(libusb_get_device(ctx->handle), &cfg) == 0 && cfg)
    {
        for (uint8_t i = 0; i < cfg->bNumInterfaces && ctx->interfaceNumber < 0; i++)
        {
            const libusb_interface &iface = cfg->interface[i];
            for (int a = 0; a < iface.num_altsetting && ctx->interfaceNumber < 0; a++)
            {
                const libusb_interface_descriptor &id = iface.altsetting[a];
                if (id.bInterfaceClass == 0x03 || ctx->interfaceNumber < 0)
                {
                    ctx->interfaceNumber = id.bInterfaceNumber;
                    for (uint8_t e = 0; e < id.bNumEndpoints; e++)
                    {
                        auto &ep = id.endpoint[e];
                        if ((ep.bEndpointAddress & 0x80) && (ep.bmAttributes & 0x3) == 3)
                            ctx->epIn = ep.bEndpointAddress;
                        if (!(ep.bEndpointAddress & 0x80) && (ep.bmAttributes & 0x3) == 3)
                            ctx->epOut = ep.bEndpointAddress;
                    }
                    ctx->hasInterrupt = (ctx->epIn || ctx->epOut);
                }
            }
        }
        libusb_free_config_descriptor(cfg);
    }
    if (ctx->interfaceNumber >= 0)
    {
        libusb_claim_interface(ctx->handle, ctx->interfaceNumber);
    }
#else
    ctx->macReady = false;
    ctx->macReadyStatus = RAYNEO_OK;
    ctx->macDeviceOnline.store(false);
    ctx->worker = std::thread(macWorkerThread, ctx);
    RAYNEO_Result startStatus = RAYNEO_OK;
    {
        std::unique_lock<std::mutex> lk(ctx->macReadyMtx);
        if (!ctx->macReadyCv.wait_for(lk, std::chrono::seconds(5), [&]
                                      { return ctx->macReady; }))
        {
            startStatus = RAYNEO_ERR_TIMEOUT;
        }
        else
        {
            startStatus = ctx->macReadyStatus;
        }
    }
    if (startStatus != RAYNEO_OK)
    {
        ctx->stopWorker.store(true);
        if (ctx->hidRunLoop)
            CFRunLoopWakeUp(ctx->hidRunLoop);
        if (ctx->worker.joinable())
            ctx->worker.join();
        ctx->running.store(false);
        return startStatus;
    }
#endif
    // Emit attached event
    {
        RAYNEO_Event evt{};
        evt.type = RAYNEO_EVENT_DEVICE_ATTACHED;
        evt.seq = ++ctx->seq;
        enqueueEvent(ctx, evt);
    }
    // Start worker thread (only if we have IN endpoint)
#ifndef __APPLE__
    if (ctx->handle && ctx->epIn)
    {
        ctx->stopWorker.store(false);
        // Prepare async transfer
        ctx->inTransfer = libusb_alloc_transfer(0);
        static auto transferCallback = [](libusb_transfer *tr)
        {
            auto *ctx = static_cast<RayneoContext__ *>(tr->user_data);
            if (!ctx)
                return;
            if (ctx->stopWorker.load())
            {
                ctx->transferDone.store(true);
                return;
            }
            if (tr->status == LIBUSB_TRANSFER_COMPLETED && tr->actual_length >= 64)
            {
                const uint8_t *buf = tr->buffer;
                processInboundFrame(ctx, buf, static_cast<size_t>(tr->actual_length));
            }
            ctx->transferDone.store(true);
            if (!ctx->stopWorker.load())
            {
                // resubmit for continuous streaming
                if (ctx->transferResubmit.load())
                {
                    ctx->transferDone.store(false);
                    int r = libusb_submit_transfer(tr);
                    if (r == 0)
                        return; // continue
                }
            }
        };
        std::vector<uint8_t> *bufferHolder = new std::vector<uint8_t>(64); // will leak if not cleaned on exit, manage on stop
        libusb_fill_interrupt_transfer(ctx->inTransfer, ctx->handle, ctx->epIn, bufferHolder->data(), 64, transferCallback, ctx, 0);
        ctx->transferActive.store(true);
        ctx->transferDone.store(false);
        ctx->transferResubmit.store(true);
        libusb_submit_transfer(ctx->inTransfer);
        ctx->worker = std::thread([ctx, bufferHolder]
                                  {
                                      const bool debugFrames = (std::getenv("RAYNEO_DEBUG_FRAMES") != nullptr);
                                      while (!ctx->stopWorker.load())
                                      {
                                          timeval tv{0, 100000}; // 100ms
                                          libusb_handle_events_timeout_completed(ctx->usbCtx, &tv, nullptr);
                                      }
                                      // cancel transfer
                                      ctx->transferResubmit.store(false);
                                      if (ctx->inTransfer && ctx->transferActive.load())
                                      {
                                          libusb_cancel_transfer(ctx->inTransfer);
                                          // wait for callback to mark done
                                          for (int i = 0; i < 50 && !ctx->transferDone.load(); ++i)
                                          {
                                              timeval tv{0, 20000};
                                              libusb_handle_events_timeout_completed(ctx->usbCtx, &tv, nullptr);
                                          }
                                      }
                                      if (debugFrames)
                                          std::fprintf(stderr, "[RayNeoSDK] worker exiting loop (async)\n");
                                      delete bufferHolder; // free buffer
                                  });
    }
#endif
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_Stop(RAYNEO_Context ctx)
{
    if (!ctx)
        return RAYNEO_ERR_INVALID_ARG;
    // Prevent double-stop
    if (ctx->stopIssued.exchange(true))
    {
        return RAYNEO_OK;
    }
    bool was = ctx->running.exchange(false);
    const bool debug = (std::getenv("RAYNEO_DEBUG_SHUTDOWN") != nullptr);
    if (was)
    {
        if (debug)
            std::fprintf(stderr, "[RayNeoSDK] Stop: signaling worker\n");
        ctx->stopWorker.store(true);
#ifdef __APPLE__
        if (ctx->hidRunLoop)
            CFRunLoopWakeUp(ctx->hidRunLoop);
#endif
        if (ctx->worker.joinable())
        {
            if (debug)
                std::fprintf(stderr, "[RayNeoSDK] Stop: joining worker...\n");
            ctx->worker.join();
            if (debug)
                std::fprintf(stderr, "[RayNeoSDK] Stop: worker joined\n");
        }
#ifndef __APPLE__
        if (ctx->handle && ctx->interfaceNumber >= 0)
        {
            if (debug)
                std::fprintf(stderr, "[RayNeoSDK] Stop: releasing interface %d\n", ctx->interfaceNumber);
            libusb_release_interface(ctx->handle, ctx->interfaceNumber);
        }
        if (ctx->handle)
        {
            if (debug)
                std::fprintf(stderr, "[RayNeoSDK] Stop: closing handle\n");
            libusb_close(ctx->handle);
            ctx->handle = nullptr;
        }
        if (ctx->usbCtx)
        {
            if (debug)
                std::fprintf(stderr, "[RayNeoSDK] Stop: exiting libusb ctx\n");
            libusb_exit(ctx->usbCtx);
            ctx->usbCtx = nullptr;
        }
        if (ctx->inTransfer)
        {
            libusb_free_transfer(ctx->inTransfer);
            ctx->inTransfer = nullptr;
        }
#else
        ctx->macReady = false;
    ctx->macReadyStatus = RAYNEO_OK;
        ctx->macDeviceOnline.store(false);
    macReleaseDevice(ctx);
    macCleanupManager(ctx);
#endif
        RAYNEO_Event evt{};
        evt.type = RAYNEO_EVENT_DEVICE_DETACHED;
        evt.seq = ++ctx->seq;
        enqueueEvent(ctx, evt);
        if (debug)
            std::fprintf(stderr, "[RayNeoSDK] Stop: done\n");
    }
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_PollEvent(RAYNEO_Context ctx, RAYNEO_Event *outEvent, uint32_t timeoutMs)
{
    if (!ctx || !outEvent)
        return RAYNEO_ERR_INVALID_ARG;
    std::unique_lock<std::mutex> lk(ctx->qMtx);
    if (timeoutMs == 0)
    {
        if (ctx->queue.empty())
            return RAYNEO_ERR_TIMEOUT;
    }
    else
    {
        ctx->qCv.wait_for(lk, std::chrono::milliseconds(timeoutMs), [&]
                          { return !ctx->queue.empty(); });
        if (ctx->queue.empty())
            return RAYNEO_ERR_TIMEOUT;
    }
    *outEvent = ctx->queue.front();
    ctx->queue.erase(ctx->queue.begin());
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_SendRaw(RAYNEO_Context ctx, const uint8_t frame[64])
{
    if (!ctx || !frame)
        return RAYNEO_ERR_INVALID_ARG;
    if (!ctx->running.load())
        return RAYNEO_ERR_NO_DEVICE;
#ifndef __APPLE__
    if (!ctx->handle)
        return RAYNEO_ERR_NO_DEVICE;
    int transferred = 0;
    int rc = -1;
    if (ctx->epOut)
    {
        rc = libusb_interrupt_transfer(ctx->handle, ctx->epOut, const_cast<uint8_t *>(frame), 64, &transferred, 500);
        if (rc == 0 && transferred == 64)
            return RAYNEO_OK;
    }
    // Fallback control: SET_REPORT Output report ID 0
    uint16_t wValue = (0x02 << 8) | 0x00; // Output report
    rc = libusb_control_transfer(ctx->handle, 0x21, 0x09, wValue, ctx->interfaceNumber >= 0 ? ctx->interfaceNumber : 0,
                                 const_cast<uint8_t *>(frame), 64, 500);
    if (rc == 64)
        return RAYNEO_OK;
    return RAYNEO_ERR_IO;
#else
    if (!ctx->macDeviceOnline.load())
        return RAYNEO_ERR_NO_DEVICE;
    IOHIDDeviceRef dev = nullptr;
    {
        std::lock_guard<std::mutex> lk(ctx->hidMtx);
        if (!ctx->hidDevice)
            return RAYNEO_ERR_NO_DEVICE;
        dev = ctx->hidDevice;
        CFRetain(dev);
    }
    IOReturn rc = IOHIDDeviceSetReport(dev, kIOHIDReportTypeOutput, 0, frame, 64);
    CFRelease(dev);
    return (rc == kIOReturnSuccess) ? RAYNEO_OK : RAYNEO_ERR_IO;
#endif
}

static void buildFrame(uint8_t command, uint8_t value, const void *payload, size_t payloadLen, uint8_t out[64])
{
    std::memset(out, 0, 64);
    out[0] = 0x66;
    out[1] = command;
    out[2] = value;
    if (payload && payloadLen)
    {
        if (payloadLen > 52)
            payloadLen = 52;
        std::memcpy(out + 3, payload, payloadLen);
    }
}

RAYNEO_Result Rayneo_SendCommand(RAYNEO_Context ctx, uint8_t command, uint8_t value, const void *payload, size_t payloadLen)
{
    if (!ctx)
        return RAYNEO_ERR_INVALID_ARG;
    uint8_t frame[64];
    buildFrame(command, value, payload, payloadLen, frame);
    return Rayneo_SendRaw(ctx, frame);
}

RAYNEO_Result Rayneo_RoundTrip(RAYNEO_Context ctx, uint8_t command, uint8_t value, const void *payload, size_t payloadLen, uint32_t timeoutMs, uint8_t outFrame[64])
{
    if (!ctx || !outFrame)
        return RAYNEO_ERR_INVALID_ARG;
    uint8_t frame[64];
    buildFrame(command, value, payload, payloadLen, frame);
    // Reset flag before send
    ctx->haveNewFrame.store(false);
    auto rc = Rayneo_SendRaw(ctx, frame);
    if (rc != RAYNEO_OK)
        return rc;
    std::unique_lock<std::mutex> lk(ctx->rtMtx);
    if (!ctx->rtCv.wait_for(lk, std::chrono::milliseconds(timeoutMs == 0 ? 500 : timeoutMs), [&]
                            { return ctx->haveNewFrame.load(); }))
        return RAYNEO_ERR_TIMEOUT;
    std::memcpy(outFrame, ctx->lastFrame, 64);
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_EnableImu(RAYNEO_Context ctx)
{
    if (!ctx)
        return RAYNEO_ERR_INVALID_ARG;
    // Command 0x01 = IMU ON (по соглашению из клиента)
    return Rayneo_SendCommand(ctx, 0x01, 0x00, nullptr, 0);
}

RAYNEO_Result Rayneo_DisableImu(RAYNEO_Context ctx) { return Rayneo_SendCommand(ctx, 0x02, 0x00, nullptr, 0); }
RAYNEO_Result Rayneo_RequestDeviceInfo(RAYNEO_Context ctx)
{
    if (!ctx)
        return RAYNEO_ERR_INVALID_ARG;
    return Rayneo_SendCommand(ctx, 0x00, 0x00, nullptr, 0);
}
RAYNEO_Result Rayneo_DisplaySet3D(RAYNEO_Context /*ctx*/) { return RAYNEO_OK; }
RAYNEO_Result Rayneo_DisplaySet2D(RAYNEO_Context /*ctx*/) { return RAYNEO_OK; }

RAYNEO_Result Rayneo_GetLastImu(RAYNEO_Context ctx, RAYNEO_ImuSample *out)
{
    if (!ctx || !out)
        return RAYNEO_ERR_INVALID_ARG;
    *out = ctx->lastImu;
    return out->valid ? RAYNEO_OK : RAYNEO_ERR_TIMEOUT;
}
RAYNEO_Result Rayneo_GetDeviceInfo(RAYNEO_Context ctx, RAYNEO_DeviceInfoMini *out)
{
    if (!ctx || !out)
        return RAYNEO_ERR_INVALID_ARG;
    *out = ctx->lastInfo;
    return out->valid ? RAYNEO_OK : RAYNEO_ERR_TIMEOUT;
}

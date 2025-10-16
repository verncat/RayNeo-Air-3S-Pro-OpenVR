// Orientation demo using SDL2 + OpenGL (immediate mode) to visualize RayNeo IMU orientation.

#include <cstdio>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <array>
#include <vector>
#include <algorithm>
#include <thread>
#include <string>
#include "rayneo_api.h"

#include <SDL.h>
#include <SDL_opengl.h>
#include <SDL_ttf.h>
#ifdef _WIN32
#include <windows.h>
#endif // _WIN32

struct Orientation
{
    // Simple fused orientation from integrating gyro only (drift!)
    // We'll keep Euler angles in radians.
    float yaw{0}, pitch{0}, roll{0};
};

struct RenderToggles
{
    bool showAxes = true;
    bool showModel = true;
    bool showText = true;
    bool showMagGraph = true;
    bool showAccGraph = true;
};

struct PositionState
{
    float vel[3]{0, 0, 0};
    float pos[3]{0, 0, 0};
    bool enable = true;
};
struct FusionState
{
    bool useFusion = true;
    bool calibrated = false;
    float gyroBias[3]{0, 0, 0};
    int calibSamples = 0;
    bool gyroOnly = false; // ignore magnet correction entirely
};
struct AccelCalState
{
    bool calibrating = false;
    bool calibrated = false;
    int samples = 0;
    float minVal[3]{1e9f, 1e9f, 1e9f};
    float maxVal[3]{-1e9f, -1e9f, -1e9f};
    float offset[3]{0, 0, 0}; // expected to be gravity vector components when stationary
    float scale[3]{1, 1, 1};  // optional axis normalization
};
struct MagnetFusionState
{
    bool enable = true;       // apply yaw correction
    bool calibrated = false;  // offsets captured
    int calibSamples = 0;     // samples accumulated for calibration
    float offset[3]{0, 0, 0}; // hard-iron offset (rough)
    float headingDeg = 0.0f;  // latest tilt-compensated heading
    bool magnetOnly = false;  // orientation derived solely from magnetometer (yaw only)
    // Extended calibration
    bool calibrating = false; // currently gathering min/max
    float minVal[3]{1e9f, 1e9f, 1e9f};
    float maxVal[3]{-1e9f, -1e9f, -1e9f};
    float scale[3]{1, 1, 1}; // scale factors to normalize ellipse (simple axis scaling)
};

// --- Bitmap font fallback (8x8) -------------------------------------------------
static unsigned char font8x8_basic[96][8] = {{0}};
static bool fontInit = false;
static void initFont()
{
    if (fontInit)
        return;
    fontInit = true;
    auto setChar = [](char c, std::initializer_list<uint8_t> rows)
    { int idx=c-32; int r=0; for(auto v:rows){ font8x8_basic[idx][r++]=v; if(r>=8) break; } };
    setChar('0', {0x3C, 0x66, 0x6E, 0x76, 0x66, 0x66, 0x3C, 0x00});
    setChar('1', {0x18, 0x38, 0x18, 0x18, 0x18, 0x1C, 0x7E, 0x00});
    setChar('2', {0x3C, 0x66, 0x06, 0x1C, 0x30, 0x66, 0x7E, 0x00});
    setChar('3', {0x3C, 0x66, 0x06, 0x1C, 0x06, 0x66, 0x3C, 0x00});
    setChar('4', {0x0C, 0x1C, 0x3C, 0x6C, 0x7E, 0x0C, 0x0C, 0x00});
    setChar('5', {0x7E, 0x60, 0x7C, 0x06, 0x06, 0x66, 0x3C, 0x00});
    setChar('6', {0x1C, 0x30, 0x60, 0x7C, 0x66, 0x66, 0x3C, 0x00});
    setChar('7', {0x7E, 0x66, 0x0C, 0x18, 0x18, 0x18, 0x18, 0x00});
    setChar('8', {0x3C, 0x66, 0x66, 0x3C, 0x66, 0x66, 0x3C, 0x00});
    setChar('9', {0x3C, 0x66, 0x66, 0x3E, 0x06, 0x0C, 0x38, 0x00});
    setChar(' ', {0x00});
    setChar('.', {0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00});
    setChar('-', {0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00});
    setChar(':', {0x00, 0x18, 0x18, 0x00, 0x00, 0x18, 0x18, 0x00});
    setChar('Y', {0x66, 0x66, 0x3C, 0x18, 0x18, 0x18, 0x3C, 0x00});
    setChar('P', {0x7C, 0x66, 0x66, 0x7C, 0x60, 0x60, 0x60, 0x00});
    setChar('R', {0x7C, 0x66, 0x66, 0x7C, 0x6C, 0x66, 0x66, 0x00});
    setChar('D', {0x78, 0x6C, 0x66, 0x66, 0x66, 0x6C, 0x78, 0x00});
    setChar('E', {0x7E, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x7E, 0x00});
    setChar('G', {0x3C, 0x66, 0x60, 0x6E, 0x66, 0x66, 0x3C, 0x00});
    setChar('(', {0x0E, 0x18, 0x30, 0x30, 0x30, 0x18, 0x0E, 0x00});
    setChar(')', {0x70, 0x18, 0x0C, 0x0C, 0x0C, 0x18, 0x70, 0x00});
    setChar('=', {0x00, 0x7E, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00});
}
static void drawChar(float x, float y, char c, float scale)
{
    if (c < 32 || c > 127)
        return;
    int idx = c - 32;
    float s = scale;
    glBegin(GL_QUADS);
    for (int row = 0; row < 8; ++row)
    {
        uint8_t bits = font8x8_basic[idx][row];
        for (int col = 0; col < 8; ++col)
        {
            if (bits & (1 << (7 - col)))
            {
                float x0 = x + col * s;
                float y0 = y + row * s;
                float x1 = x0 + s;
                float y1 = y0 + s;
                glVertex2f(x0, y0);
                glVertex2f(x1, y0);
                glVertex2f(x1, y1);
                glVertex2f(x0, y1);
            }
        }
    }
    glEnd();
}
static void drawString(float x, float y, const char *txt, float scale = 2.0f)
{
    for (const char *p = txt; *p; ++p)
    {
        drawChar(x, y, *p, scale);
        x += 8 * scale;
    }
}

static void drawGlassesModel()
{
    glColor3f(0.9f, 0.9f, 0.9f);
    glBegin(GL_LINE_LOOP); // left lens
    glVertex3f(-0.8f, 0.3f, 0.0f);
    glVertex3f(-0.2f, 0.3f, 0.0f);
    glVertex3f(-0.2f, -0.3f, 0.0f);
    glVertex3f(-0.8f, -0.3f, 0.0f);
    glEnd();
    glBegin(GL_LINE_LOOP); // right lens
    glVertex3f(0.2f, 0.3f, 0.0f);
    glVertex3f(0.8f, 0.3f, 0.0f);
    glVertex3f(0.8f, -0.3f, 0.0f);
    glVertex3f(0.2f, -0.3f, 0.0f);
    glEnd();
    glBegin(GL_LINES); // bridge
    glVertex3f(-0.2f, 0.05f, 0.0f);
    glVertex3f(0.2f, 0.05f, 0.0f);
    glVertex3f(-0.2f, -0.05f, 0.0f);
    glVertex3f(0.2f, -0.05f, 0.0f);
    glEnd();
    glBegin(GL_LINES); // temples
    glVertex3f(-0.8f, 0.2f, 0.0f);
    glVertex3f(-1.1f, 0.25f, -0.6f);
    glVertex3f(0.8f, 0.2f, 0.0f);
    glVertex3f(1.1f, 0.25f, -0.6f);
    glEnd();
}

static void applyOrientation(const Orientation &o)
{
    glRotatef(o.yaw * 57.2958f, 0, 0, 1);
    glRotatef(o.pitch * 57.2958f, 0, 1, 0);
    glRotatef(o.roll * 57.2958f, 1, 0, 0);
}

static void fuseImu(Orientation &o, FusionState &f, const RAYNEO_ImuSample &s, float dt)
{
    if (!s.valid)
        return;
    float gx = (s.gyroDps[0] - (f.calibrated ? f.gyroBias[0] : 0.f)) * 0.0174532925f;
    float gy = (s.gyroDps[1] - (f.calibrated ? f.gyroBias[1] : 0.f)) * 0.0174532925f;
    float gz = (s.gyroDps[2] - (f.calibrated ? f.gyroBias[2] : 0.f)) * 0.0174532925f;
    o.yaw += gz * dt;
    o.pitch += gy * dt;
    o.roll += gx * dt;
}

static void drawAxes(const Orientation &o)
{
    float cy = cosf(o.yaw), sy = sinf(o.yaw);
    float cp = cosf(o.pitch), sp = sinf(o.pitch);
    float cr = cosf(o.roll), sr = sinf(o.roll);
    auto rotate = [&](float x, float y, float z)
    {
        float x1=x; float y1= y*cr - z*sr; float z1= y*sr + z*cr;
        float x2= x1*cp + z1*sp; float y2=y1; float z2= -x1*sp + z1*cp;
        float x3= x2*cy - y2*sy; float y3= x2*sy + y2*cy; float z3=z2;
        return std::array<float,3>{x3,y3,z3}; };
    glBegin(GL_LINES);
    auto X = rotate(1, 0, 0);
    glColor3f(1, 0, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(X[0], X[1], X[2]);
    auto Y = rotate(0, 1, 0);
    glColor3f(0, 1, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(Y[0], Y[1], Y[2]);
    auto Z = rotate(0, 0, 1);
    glColor3f(0, 0, 1);
    glVertex3f(0, 0, 0);
    glVertex3f(Z[0], Z[1], Z[2]);
    glEnd();
}

int main()
{
    printf("RayNeo orientation demo (axes)\n");
    RAYNEO_Context ctx{};
    if (Rayneo_Create(&ctx) != RAYNEO_OK)
    {
        printf("Create failed\n");
        return 1;
    }
    Rayneo_SetTargetVidPid(ctx, 0x1BBB, 0xAF50);
    auto startRc = Rayneo_Start(ctx, 0);
    if (startRc != RAYNEO_OK)
    {
        printf("Start failed: %d (%s). Possible reasons: device not connected, wrong VID/PID, libusb DLL missing, driver conflict.\n",
               startRc, Rayneo_ResultToString((RAYNEO_Result)startRc));
        printf("Expected VID:PID = %04X:%04X\n", 0x1BBB, 0xAF50);
        printf("Check that libusb-1.0.dll is present next to the EXE.\n");
#ifndef __APPLE__
        // // Attempt to enumerate available USB devices for diagnostics
        // extern "C" {
        //     struct libusb_context;
        //     struct libusb_device;
        //     struct libusb_device_descriptor { uint8_t  bLength; uint8_t  bDescriptorType; uint16_t bcdUSB; uint8_t  bDeviceClass; uint8_t  bDeviceSubClass; uint8_t  bDeviceProtocol; uint8_t  bMaxPacketSize0; uint16_t idVendor; uint16_t idProduct; uint16_t bcdDevice; uint8_t  iManufacturer; uint8_t  iProduct; uint8_t  iSerialNumber; uint8_t  bNumConfigurations; };
        //     int libusb_init(libusb_context**);
        //     ssize_t libusb_get_device_list(libusb_context*, libusb_device***);
        //     int libusb_get_device_descriptor(libusb_device*, libusb_device_descriptor*);
        //     void libusb_free_device_list(libusb_device**, int);
        //     void libusb_exit(libusb_context*);
        // }
        // libusb_context* tmpCtx=nullptr;
        // if (libusb_init(&tmpCtx)==0) {
        //     libusb_device** list=nullptr; ssize_t n=libusb_get_device_list(tmpCtx,&list);
        //     printf("USB devices detected (%zd):\n", n);
        //     for (ssize_t i=0;i<n;i++) {
        //         libusb_device_descriptor d{}; if (libusb_get_device_descriptor(list[i], &d)==0) {
        //             printf("  %04X:%04X class=0x%02X\n", d.idVendor, d.idProduct, d.bDeviceClass);
        //         }
        //     }
        //     libusb_free_device_list(list,1);
        //     libusb_exit(tmpCtx);
        // } else {
        //     printf("Failed to init libusb for enumeration.\n");
        // }
#endif
        Rayneo_Destroy(ctx);
        std::this_thread::sleep_for(std::chrono::seconds(5));
        return 1;
    }
    Rayneo_EnableImu(ctx);

    Orientation orient{};
    FusionState fusion{};
    MagnetFusionState magFusion{};
    PositionState motion{};
    AccelCalState accCal{};
    auto lastTime = std::chrono::steady_clock::now();

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0)
    {
        printf("SDL init failed: %s\n", SDL_GetError());
        Rayneo_Destroy(ctx);
        return 1;
    }
    if (TTF_Init() != 0)
    {
        printf("SDL_ttf init failed: %s\n", TTF_GetError());
    }
    // Request core profile? Keep compatibility for immediate mode.
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_Window *window = SDL_CreateWindow("RayNeo Axes", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 800, 600, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (!window)
    {
        printf("Window creation failed: %s\n", SDL_GetError());
        SDL_Quit();
        Rayneo_Destroy(ctx);
        return 1;
    }
    SDL_GLContext glctx = SDL_GL_CreateContext(window);
    if (!glctx)
    {
        printf("GL context failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        Rayneo_Destroy(ctx);
        return 1;
    }
    SDL_GL_SetSwapInterval(1); // vsync
    glEnable(GL_DEPTH_TEST);
    auto reshape = [&](int w, int h)
    { if(h==0) h=1; glViewport(0,0,w,h); glMatrixMode(GL_PROJECTION); glLoadIdentity(); float aspect=float(w)/float(h); glFrustum(-aspect, aspect, -1,1, 1.0,10.0); glMatrixMode(GL_MODELVIEW); };
    reshape(800, 600);
    // Try load a TTF font; fallback to bitmap font if not available.
    initFont();
    TTF_Font *font = nullptr;
    const char *fontCandidates[] = {
        "COMIC.TTF",
    };
    for (auto f : fontCandidates)
    {
        font = TTF_OpenFont(f, 18);
        if (font)
            break;
    }
    if (!font)
    {
        printf("Could not load TTF font, falling back to bitmap font overlay.\n");
    }
    else
    {
        printf("Loaded TTF font for SDL text rendering.\n");
    }
    RenderToggles toggles;
    double lastTitleUpdate = 0.0;
    bool running = true;
    while (running)
    {
        // Poll SDL events
        SDL_Event sev;
        while (SDL_PollEvent(&sev))
        {
            if (sev.type == SDL_QUIT)
                running = false;
            else if (sev.type == SDL_WINDOWEVENT && sev.window.event == SDL_WINDOWEVENT_SIZE_CHANGED)
            {
                reshape(sev.window.data1, sev.window.data2);
            }
            else if (sev.type == SDL_KEYDOWN)
            {
                switch (sev.key.keysym.sym)
                {
                case SDLK_ESCAPE:
                    running = false;
                    break;
                case SDLK_a:
                    toggles.showAxes = !toggles.showAxes;
                    break;
                case SDLK_m:
                    toggles.showModel = !toggles.showModel;
                    break;
                case SDLK_t:
                    toggles.showText = !toggles.showText;
                    break;
                case SDLK_g:
                    toggles.showMagGraph = !toggles.showMagGraph;
                    break;
                case SDLK_h:
                    magFusion.magnetOnly = !magFusion.magnetOnly;
                    break; // toggle magnet-only orientation
                case SDLK_j:
                    fusion.gyroOnly = !fusion.gyroOnly;
                    if (fusion.gyroOnly)
                    {
                        magFusion.magnetOnly = false;
                        magFusion.enable = false;
                    }
                    break; // gyro-only orientation
                case SDLK_x:
                    toggles.showAccGraph = !toggles.showAccGraph;
                    break; // toggle accelerometer graph
                case SDLK_k:
                    if (!accCal.calibrating)
                    {
                        accCal.calibrating = true;
                        accCal.calibrated = false;
                        accCal.samples = 0;
                        for (int i = 0; i < 3; i++)
                        {
                            accCal.minVal[i] = 1e9f;
                            accCal.maxVal[i] = -1e9f;
                        }
                        printf("Accel calibration started: keep device stationary (or do gentle rotations) then press K again.\n");
                    }
                    else
                    {
                        accCal.calibrating = false; // finish
                        for (int i = 0; i < 3; i++)
                        {
                            accCal.offset[i] = (accCal.maxVal[i] + accCal.minVal[i]) * 0.5f;
                        }
                        float range[3];
                        for (int i = 0; i < 3; i++)
                        {
                            range[i] = accCal.maxVal[i] - accCal.minVal[i];
                            accCal.scale[i] = (range[i] > 1e-6f) ? (2.0f / range[i]) : 1.0f;
                        }
                        accCal.calibrated = true;
                        printf("Accel calibration finished. offset=(%.3f %.3f %.3f) scale=(%.3f %.3f %.3f)\n", accCal.offset[0], accCal.offset[1], accCal.offset[2], accCal.scale[0], accCal.scale[1], accCal.scale[2]);
                    }
                    break;
                case SDLK_c:
                    if (!magFusion.calibrating)
                    {
                        // Start calibration: reset ranges
                        magFusion.calibrating = true;
                        magFusion.calibSamples = 0;
                        magFusion.calibrated = false;
                        for (int i = 0; i < 3; i++)
                        {
                            magFusion.minVal[i] = 1e9f;
                            magFusion.maxVal[i] = -1e9f;
                        }
                        printf("Mag calibration started: rotate device in all orientations until ranges stabilize, then press C again to finish.\n");
                    }
                    else
                    {
                        // Finish calibration
                        magFusion.calibrating = false;
                        // Compute offsets and scales
                        for (int i = 0; i < 3; i++)
                        {
                            magFusion.offset[i] = (magFusion.maxVal[i] + magFusion.minVal[i]) * 0.5f;
                        }
                        // average range for normalization
                        float range[3];
                        for (int i = 0; i < 3; i++)
                            range[i] = magFusion.maxVal[i] - magFusion.minVal[i];
                        float avgRange = (range[0] + range[1] + range[2]) / 3.0f;
                        for (int i = 0; i < 3; i++)
                        {
                            magFusion.scale[i] = (range[i] > 1e-6f) ? (avgRange / range[i]) : 1.0f;
                        }
                        magFusion.calibrated = true;
                        printf("Mag calibration finished. Offsets=(%.3f %.3f %.3f) Scales=(%.3f %.3f %.3f)\n",
                               magFusion.offset[0], magFusion.offset[1], magFusion.offset[2], magFusion.scale[0], magFusion.scale[1], magFusion.scale[2]);
                    }
                    break;
                case SDLK_r:
                    orient = Orientation{};
                    fusion.calibrated = false;
                    fusion.calibSamples = 0;
                    fusion.gyroBias[0] = fusion.gyroBias[1] = fusion.gyroBias[2] = 0;
                    break;
                case SDLK_f:
                    motion.enable = !motion.enable;
                    break; // toggle position integration
                case SDLK_y:
                    magFusion.enable = !magFusion.enable;
                    break; // toggle magnet yaw correction
                case SDLK_p:
                    motion.pos[0] = motion.pos[1] = motion.pos[2] = 0;
                    motion.vel[0] = motion.vel[1] = motion.vel[2] = 0;
                    break; // zero position
                default:
                    break;
                }
            }
        }
        // Poll IMU events (non-blocking)
        RAYNEO_Event evt{};
        static std::vector<std::array<float, 3>> magHistory;
        const size_t MAX_HISTORY = 800; // about window width
        static std::vector<std::array<float, 3>> accHistory;
        while (Rayneo_PollEvent(ctx, &evt, 0) == RAYNEO_OK)
        {
            if (evt.type == RAYNEO_EVENT_IMU_SAMPLE)
            {
                auto now = std::chrono::steady_clock::now();
                float dt = std::chrono::duration<float>(now - lastTime).count();
                lastTime = now;
                if (!fusion.calibrated)
                {
                    fusion.gyroBias[0] = (fusion.gyroBias[0] * fusion.calibSamples + evt.data.imu.gyroDps[0]) / (fusion.calibSamples + 1);
                    fusion.gyroBias[1] = (fusion.gyroBias[1] * fusion.calibSamples + evt.data.imu.gyroDps[1]) / (fusion.calibSamples + 1);
                    fusion.gyroBias[2] = (fusion.gyroBias[2] * fusion.calibSamples + evt.data.imu.gyroDps[2]) / (fusion.calibSamples + 1);
                    fusion.calibSamples++;
                    if (fusion.calibSamples > 300)
                        fusion.calibrated = true;
                }
                if (!magFusion.magnetOnly)
                {
                    fuseImu(orient, fusion, evt.data.imu, dt);
                }
                // Magnetometer yaw correction (tilt-compensated)
                if (evt.data.imu.valid)
                {
                    const float mxRaw = evt.data.imu.magnet[0];
                    const float myRaw = evt.data.imu.magnet[1];
                    const float mzRaw = evt.data.imu.magnet[2];
                    if (magFusion.calibrating)
                    {
                        // Update min/max each axis
                        magFusion.minVal[0] = std::min(magFusion.minVal[0], mxRaw);
                        magFusion.maxVal[0] = std::max(magFusion.maxVal[0], mxRaw);
                        magFusion.minVal[1] = std::min(magFusion.minVal[1], myRaw);
                        magFusion.maxVal[1] = std::max(magFusion.maxVal[1], myRaw);
                        magFusion.minVal[2] = std::min(magFusion.minVal[2], mzRaw);
                        magFusion.maxVal[2] = std::max(magFusion.maxVal[2], mzRaw);
                        magFusion.calibSamples++;
                    }
                    // Calibrate offsets during initial stationary period
                    float mx = mxRaw - magFusion.offset[0];
                    float my = myRaw - magFusion.offset[1];
                    float mz = mzRaw - magFusion.offset[2];
                    mx *= magFusion.scale[0];
                    my *= magFusion.scale[1];
                    mz *= magFusion.scale[2];
                    // Tilt compensation using current pitch/roll
                    float cr = cosf(orient.roll);
                    float sr = sinf(orient.roll);
                    float cp = cosf(orient.pitch);
                    float sp = sinf(orient.pitch);
                    // Rotate magnetic vector from body to level frame
                    float mxh = mx * cp + mz * sp;                     // pitch compensation
                    float myh = mx * sr * sp + my * cr - mz * sr * cp; // combined roll+pitch
                    float heading = atan2f(-myh, mxh);                 // right-handed; invert Y for conventional heading
                    // Normalize to [0,360)
                    if (heading < 0)
                        heading += 2.0f * (float)M_PI;
                    magFusion.headingDeg = heading * 57.2957795f;
                    if (fusion.gyroOnly)
                    {
                        // In gyro-only mode we do nothing here (yaw from gyro integration only)
                    }
                    else if (magFusion.magnetOnly)
                    {
                        // Directly set yaw from magnetic heading; zero other axes
                        orient.yaw = heading;
                        orient.pitch = 0;
                        orient.roll = 0;
                    }
                    else if (magFusion.enable && magFusion.calibrated)
                    {
                        // Blend gyro yaw towards magnetic heading (convert to radians)
                        float yawMag = heading;
                        // unwrap difference to (-pi,pi)
                        float diff = yawMag - orient.yaw;
                        while (diff > (float)M_PI)
                            diff -= 2.0f * (float)M_PI;
                        while (diff < -(float)M_PI)
                            diff += 2.0f * (float)M_PI;
                        const float beta = 0.005f; // small correction gain
                        orient.yaw += diff * beta;
                    }
                }
                if (evt.data.imu.valid && accCal.calibrating)
                {
                    for (int i = 0; i < 3; i++)
                    {
                        float v = evt.data.imu.acc[i];
                        accCal.minVal[i] = std::min(accCal.minVal[i], v);
                        accCal.maxVal[i] = std::max(accCal.maxVal[i], v);
                    }
                    accCal.samples++;
                }
                if (motion.enable && evt.data.imu.valid)
                {
                    // Apply calibration (offset removal) then subtract gravity along device's reference axis.
                    float axRaw = evt.data.imu.acc[0];
                    float ayRaw = evt.data.imu.acc[1];
                    float azRaw = evt.data.imu.acc[2];
                    float ax = axRaw - (accCal.calibrated ? accCal.offset[0] : 0.0f);
                    float ay = ayRaw - (accCal.calibrated ? accCal.offset[1] : 0.0f);
                    float az = azRaw - (accCal.calibrated ? accCal.offset[2] : 1.0f); // assume gravity mostly on Z until better orientation-based removal
                    motion.vel[0] += ax * 9.80665f * dt;
                    motion.vel[1] += ay * 9.80665f * dt;
                    motion.vel[2] += az * 9.80665f * dt;
                    motion.pos[0] += motion.vel[0] * dt;
                    motion.pos[1] += motion.vel[1] * dt;
                    motion.pos[2] += motion.vel[2] * dt;
                }
                // Store magnet sample
                if (evt.data.imu.valid)
                {
                    magHistory.push_back({evt.data.imu.magnet[0], evt.data.imu.magnet[1], evt.data.imu.magnet[2]});
                    if (magHistory.size() > MAX_HISTORY)
                        magHistory.erase(magHistory.begin(), magHistory.begin() + (magHistory.size() - MAX_HISTORY));
                    accHistory.push_back({evt.data.imu.acc[0], evt.data.imu.acc[1], evt.data.imu.acc[2]});
                    if (accHistory.size() > MAX_HISTORY)
                        accHistory.erase(accHistory.begin(), accHistory.begin() + (accHistory.size() - MAX_HISTORY));
                }
            }
            else if (evt.type == RAYNEO_EVENT_DEVICE_DETACHED)
            {
                running = false;
                break;
            }
        }
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();
        glTranslatef(0, 0, -3.0f);
        glPushMatrix();
        applyOrientation(orient);
        if (toggles.showModel)
            drawGlassesModel();
        if (toggles.showAxes)
            drawAxes(orient);
        glPopMatrix();
        if (toggles.showText)
        {
            int w, h;
            SDL_GetWindowSize(window, &w, &h);
            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            glLoadIdentity();
            glOrtho(0, w, h, 0, -1, 1);
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();
            glDisable(GL_DEPTH_TEST);
            char line1[128];
            std::snprintf(line1, sizeof(line1), "Yaw=%.1f  Pitch=%.1f  Roll=%.1f deg", orient.yaw * 57.2958f, orient.pitch * 57.2958f, orient.roll * 57.2958f);
            char lineBias[128];
            std::snprintf(lineBias, sizeof(lineBias), "GyroBias[%.2f %.2f %.2f]%s", fusion.gyroBias[0], fusion.gyroBias[1], fusion.gyroBias[2], fusion.calibrated ? "" : " (calib)");
            char lineHeading[240];
            std::snprintf(lineHeading, sizeof(lineHeading), "MagHeading: %.1f deg %s%s%s%s%s", magFusion.headingDeg, magFusion.enable ? "ON" : "OFF", magFusion.calibrated ? "" : " (legacy calib)", magFusion.magnetOnly ? " [MAG ONLY]" : "", fusion.gyroOnly ? " [GYRO ONLY]" : "", magFusion.calibrating ? " [CALIBRATING]" : "");
            char lineMagCal[240];
            std::snprintf(lineMagCal, sizeof(lineMagCal), "MagCal: min(%.1f %.1f %.1f) max(%.1f %.1f %.1f) scale(%.2f %.2f %.2f) samples:%d", magFusion.minVal[0], magFusion.minVal[1], magFusion.minVal[2], magFusion.maxVal[0], magFusion.maxVal[1], magFusion.maxVal[2], magFusion.scale[0], magFusion.scale[1], magFusion.scale[2], magFusion.calibSamples);
            char linePos[160];
            std::snprintf(linePos, sizeof(linePos), "Pos(m): %.3f %.3f %.3f", motion.pos[0], motion.pos[1], motion.pos[2]);
            char lineVel[160];
            std::snprintf(lineVel, sizeof(lineVel), "Vel(m/s): %.3f %.3f %.3f %s", motion.vel[0], motion.vel[1], motion.vel[2], motion.enable ? "" : "(paused)");
            // Pull latest snapshot for accel/magnet (non-blocking convenience)
            RAYNEO_ImuSample last{};
            if (Rayneo_GetLastImu(ctx, &last) != RAYNEO_OK)
            {
                last.valid = 0;
            }
            char lineAcc[160];
            if (last.valid)
            {
                // Show raw minus offset if calibrated
                float axd = last.acc[0] - (accCal.calibrated ? accCal.offset[0] : 0.0f);
                float ayd = last.acc[1] - (accCal.calibrated ? accCal.offset[1] : 0.0f);
                float azd = last.acc[2] - (accCal.calibrated ? accCal.offset[2] : 1.0f);
                std::snprintf(lineAcc, sizeof(lineAcc), "Acc(adjusted): %.3f %.3f %.3f g", axd, ayd, azd);
            }
            else
            {
                std::snprintf(lineAcc, sizeof(lineAcc), "Acc: (n/a)");
            }
            char lineAccCal[256];
            std::snprintf(lineAccCal, sizeof(lineAccCal), "AccCal: min(%.2f %.2f %.2f) max(%.2f %.2f %.2f) off(%.2f %.2f %.2f) samples:%d%s", accCal.minVal[0], accCal.minVal[1], accCal.minVal[2], accCal.maxVal[0], accCal.maxVal[1], accCal.maxVal[2], accCal.offset[0], accCal.offset[1], accCal.offset[2], accCal.samples, accCal.calibrating ? " [ACC CALIBRATING]" : "");
            char lineMag[160];
            if (last.valid)
            {
                std::snprintf(lineMag, sizeof(lineMag), "Mag: %.3f %.3f %.3f", last.magnet[0], last.magnet[1], last.magnet[2]);
            }
            else
            {
                std::snprintf(lineMag, sizeof(lineMag), "Mag: (n/a)");
            }
            char lineGyro[160];
            if (last.valid)
            {
                std::snprintf(lineGyro, sizeof(lineGyro), "Gyro(dps): %.2f %.2f %.2f", last.gyroDps[0], last.gyroDps[1], last.gyroDps[2]);
            }
            else
            {
                std::snprintf(lineGyro, sizeof(lineGyro), "Gyro: (n/a)");
            }
            char lineTemp[128];
            if (last.valid)
            {
                std::snprintf(lineTemp, sizeof(lineTemp), "Temp: %.2f C", last.temperature);
            }
            else
            {
                std::snprintf(lineTemp, sizeof(lineTemp), "Temp: (n/a)");
            }
            char linePSensor[128];
            if (last.valid)
            {
                std::snprintf(linePSensor, sizeof(linePSensor), "Prox: %.2f", last.psensor);
            }
            else
            {
                std::snprintf(linePSensor, sizeof(linePSensor), "Prox: (n/a)");
            }
            char lineLSensor[128];
            if (last.valid)
            {
                std::snprintf(lineLSensor, sizeof(lineLSensor), "Light: %.2f", last.lsensor);
            }
            else
            {
                std::snprintf(lineLSensor, sizeof(lineLSensor), "Light: (n/a)");
            }
            char lineTick[160];
            if (last.valid)
            {
                std::snprintf(lineTick, sizeof(lineTick), "Tick:%u Cnt:%u Flag:0x%02X", last.tick, last.count, last.flag);
            }
            else
            {
                std::snprintf(lineTick, sizeof(lineTick), "Tick/Cnt: (n/a)");
            }
            auto renderLine = [&](const char *text, int x, int y, SDL_Color col)
            {
                if (!font)
                { // bitmap fallback path
                    glColor3f(col.r / 255.0f, col.g / 255.0f, col.b / 255.0f);
                    drawString((float)x, (float)y, text, (text == line1) ? 2.0f : 1.5f);
                    return;
                }
                SDL_Surface *surf = TTF_RenderUTF8_Blended(font, text, col);
                if (!surf)
                {
                    glColor3f(col.r / 255.0f, col.g / 255.0f, col.b / 255.0f);
                    drawString((float)x, (float)y, text, (text == line1) ? 2.0f : 1.5f);
                    return;
                }
                // Convert to known RGBA32 format so channel order matches GL_RGBA for little-endian.
                SDL_Surface *conv = SDL_ConvertSurfaceFormat(surf, SDL_PIXELFORMAT_RGBA32, 0);
                SDL_FreeSurface(surf);
                if (!conv)
                {
                    glColor3f(col.r / 255.0f, col.g / 255.0f, col.b / 255.0f);
                    drawString((float)x, (float)y, text, (text == line1) ? 2.0f : 1.5f);
                    return;
                }
                GLuint tex;
                glGenTextures(1, &tex);
                glBindTexture(GL_TEXTURE_2D, tex);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, conv->w, conv->h, 0, GL_RGBA, GL_UNSIGNED_BYTE, conv->pixels);
                glEnable(GL_TEXTURE_2D);
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                glColor3f(1, 1, 1);
                float x0 = (float)x, y0 = (float)y, x1 = x0 + conv->w, y1 = y0 + conv->h;
                glBegin(GL_QUADS);
                glTexCoord2f(0, 0);
                glVertex2f(x0, y0);
                glTexCoord2f(1, 0);
                glVertex2f(x1, y0);
                glTexCoord2f(1, 1);
                glVertex2f(x1, y1);
                glTexCoord2f(0, 1);
                glVertex2f(x0, y1);
                glEnd();
                glDisable(GL_BLEND);
                glDisable(GL_TEXTURE_2D);
                glDeleteTextures(1, &tex);
                SDL_FreeSurface(conv);
            };
            int yBase = 10;
            int yStep = 20;
            renderLine(line1, 10, yBase, SDL_Color{255, 255, 0, 255});
            yBase += yStep;
            renderLine(lineBias, 10, yBase, SDL_Color{255, 230, 180, 255});
            yBase += yStep;
            renderLine(lineHeading, 10, yBase, SDL_Color{255, 255, 180, 255});
            yBase += yStep;
            if (magFusion.calibrating || magFusion.calibrated)
            {
                renderLine(lineMagCal, 10, yBase, SDL_Color{255, 200, 120, 255});
                yBase += yStep;
            }
            renderLine(linePos, 10, yBase, SDL_Color{255, 255, 255, 255});
            yBase += yStep;
            renderLine(lineVel, 10, yBase, SDL_Color{200, 255, 255, 255});
            yBase += yStep;
            renderLine(lineAcc, 10, yBase, SDL_Color{200, 220, 255, 255});
            yBase += yStep;
            if (accCal.calibrating || accCal.calibrated)
            {
                renderLine(lineAccCal, 10, yBase, SDL_Color{200, 180, 255, 255});
                yBase += yStep;
            }
            renderLine(lineMag, 10, yBase, SDL_Color{200, 255, 200, 255});
            yBase += yStep;
            renderLine(lineGyro, 10, yBase, SDL_Color{255, 200, 180, 255});
            yBase += yStep;
            renderLine(lineTemp, 10, yBase, SDL_Color{255, 220, 120, 255});
            yBase += yStep;
            renderLine(linePSensor, 10, yBase, SDL_Color{220, 180, 255, 255});
            yBase += yStep;
            renderLine(lineLSensor, 10, yBase, SDL_Color{220, 255, 180, 255});
            yBase += yStep;
            renderLine(lineTick, 10, yBase, SDL_Color{180, 200, 200, 255});
            yBase += yStep;
            renderLine("A:Axes M:Model T:Text G:MagGraph X:AccGraph H:MagOnly J:GyroOnly C:MagCal K:AccCal R:Reset F:PosOnOff P:Zero Y:Mag ESC:Exit", 10, yBase, SDL_Color{180, 180, 180, 255});
            if (toggles.showMagGraph)
            {
                // Draw magnetometer graph in a reserved rectangle at bottom-right
                int gw = 300;
                int gh = 120;
                int gx = w - gw - 10;
                int gy = h - gh - 10;
                // Find min/max for autoscaling
                float minV = 1e9f, maxV = -1e9f;
                for (auto &m : magHistory)
                {
                    for (int i = 0; i < 3; i++)
                    {
                        minV = std::min(minV, m[i]);
                        maxV = std::max(maxV, m[i]);
                    }
                }
                if (minV == 1e9f)
                {
                    minV = 0;
                    maxV = 1;
                }
                if (fabs(maxV - minV) < 1e-6f)
                {
                    maxV = minV + 1.0f;
                }
                glColor3f(0.2f, 0.2f, 0.2f);
                glBegin(GL_LINE_LOOP);
                glVertex2f(gx, gy);
                glVertex2f(gx + gw, gy);
                glVertex2f(gx + gw, gy + gh);
                glVertex2f(gx, gy + gh);
                glEnd();
                auto drawSeries = [&](int idx, float r, float g, float b)
                {
                    glColor3f(r, g, b);
                    glBegin(GL_LINE_STRIP);
                    int N = (int)magHistory.size();
                    for (int i = 0; i < N; i++)
                    {
                        float v = magHistory[i][idx];
                        float norm = (v - minV) / (maxV - minV);
                        float x = gx + (float)i / (MAX_HISTORY - 1) * gw;
                        float y = gy + (1.0f - norm) * gh;
                        glVertex2f(x, y);
                    }
                    glEnd();
                };
                drawSeries(0, 1, 0, 0); // X - red
                drawSeries(1, 0, 1, 0); // Y - green
                drawSeries(2, 0, 0, 1); // Z - blue
                // Legend
                glColor3f(1, 1, 1);
                drawString(gx + 4, gy + 4, "Mag X(red) Y(green) Z(blue)", 1.0f);
                char rng[128];
                std::snprintf(rng, sizeof(rng), "min=%.2f max=%.2f", minV, maxV);
                drawString(gx + 4, gy + 18, rng, 1.0f);
            }
            if (toggles.showAccGraph)
            {
                // Draw accelerometer graph bottom-left
                int gw = 300;
                int gh = 120;
                int gx = 10;
                int gy = h - gh - 10;
                float minA = 1e9f, maxA = -1e9f;
                for (auto &a : accHistory)
                {
                    for (int i = 0; i < 3; i++)
                    {
                        minA = std::min(minA, a[i]);
                        maxA = std::max(maxA, a[i]);
                    }
                }
                if (minA == 1e9f)
                {
                    minA = 0;
                    maxA = 1;
                }
                if (fabs(maxA - minA) < 1e-6f)
                {
                    maxA = minA + 1.0f;
                }
                glColor3f(0.2f, 0.2f, 0.2f);
                glBegin(GL_LINE_LOOP);
                glVertex2f(gx, gy);
                glVertex2f(gx + gw, gy);
                glVertex2f(gx + gw, gy + gh);
                glVertex2f(gx, gy + gh);
                glEnd();
                auto drawAccSeries = [&](int idx, float r, float g, float b)
                {
                    glColor3f(r, g, b);
                    glBegin(GL_LINE_STRIP);
                    int N = (int)accHistory.size();
                    for (int i = 0; i < N; i++)
                    {
                        float v = accHistory[i][idx];
                        float norm = (v - minA) / (maxA - minA);
                        float x = gx + (float)i / (MAX_HISTORY - 1) * gw;
                        float y = gy + (1.0f - norm) * gh;
                        glVertex2f(x, y);
                    }
                    glEnd();
                };
                drawAccSeries(0, 1, 0, 0); // X
                drawAccSeries(1, 0, 1, 0); // Y
                drawAccSeries(2, 0, 0, 1); // Z
                glColor3f(1, 1, 1);
                drawString(gx + 4, gy + 4, "Acc X(red) Y(green) Z(blue)", 1.0f);
                char rngA[128];
                std::snprintf(rngA, sizeof(rngA), "min=%.2f max=%.2f", minA, maxA);
                drawString(gx + 4, gy + 18, rngA, 1.0f);
            }
            glEnable(GL_DEPTH_TEST);
            glMatrixMode(GL_MODELVIEW);
            glPopMatrix();
            glMatrixMode(GL_PROJECTION);
            glPopMatrix();
            glMatrixMode(GL_MODELVIEW);
        }
        // Title update every ~0.25s
        double tnow = SDL_GetTicks() * 0.001;
        if (tnow - lastTitleUpdate > 0.25)
        {
            char buf[160];
            std::snprintf(buf, sizeof(buf), "Yaw %.1f  Pitch %.1f  Roll %.1f (deg)", orient.yaw * 57.2958f, orient.pitch * 57.2958f, orient.roll * 57.2958f);
            SDL_SetWindowTitle(window, buf);
            lastTitleUpdate = tnow;
        }
        SDL_GL_SwapWindow(window);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (font)
        TTF_CloseFont(font);
    TTF_Quit();
    SDL_GL_DeleteContext(glctx);
    SDL_DestroyWindow(window);
    SDL_Quit();
    // Rayneo_Destroy(ctx);
    return 0;
}

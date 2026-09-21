# Simple Example

Minimal console program demonstrating the core RayNeo SDK flow: initialize, start service, enable IMU, poll and print events.

## Target
CMake target: `RayNeoSimpleExample`
Output binary (multi-config generators like MSVC): `build/examples/simple/<Config>/RayNeoSimpleExample[.exe]`

## Build
### Windows (vcpkg toolchain suggested)
```powershell
cmake -B build -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake -DRAYNEO_BUILD_EXAMPLES=ON
cmake --build build --config RelWithDebInfo --target RayNeoSimpleExample
```

### macOS / Linux
```bash
cmake -B build -DRAYNEO_BUILD_EXAMPLES=ON
cmake --build build --target RayNeoSimpleExample --config Release
```

## Run
### Windows
```powershell
cd build/examples/simple/RelWithDebInfo
./RayNeoSimpleExample.exe
```
### macOS / Linux
```bash
./build/examples/simple/RayNeoSimpleExample
```

## Typical Flow
1. Discover supported USB identities with `Rayneo_Discovery(devices, capacity, &count)`.
2. Select the single detected pair with `Rayneo_SetTargetVidPid(context, vid, pid)`.
3. `Rayneo_Start(context, 0)` to begin transport and the event thread.
4. Enable IMU streaming with `Rayneo_EnableImu(context)`.
5. Poll events with `Rayneo_PollEvent(context, &evt, 500)` until program exit.
6. `Rayneo_Stop(context)` then `Rayneo_Destroy(context)`.

## Device selection
Both examples discover supported VID/PID pairs automatically and print the matches.
They exit with a diagnostic if discovery fails, no pair is found, or multiple
different pairs are found. Connect only the glasses you want to use. Discovery
groups identical VID/PID pairs and cannot distinguish two identical devices.
Supported pairs are declared in `include/rayneo_api.h`. To identify a new pair:
- Windows: Device Manager -> Properties -> Details -> Hardware Ids.
- macOS: `system_profiler SPUSBDataType`.
- Linux: `lsusb`.

## Debug Environment Variables
Set before running to inspect frames or lifecycle:
```powershell
$env:RAYNEO_DEBUG_FRAMES=1
$env:RAYNEO_DEBUG_SHUTDOWN=1
./RayNeoSimpleExample.exe
```
POSIX:
```bash
RAYNEO_DEBUG_FRAMES=1 RAYNEO_DEBUG_SHUTDOWN=1 ./RayNeoSimpleExample
```

## Event Types (Subset)
- DEVICE_ATTACHED / DEVICE_DETACHED
- DEVICE_INFO (after `Rayneo_RequestDeviceInfo` or automatic)
- IMU_SAMPLE (requires enabling IMU)

## Troubleshooting
| Symptom | Cause | Fix |
|---------|-------|-----|
| 0xC000007B on start | Missing/arch-mismatch DLL | Ensure `RayNeoSDK.dll` & `libusb-1.0.dll` are x64 and present |
| No IMU data | IMU not enabled | Call `Rayneo_EnableImu(context, 1)` earlier |
| No device info | Not requested yet | Call `Rayneo_RequestDeviceInfo(context)` |
| macOS not starting | HID match failed | Check VID/PID, replug device |

## Next Ideas
- Add CLI args for vid/pid selection.
- Output CSV for IMU samples.
- Integrate simple ring buffer for averaging.

---
*README for `examples/simple`*

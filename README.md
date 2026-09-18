# RayNeo SDK (Work-in-Progress)

A lightweight C/C++ SDK and examples for interacting with **RayNeo Air3s Pro** (and probably other RayNeo's) devices. 

<div align="center">
  <img src="images/tcl-rayneo-air3s-pro.png" alt="RayNeo Air3s Pro" width="400px">
</div>

The project currently focuses on:

- Cross-platform transport layer
  - libusb on Windows/Linux
  - Native HID (IOKit) on macOS (integrated directly in the SDK)
- Simple C API (`include/rayneo/rayneo_api.h`) for:
  - Device attach/detach events
  - IMU streaming (accel / gyro / magnet / temperature / tick)
  - Basic device info block
  - Round-trip command/ack frames
- Example applications:
  - `examples/simple` – console usage demo
  - `examples/orientation_demo` – SDL2 + OpenGL visualization
- Optional OpenVR stub driver prototype (if enabled)

> NOTE: This repository is evolving. Some components (e.g. higher-level service logic, richer device info parsing, error reporting) are intentionally minimal.

## Download prebuilt Binaries

**SDK:**

[![Build & Release RayNeo SDK](https://github.com/verncat/RayNeo-Air-3S-Pro-OpenVR/actions/workflows/release.yml/badge.svg)](https://github.com/verncat/RayNeo-Air-3S-Pro-OpenVR/actions/workflows/release.yml) 

[Windows](https://github.com/verncat/RayNeo-Air-3S-Pro-OpenVR/releases/latest/download/rayneo-sdk-windows-latest.tar.gz)
[MacOS](https://github.com/verncat/RayNeo-Air-3S-Pro-OpenVR/releases/latest/download/rayneo-sdk-macos-latest.tar.gz)
[Linux](https://github.com/verncat/RayNeo-Air-3S-Pro-OpenVR/releases/latest/download/rayneo-sdk-ubuntu-latest.tar.gz)

**SteamVR Driver:**
New driver has moved to separate repo: https://github.com/verncat/RayNeo-Air-3S-Pro-OpenVR-Driver

Old driver:
[![SteamVR Driver Build & Release](https://github.com/verncat/RayNeo-Air-3S-Pro-OpenVR/actions/workflows/steamvr-driver.yml/badge.svg)](https://github.com/verncat/RayNeo-Air-3S-Pro-OpenVR/actions/workflows/steamvr-driver.yml)

[Download Linux64+Win64](https://github.com/verncat/RayNeo-Air-3S-Pro-OpenVR/releases/latest/download/steamvr_rayneo_driver-latest.zip)


## Release version

Edit `RAYNEO_API_VERSION_MAJOR`, `RAYNEO_API_VERSION_MINOR`, and
`RAYNEO_API_VERSION_PATCH` in `include/rayneo_api.h` to set the release SemVer
(currently `1.3.0`). Use decimal integers without leading zeros. These three
macros are the single source of the version; `RAYNEO_VERSION_STRING` is derived
automatically. Major and minor must fit in 16 bits. `RAYNEO_API_VERSION` and
`Rayneo_GetApiVersion()` retain the packed major/minor format for compatibility;
patch is available through its macro and the version string.

On pushes to `main`, SDK and SteamVR builds run only when the version differs
from the header before the push and the corresponding release tag does not
exist. Manual runs compare against the parent commit. Introducing the release
macro counts as a version change; edits that keep the version unchanged do not.

Tags are `sdk-v<VERSION>` and `steamvr-driver-v<VERSION>`. Release notes include
commits since the previous reachable tag for that product and a full diff link.
Legacy timestamp tags are supported. Without a previous tag, all commits are
included. The three numeric macros define stable releases without prerelease
or build suffixes.

## Repository Layout

```
CMakeLists.txt                # Root build script
include/rayneo/rayneo_api.h   # Public C API header
src/RayneoApi.cpp             # Core SDK implementation
main.cpp                      # Basic client executable
examples/                     # Example apps
openvr_driver/                # Optional stub driver
rayneoSDKHeaders/             # Additional internal headers (not yet fully used) from original SDK
thirdparties/openvr/          # Third-party OpenVR bits
```

## Building

### Cloning (Git Submodules)

This repository uses git submodules (e.g. `thirdparties/openvr`). Ensure they are fetched before building:

Clone with submodules in one step:
```bash
git clone --recursive https://github.com/verncat/RayNeo-Air-3S-Pro-OpenVR.git
```

### Prerequisites

- CMake >= 3.16
- A C++17 capable compiler
- Windows: Visual Studio 2022 + vcpkg dependencies (SDL2, SDL2_ttf, libusb, etc.)
- macOS: Xcode command line tools (IOKit/CoreFoundation frameworks available by default)
- Linux: libusb-1.0 dev package, SDL2/SDL2_ttf dev packages, OpenGL

### Configure & Build (Windows example)

```powershell
# From the repository root
cmake -B build -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake -DRAYNEO_BUILD_EXAMPLES=ON -DRAYNEO_BUILD_OPENVR_DRIVER=OFF
cmake --build build --config RelWithDebInfo --target RayNeoSDK
cmake --build build --config RelWithDebInfo --target RayNeoOrientationDemo
```

### macOS

```bash
cmake -B build -DRAYNEO_BUILD_EXAMPLES=ON -DRAYNEO_BUILD_OPENVR_DRIVER=OFF
cmake --build build --target RayNeoSDK --config Release
```

### Linux

```bash
cmake -B build -DRAYNEO_BUILD_EXAMPLES=ON -DRAYNEO_BUILD_OPENVR_DRIVER=OFF
cmake --build build --target RayNeoSDK --config Release
```

### Runtime Deployment (Windows)

The build copies `RayNeoSDK.dll` and (if needed) `libusb-1.0.dll` next to example executables. Additional SDL2 / freetype / zlib / png dependencies are typically auto-copied by vcpkg's app-local mechanism.

## macOS Transport Notes

macOS path uses IOKit HID APIs directly. A worker thread runs a CFRunLoop, registers an input report callback, and feeds frames into the same parser used by libusb path.

## Roadmap / TODO
- Move OpenVR Driver to the another repo
- The OpenVR stub driver shouldn't be a not stub lol
- Add crossplatform script for deploy SteamVR driver to their drivers path 
- Expand device info parsing into structured fields
- Integrate richer error codes & diagnostics
- Add unit tests for frame parser & event queue

## License

This project is released under the **MIT License** — see the [LICENSE](LICENSE) file.

## Contributing

PRs / patches welcome once the initial API surface stabilizes. Please:

1. Keep changes platform-agnostic where possible
2. Add comments for non-obvious protocol fields
3. Avoid introducing heavy dependencies without discussion

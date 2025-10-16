#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>

#include "rayneo_api.h"

int main() {
    std::cout << "RayNeo simple example (C API)\n";
    std::cout << "API version: " << Rayneo_GetApiVersion() << "\n";

    RAYNEO_Context ctx{};
    if (Rayneo_Create(&ctx) != RAYNEO_OK) {
        std::cerr << "Failed to create context" << std::endl;
        return 1;
    }

    const uint16_t kVid = 0x1BBB; 
    const uint16_t kPid = 0xAF50;
    Rayneo_SetTargetVidPid(ctx, kVid, kPid);

    if (Rayneo_Start(ctx, 0) != RAYNEO_OK) {
        std::cerr << "Device start failed (Device not found?)" << std::endl;
        Rayneo_Destroy(ctx);
        return 1;
    }

    // Включаем IMU и запрашиваем информацию об устройстве.
    Rayneo_EnableImu(ctx);
    Rayneo_DisplaySet3D(ctx);
    Rayneo_RequestDeviceInfo(ctx);

    auto start = std::chrono::steady_clock::now();
    while (true) {
        RAYNEO_Event evt{};
        auto rc = Rayneo_PollEvent(ctx, &evt, 500); // ждём события до 500мс
        if (rc == RAYNEO_OK) {
            if (evt.type == RAYNEO_EVENT_IMU_SAMPLE) {
                std::cout << "IMU tick=" << evt.data.imu.tick
                          << " acc=" << evt.data.imu.acc[0] << "," << evt.data.imu.acc[1] << "," << evt.data.imu.acc[2]
                          << " gyro(dps)=" << evt.data.imu.gyroDps[0] << "," << evt.data.imu.gyroDps[1] << "," << evt.data.imu.gyroDps[2]
                          << " temp=" << evt.data.imu.temperature
                          << " psensor=" << evt.data.imu.psensor
                          << " lsensor=" << evt.data.imu.lsensor
                          << std::endl;
            } else if (evt.type == RAYNEO_EVENT_DEVICE_INFO) {
                const auto &d = evt.data.info;
                std::cout << "DeviceInfo:\n"
                          << "  tick=" << d.tick << " value=0x" << std::hex << (int)d.value << std::dec << "\n"
                          << "  cpuid="; for(int i=0;i<12;i++){ std::cout << std::hex << (int)d.cpuid[i]; if(i<11) std::cout << ':'; } std::cout << std::dec << "\n"
                          << "  board_id=0x" << std::hex << (int)d.board_id << std::dec
                          << " sensor_on=" << (int)d.sensor_on
                          << " support_fov=" << (int)d.support_fov << "\n"
                          << "  date=" << d.date << " (" << d.year << '-' << (int)d.month << '-' << (int)d.day << ")\n"
                          << "  fps=" << (int)d.glasses_fps
                          << " luminance=" << (int)d.luminance << "/" << (int)d.max_luminance
                          << " volume=" << (int)d.volume << "/" << (int)d.max_volume
                          << " side_by_side=" << (int)d.side_by_side << "\n"
                          << "  psensor_enable=" << (int)d.psensor_enable
                          << " audio_mode=" << (int)d.audio_mode
                          << " dp_status=" << (int)d.dp_status
                          << " status3=" << (int)d.status3 << "\n"
                          << "  psensor_valid=" << (int)d.psensor_valid
                          << " lsensor_valid=" << (int)d.lsensor_valid
                          << " gyro_valid=" << (int)d.gyro_valid
                          << " magnet_valid=" << (int)d.magnet_valid << "\n"
                          << "  reserve1=" << d.reserve1 << " reserve2=" << d.reserve2 << "\n"
                          << "  support_panel_color_adjust=" << (int)d.support_panel_color_adjust
                          << " flag=0x" << std::hex << (int)d.flag << std::dec << "\n";
                // Также при необходимости можно вывести первые 8 сырых байт
                std::cout << "  raw[0..7]: "; for(int i=0;i<8;i++) std::cout << std::hex << (int)d.raw[i] << ' '; std::cout << std::dec << "\n";
                // Rayneo_RequestDeviceInfo(ctx);
            } else if (evt.type == RAYNEO_EVENT_DEVICE_ATTACHED) {
                std::cout << "Device attached" << std::endl;
            } else if (evt.type == RAYNEO_EVENT_NOTIFY) {
                std::cout << "NOTIFY code=0x" << std::hex << evt.data.notify.code << std::dec
                          << " message=" << evt.data.notify.message << std::endl;
            } else if (evt.type == RAYNEO_EVENT_LOG) {
                // Прочие логи
                std::cout << "LOG(level=" << evt.data.log.level << "): " << evt.data.log.message << std::endl;
            } else if (evt.type == RAYNEO_EVENT_DEVICE_DETACHED) {
                std::cout << "Device detached" << std::endl;
                break; // завершаем
            }
        }
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - start).count() > 10) {
            std::cout << "Timeout demo end" << std::endl;
            break;
        }
    }
    Rayneo_DisableImu(ctx);
    Rayneo_DisplaySet2D(ctx);

    // Rayneo_Destroy сам вызывает Rayneo_Stop (двойной Stop может крашиться на некоторых версиях)
    // Rayneo_Destroy(ctx);
    std::cout << "End main\n";
    return 0;
}

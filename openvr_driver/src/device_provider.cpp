//============ Copyright (c) Valve Corporation, All rights reserved. ============
#include "device_provider.h"

#include "driverlog.h"
#include <cmath>

// Simple global pointer to current provider (single instance assumption)
static MyDeviceProvider* g_device_provider_instance = nullptr;

//-----------------------------------------------------------------------------
// Purpose: This is called by vrserver after it receives a pointer back from HmdDriverFactory.
// You should do your resources allocations here (**not** in the constructor).
//-----------------------------------------------------------------------------
vr::EVRInitError MyDeviceProvider::Init( vr::IVRDriverContext *pDriverContext )
{
	g_device_provider_instance = this;
	// We need to initialise our driver context to make calls to the server.
	// OpenVR provides a macro to do this for us.
	VR_INIT_SERVER_DRIVER_CONTEXT( pDriverContext );

	// Initialize RayNeo context before creating devices (if we need early info).
	InitRayneo();

	// Initialize our HMD tracked device.
	my_hmd_device_ = std::make_unique< MyHMDControllerDeviceDriver >();

	// TrackedDeviceAdded returning true means we have had our device added to SteamVR.
	if ( !vr::VRServerDriverHost()->TrackedDeviceAdded( my_hmd_device_->MyGetSerialNumber().c_str(), vr::TrackedDeviceClass_HMD, my_hmd_device_.get() ) )
	{
		DriverLog( "Failed to create hmd device!" );
		return vr::VRInitError_Driver_Unknown;
	}

	return vr::VRInitError_None;
}

//-----------------------------------------------------------------------------
// Purpose: Tells the runtime which version of the API we are targeting.
// Helper variables in the header you're using contain this information, which can be returned here.
//-----------------------------------------------------------------------------
const char *const *MyDeviceProvider::GetInterfaceVersions()
{
	return vr::k_InterfaceVersions;
}

//-----------------------------------------------------------------------------
// Purpose: This function is deprecated and never called. But, it must still be defined, or we can't compile.
//-----------------------------------------------------------------------------
bool MyDeviceProvider::ShouldBlockStandbyMode()
{
	return false;
}

//-----------------------------------------------------------------------------
// Purpose: This is called in the main loop of vrserver.
// Drivers *can* do work here, but should ensure this work is relatively inexpensive.
// A good thing to do here is poll for events from the runtime or applications
//-----------------------------------------------------------------------------
void MyDeviceProvider::RunFrame()
{
	// call our devices to run a frame
	if ( my_hmd_device_ != nullptr )
	{
		my_hmd_device_->MyRunFrame();
	}


	//Now, process events that were submitted for this frame.
	vr::VREvent_t vrevent{};
	while ( vr::VRServerDriverHost()->PollNextEvent( &vrevent, sizeof( vr::VREvent_t ) ) )
	{
		if ( my_hmd_device_ != nullptr )
		{
			my_hmd_device_->MyProcessEvent( vrevent );
		}
	}
}

//-----------------------------------------------------------------------------
// Purpose: This function is called when the system enters a period of inactivity.
// The devices might want to turn off their displays or go into a low power mode to preserve them.
//-----------------------------------------------------------------------------
void MyDeviceProvider::EnterStandby()
{
}

//-----------------------------------------------------------------------------
// Purpose: This function is called after the system has been in a period of inactivity, and is waking up again.
// Turn back on the displays or devices here.
//-----------------------------------------------------------------------------
void MyDeviceProvider::LeaveStandby()
{
}

//-----------------------------------------------------------------------------
// Purpose: This function is called just before the driver is unloaded from vrserver.
// Drivers should free whatever resources they have acquired over the session here.
// Any calls to the server is guaranteed to be valid before this, but not after it has been called.
//-----------------------------------------------------------------------------
void MyDeviceProvider::Cleanup()
{
	// Our controller devices will have already deactivated. Let's now destroy them.
	my_hmd_device_ = nullptr;
	StopRayneo();
	g_device_provider_instance = nullptr; // clear global instance
}

void MyDeviceProvider::InitRayneo()
{
	// Reinitialize if already present
	if (rayneo_ctx_) {
		StopRayneo();
	}

	if (Rayneo_Create(&rayneo_ctx_) != RAYNEO_OK || !rayneo_ctx_) {
		DriverLog("[provider] Rayneo_Create failed");
		rayneo_ctx_ = nullptr;
		return;
	}

	const uint16_t kVid = 0x1BBB;
	const uint16_t kPid = 0xAF50;
	Rayneo_SetTargetVidPid(rayneo_ctx_, kVid, kPid);

	RAYNEO_Result startRc = Rayneo_Start(rayneo_ctx_, 0);
	if (startRc != RAYNEO_OK) {
		DriverLog("[provider] Rayneo_Start failed: %d (Device not found?)", (int)startRc);
		Rayneo_Destroy(rayneo_ctx_);
		rayneo_ctx_ = nullptr;
		return;
	}
	rayneo_started_ = true;

	Rayneo_RequestDeviceInfo(rayneo_ctx_);

	RAYNEO_Result r3d = Rayneo_DisplaySet3D(rayneo_ctx_);
	if (r3d == RAYNEO_OK) {
		DriverLog("[provider] RayNeo_DisplaySet3D success");
	} else {
		DriverLog("[provider] RayNeo_DisplaySet3D failed: %d", (int)r3d);
	}

	RAYNEO_Result imuRc = Rayneo_EnableImu(rayneo_ctx_);
	if (imuRc == RAYNEO_OK) {
		DriverLog("[provider] RayNeo_EnableImu success");
	} else {
		DriverLog("[provider] RayNeo_EnableImu failed: %d", (int)imuRc);
	}

	StartRayneoEventThread();
}

void MyDeviceProvider::StartRayneoEventThread()
{
	if (rayneo_event_thread_running_.load() || !rayneo_ctx_ || !rayneo_started_) return;
	rayneo_event_thread_running_.store(true);
	rayneo_event_thread_ = std::thread(&MyDeviceProvider::RayneoEventLoop, this);
}

void MyDeviceProvider::RayneoEventLoop()
{
	while (rayneo_event_thread_running_.load()) {
		if (!rayneo_ctx_ || !rayneo_started_) break;
		RAYNEO_Event evt{};
		auto rc = Rayneo_PollEvent(rayneo_ctx_, &evt, 500);
		if (rc == RAYNEO_OK) {
			if (evt.type == RAYNEO_EVENT_DEVICE_DETACHED) {
				DriverLog("[provider] RayNeo device detached");
				break;
			} else if (evt.type == RAYNEO_EVENT_DEVICE_ATTACHED) {
				DriverLog("[provider] RayNeo device attached");
			} else if (evt.type == RAYNEO_EVENT_IMU_SAMPLE) {
				// Integrate gyro to update orientation quaternion.
				// Basic incremental quaternion integration (no drift correction).
				const auto &s = evt.data.imu;
				if (s.valid) {
					std::lock_guard<std::mutex> lock(imu_mutex_);
					// Compute dt (assuming tick is milliseconds)
					float dt = 0.0f;
					if (last_imu_tick_ != 0 && s.tick > last_imu_tick_) {
						uint32_t dtMs = s.tick - last_imu_tick_;
						dt = static_cast<float>(dtMs) * 0.001f;
					}
					last_imu_tick_ = s.tick;

					// Angular velocity in rad/s (use gyroRad if filled else convert from gyroDps)
					float wx = s.gyroRad[0];
					float wy = s.gyroRad[1];
					float wz = s.gyroRad[2];
					if (wx == 0.f && wy == 0.f && wz == 0.f) {
						// fallback convert from dps if rad array not provided
						const float deg2rad = 3.14159265358979323846f / 180.f;
						wx = s.gyroDps[0] * deg2rad;
						wy = s.gyroDps[1] * deg2rad;
						wz = s.gyroDps[2] * deg2rad;
					}

					if (dt > 0.f) {
						// Form delta quaternion from angular velocity vector magnitude
						float angle = std::sqrt(wx*wx + wy*wy + wz*wz) * dt; // radians
						if (angle > 0.f) {
							float axis_x = wx;
							float axis_y = wy;
							float axis_z = wz;
							float invMag = 1.f / std::sqrt(axis_x*axis_x + axis_y*axis_y + axis_z*axis_z);
							axis_x *= invMag; axis_y *= invMag; axis_z *= invMag;
							float half = angle * 0.5f;
							float sinHalf = std::sin(half);
							float dq_w = std::cos(half);
							float dq_x = axis_x * sinHalf;
							float dq_y = axis_y * sinHalf;
							float dq_z = axis_z * sinHalf;
							// Multiply current quaternion by delta: q_new = q * dq
							float nw = imu_q_w_*dq_w - imu_q_x_*dq_x - imu_q_y_*dq_y - imu_q_z_*dq_z;
							float nx = imu_q_w_*dq_x + imu_q_x_*dq_w + imu_q_y_*dq_z - imu_q_z_*dq_y;
							float ny = imu_q_w_*dq_y - imu_q_x_*dq_z + imu_q_y_*dq_w + imu_q_z_*dq_x;
							float nz = imu_q_w_*dq_z + imu_q_x_*dq_y - imu_q_y_*dq_x + imu_q_z_*dq_w;
							// Normalize
							float nrm = std::sqrt(nw*nw + nx*nx + ny*ny + nz*nz);
							if (nrm > 0.f) { nw /= nrm; nx /= nrm; ny /= nrm; nz /= nrm; }
							imu_q_w_ = nw; imu_q_x_ = nx; imu_q_y_ = ny; imu_q_z_ = nz;
						}
					}
				}
			} else if (evt.type == RAYNEO_EVENT_DEVICE_INFO) {
				DriverLog("[provider] RayNeo device info received");
			} else if (evt.type == RAYNEO_EVENT_NOTIFY) {
				DriverLog("[provider] RayNeo notify code=0x%X msg=%s", (unsigned)evt.data.notify.code, evt.data.notify.message);
				if (evt.data.notify.code == RAYNEO_NOTIFY_SLEEP) {
					sleeping_.store(true);
					DriverLog("[provider] Sleep state entered");
				} else if (evt.data.notify.code == RAYNEO_NOTIFY_WAKE) {
					sleeping_.store(false);
					DriverLog("[provider] Wake state");
				} else if (evt.data.notify.code == RAYNEO_NOTIFY_BUTTON) {
					// Recenter();
					button_notify_pending_.store(true);
					DriverLog("[provider] Recenter applied");
				}
			} else if (evt.type == RAYNEO_EVENT_LOG) {
				DriverLog("[provider] RayNeo log(level=%d): %s", (int)evt.data.log.level, evt.data.log.message);
			}
		}
	}
	rayneo_event_thread_running_.store(false);
}

MyDeviceProvider* GetMyDeviceProviderInstance() { return g_device_provider_instance; }

void MyDeviceProvider::StopRayneo()
{
	Rayneo_DisableImu(rayneo_ctx_);
	Rayneo_DisplaySet2D(rayneo_ctx_);

	rayneo_event_thread_running_.store(false);
	if (rayneo_event_thread_.joinable()) {
		rayneo_event_thread_.join();
	}
	if (rayneo_ctx_) {
		if (rayneo_started_) {
			Rayneo_Stop(rayneo_ctx_);
			rayneo_started_ = false;
		}
		Rayneo_Destroy(rayneo_ctx_);
		rayneo_ctx_ = nullptr;
		DriverLog("[provider] RayNeo SDK context destroyed");
	}
}
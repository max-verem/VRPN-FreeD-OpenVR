#pragma once

#include <map>
#include <list>
#include <array>
#include <memory>
#include <openvr.h>
#include <quat.h>
#include <vrpn_Connection.h>
#include "vrpn_Tracker_OpenVR_HMD.h"
#include "vrpn_Tracker_OpenVR_Controller.h"
#include "vrpn_Tracker_Camera.h"

/// Sensor numbers in SteamVR and for tracking
static const auto HMD_SENSOR = 0;
static const auto CONTROLLER_SENSORS = { 1, 2 };
static const auto MAX_CONTROLLER_ID = 2;

//static const auto NUM_TRACKERS = 3;
//static const auto NUM_ANALOGS = 7;
//static const auto NUM_BUTTONS = 14;

/// For the HMD and two controllers
static const std::array<uint32_t, 3> FIRST_BUTTON_ID{ 0, 2, 8 };
static const std::array<uint32_t, 3> FIRST_ANALOG_ID{ 0, 1, 4 };

/// Offsets from the first button ID for a controller that a button is
/// reported.
static const auto SYSTEM_BUTTON_OFFSET = 0;
static const auto MENU_BUTTON_OFFSET = 1;
static const auto GRIP_BUTTON_OFFSET = 2;
static const auto TRACKPAD_TOUCH_BUTTON_OFFSET = 3;
static const auto TRACKPAD_CLICK_BUTTON_OFFSET = 4;
static const auto TRIGGER_BUTTON_OFFSET = 4;

/// Offsets from the first button ID for a controller that an analog is
/// reported.
static const auto TRACKPAD_X_ANALOG_OFFSET = 0;
static const auto TRACKPAD_Y_ANALOG_OFFSET = 1;
static const auto TRIGGER_ANALOG_OFFSET = 2;

class vrpn_Server_OpenVR {
public:
	vrpn_Server_OpenVR(int argc, char *argv[]);
	~vrpn_Server_OpenVR();
	void mainloop();
    int sleep_interval;
    static const std::string getDeviceClassName(vr::ETrackedDeviceClass device_class_id);
    static const std::string getDeviceSerial(vr::TrackedDeviceIndex_t trackedDeviceIndex, vr::IVRSystem * vr);
private:
	std::unique_ptr<vr::IVRSystem> vr{ nullptr };
	vrpn_Connection *connection;
    std::map<vr::TrackedDeviceIndex_t, std::unique_ptr<vrpn_Tracker_OpenVR>> devices{};
    std::list<std::unique_ptr<vrpn_Tracker_Camera>> cameras{};
    q_xyz_quat_type stk, ref;
};


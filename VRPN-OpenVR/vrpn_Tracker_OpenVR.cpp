#include "vrpn_Tracker_OpenVR.h"
#include <openvr.h>
#include <quat.h>
#include <iostream>

// -------------------------------------------------------------------------------------
//
// https://www.codeproject.com/Articles/1171122/How-to-Get-Raw-Positional-Data-from-HTC-Vive
// https://github.com/Omnifinity/OpenVR-Tracking-Example/blob/master/HTC%20Lighthouse%20Tracking%20Example/LighthouseTracking.cpp
// https://steamcommunity.com/app/358720/discussions/0/358417008714224220/#c359543542244499836
//

// Get the quaternion representing the rotation
static vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix) {
    vr::HmdQuaternion_t q;

    q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
    return q;
}

// Get the vector representing the position
static vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t matrix) {
    vr::HmdVector3_t vector;

    vector.v[0] = matrix.m[0][3];
    vector.v[1] = matrix.m[1][3];
    vector.v[2] = matrix.m[2][3];

    return vector;
}

// -------------------------------------------------------------------------------------

vrpn_Tracker_OpenVR::vrpn_Tracker_OpenVR(const std::string& name, vrpn_Connection* connection, vr::IVRSystem * vr, vr::TrackedDeviceIndex_t trackedDeviceIndex) :
	vrpn_Tracker(name.c_str(), connection), name(name), vr(vr), trackedDeviceIndex(trackedDeviceIndex)
{
	// Initialize the vrpn_Tracker
    // We track each device separately so this will only ever have one sensor
	vrpn_Tracker::num_sensors = 1;
    device_class_id = vr->GetTrackedDeviceClass(trackedDeviceIndex);
}

void vrpn_Tracker_OpenVR::updateTracking(vr::TrackedDevicePose_t *pose)
{
    // Sensor, doesn't change since we are tracking individual devices
    d_sensor = 0;

    // Position
    pos[0] = pose->mDeviceToAbsoluteTracking.m[0][3];
    pos[1] = pose->mDeviceToAbsoluteTracking.m[1][3];
    pos[2] = pose->mDeviceToAbsoluteTracking.m[2][3];

    // Quaternion
    q_type q_current;

    // use *native* or lib method for cal quat from transform matrix
#if 1
    ConvertSteamVRMatrixToQMatrix(pose->mDeviceToAbsoluteTracking, matrix);
    q_from_col_matrix(q_current, matrix);  // void q_from_col_matrix (q_type destQuat, const q_matrix_type matrix);
#else
    vr::HmdQuaternion_t q_current_o = GetRotation(pose->mDeviceToAbsoluteTracking);
    q_current[0] = q_current_o.x;
    q_current[1] = q_current_o.y;
    q_current[2] = q_current_o.z;
    q_current[3] = q_current_o.w;
#endif

    // prerotate HTC Vive Tracker
    if (device_class_id == vr::TrackedDeviceClass_GenericTracker)
    {
        q_type prerot90;
        q_from_euler(prerot90, 0, 0, -3.1415926 / 2.0); // double yaw, double pitch, double roll
        q_mult(q_current, q_current, prerot90);
    };

    // save it
    d_quat[0] = q_current[0];
    d_quat[1] = q_current[1];
    d_quat[2] = q_current[2];
    d_quat[3] = q_current[3];

    // Pack message
	vrpn_gettimeofday(&timestamp, NULL);
	char msgbuf[1000];
	vrpn_int32 len = vrpn_Tracker::encode_to(msgbuf);
	if (d_connection->pack_message(len, timestamp, position_m_id, d_sender_id, msgbuf, vrpn_CONNECTION_LOW_LATENCY)) {
		std::cerr << " Can't write message";
	}
}

void vrpn_Tracker_OpenVR::getPose(q_xyz_quat_type *pose)
{
    pose->quat[0] = d_quat[0];
    pose->quat[1] = d_quat[1];
    pose->quat[2] = d_quat[2];
    pose->quat[3] = d_quat[3];

    pose->xyz[0] = pos[0];
    pose->xyz[1] = pos[1];
    pose->xyz[2] = pos[2];
}

std::string vrpn_Tracker_OpenVR::getName()
{
    return name;
}

void vrpn_Tracker_OpenVR::mainloop() {
    vrpn_gettimeofday( &(vrpn_Tracker_OpenVR::timestamp), NULL );
	vrpn_Tracker::server_mainloop();
}

void vrpn_Tracker_OpenVR::ConvertSteamVRMatrixToQMatrix(const vr::HmdMatrix34_t &matPose, q_matrix_type &matrix) {
	matrix[0][0] = matPose.m[0][0];
	matrix[1][0] = matPose.m[1][0];
	matrix[2][0] = matPose.m[2][0];
	matrix[3][0] = 0.0;
	matrix[0][1] = matPose.m[0][1];
	matrix[1][1] = matPose.m[1][1];
	matrix[2][1] = matPose.m[2][1];
	matrix[3][1] = 0.0;
	matrix[0][2] = matPose.m[0][2];
	matrix[1][2] = matPose.m[1][2];
	matrix[2][2] = matPose.m[2][2];
	matrix[3][2] = 0.0;
	matrix[0][3] = matPose.m[0][3];
	matrix[1][3] = matPose.m[1][3];
	matrix[2][3] = matPose.m[2][3];
	matrix[3][3] = 1.0f;
}
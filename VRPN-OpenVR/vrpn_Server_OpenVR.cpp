#include <iostream>
#include <string>
#include <quat.h>
#include <vrpn_Connection.h>
#include "vrpn_Server_OpenVR.h"
#include "console.h"

vrpn_Server_OpenVR::vrpn_Server_OpenVR(int argc, char *argv[])
{
    int cam_idx = 0;
    std::string connectionName = "";
    int listen_vrpn_port = vrpn_DEFAULT_LISTEN_PORT_NO;

    // set quaternions default
    q_type I = Q_ID_QUAT;
    q_copy(stk.quat, I);
    q_copy(ref.quat, I);

    sleep_interval = 1;

    // Initialize OpenVR
    vr::EVRInitError eError = vr::VRInitError_None;
    vr = std::unique_ptr<vr::IVRSystem>(vr::VR_Init(&eError, vr::VRApplication_Utility/*VRApplication_Background*/)); /// https://github.com/ValveSoftware/openvr/wiki/API-Documentation
    if (eError != vr::VRInitError_None)
    {
//        vr.reset(nullptr);
        std::cerr << "Unable to init VR runtime: " << vr::VR_GetVRInitErrorAsEnglishDescription(eError) << std::endl;
        exit(1);
    };

    // Process arguments
    if (argc > 1)
    {
        for (int p = 1; p < argc;)
        {
            if (!strcmp(argv[p], "port") && (p + 1) <= argc)        // 1 argument: port <listen port>
            {
                listen_vrpn_port = atoi(argv[p + 1]);
                p += 2;
            }
            else if (!strcmp(argv[p], "sleep_interval") && (p + 1) <= argc)  // 1 argument: sleep <listen port>
            {
                sleep_interval = atoi(argv[p + 1]);
                p += 2;
            }
            else if (!strcmp(argv[p], "ref") && (p + 6) <= argc)    // 3 argument: ref <x> <y> <z> <yaw'> <roll'> <pitch'>
            {
                ref.xyz[0] = atof(argv[p + 1]);
                ref.xyz[1] = atof(argv[p + 2]);
                ref.xyz[2] = atof(argv[p + 3]);

                q_from_euler(ref.quat,
                    Q_DEG_TO_RAD(atof(argv[p + 4])),
                    Q_DEG_TO_RAD(atof(argv[p + 5])),
                    Q_DEG_TO_RAD(atof(argv[p + 6])));

                p += 7;
            }
            else if (!strcmp(argv[p], "cam") && (p + 8) <= argc)    // 8 argument: cam <NAME> <TRACKER SERIAL> <x> <y> <z> <yaw'> <roll'> <pitch'>
            {
                // Initialize VRPN Connection
                if (connectionName == "")
                {
                    connectionName = ":" + std::to_string(listen_vrpn_port);
                    connection = vrpn_create_server_connection(connectionName.c_str());
                }

                q_xyz_quat_type arm;
                std::unique_ptr<vrpn_Tracker_Camera> newCAM;
                std::string name, serial;

                // build name
                name = "virtual/"; name += argv[p + 1];

                // serial
                serial = argv[p + 2];

                // build arm
                arm.xyz[0] = atof(argv[p + 3]);
                arm.xyz[1] = atof(argv[p + 4]);
                arm.xyz[2] = atof(argv[p + 5]);

                q_from_euler(arm.quat,
                    Q_DEG_TO_RAD(atof(argv[p + 6])),
                    Q_DEG_TO_RAD(atof(argv[p + 7])),
                    Q_DEG_TO_RAD(atof(argv[p + 8])));

                // build cam class
                newCAM = std::make_unique<vrpn_Tracker_Camera>(cam_idx++, name, connection, serial, &arm);

                p += 9;

                /* check if dst for free-d specified */
                while (p < argc && !strcmp(argv[p], "filter"))
                {
                    if (!strcmp(argv[p + 1], "kalman"))
                    {
                        newCAM.get()->filterAdd(new filter_kalman(atof(argv[p + 2]), atof(argv[p + 3])));
                        p += 4;
                    }
                    else if (!strcmp(argv[p + 1], "exp1"))
                    {
                        newCAM.get()->filterAdd(new filter_exp1(atof(argv[p + 2]), atof(argv[p + 3])));
                        p += 4;
                    }
                    else if (!strcmp(argv[p + 1], "exp1dyn"))
                    {
                        newCAM.get()->filterAdd(new filter_exp1dyn(atof(argv[p + 2]), atof(argv[p + 3])));
                        p += 4;
                    }
                    else if (!strcmp(argv[p + 1], "exp1pasha"))
                    {
                        newCAM.get()->filterAdd(new filter_exp1pasha(atof(argv[p + 2]), atof(argv[p + 3])));
                        p += 4;
                    }
                    else
                        break;
                }

                /* check if dst for free-d specified */
                while (p < argc && !strcmp(argv[p], "freed") && (p + 1) <= argc)
                {
                    newCAM.get()->freedAdd(argv[p + 1]);
                    p += 2;
                }

                cameras.push_back(std::move(newCAM));
            }
            else
            {
                std::cerr << "Failed to parse argument [" << argv[p] << "], either unknown or wrong parameters count" << std::endl;
                exit(1);
            }
        }
    }

    // Initialize VRPN Connection
    if (connectionName == "")
    {
        connectionName = ":" + std::to_string(listen_vrpn_port);
        connection = vrpn_create_server_connection(connectionName.c_str());
    }

    console_setup(&console_in, &console_out);
}


vrpn_Server_OpenVR::~vrpn_Server_OpenVR() {
    vr::VR_Shutdown();
    if (connection) {
        connection->removeReference();
        connection = NULL;
    }
}

static void console_put_xyz_quat(const char* title, const q_xyz_quat_type *val)
{
    char *buf = NULL;
    q_vec_type yawPitchRoll;

    q_to_euler(yawPitchRoll, val->quat); // quaternion to euler for display

    asprintf(&buf, "%8s=[%8.4f, %8.4f, %8.4f], euler=[Yaw/Z=%9.4f', Pitch/Y=%9.4f', Roll/X=%9.4f']",
        title, val->xyz[0], val->xyz[1], val->xyz[2],
        Q_RAD_TO_DEG(yawPitchRoll[0]),
        Q_RAD_TO_DEG(yawPitchRoll[1]),
        Q_RAD_TO_DEG(yawPitchRoll[2]));
    console_put(buf);

    free(buf);
}

static int64_t utils_timeval_diff_us(struct timeval *B, struct timeval *A)
{
    int64_t r;

    r = B->tv_sec - A->tv_sec;
    r *= 1000000LL;
    r += B->tv_usec - A->tv_usec;

    return r;
};

static struct timeval timestamp5 = { 0, 0 };

void vrpn_Server_OpenVR::mainloop() {
    char *buf = NULL, press;
    int ref_tracker_idx = -1;
    struct timeval timestamp;
    struct timeval timestamp2, timestamp3, timestamp4;

    press = console_keypress(console_in);
    if (press >= '0' && press <= '9')
        ref_tracker_idx = press - '0';

    // Get Tracking Information
    vrpn_gettimeofday(&timestamp, NULL);
    vr::TrackedDevicePose_t m_rTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
    vr->GetDeviceToAbsoluteTrackingPose(    /// https://github.com/ValveSoftware/openvr/wiki/IVRSystem::GetDeviceToAbsoluteTrackingPose
        vr::TrackingUniverseStanding,
        0 /*float fPredictedSecondsToPhotonsFromNow*/,
        m_rTrackedDevicePose,
        vr::k_unMaxTrackedDeviceCount
    );
    vrpn_gettimeofday(&timestamp2, NULL);

    // show built info
    buf = NULL;
    asprintf(&buf, "VRPN/FREE-D for StreamVR. api %s, app built [" __DATE__ " " __TIME__ "]", vr->GetRuntimeVersion());
    console_put(buf);
    if (buf) free(buf);
    console_put("");

    for (vr::TrackedDeviceIndex_t unTrackedDevice = 0; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; unTrackedDevice++) {
        const char* state = "Running_OK";
        int f_update_data = 1;
        vr::TrackedDevicePose_t* pose = &m_rTrackedDevicePose[unTrackedDevice];

        if (!pose->bDeviceIsConnected)
            continue;

        if (!pose->bPoseIsValid) {
            state = " !bPoseIsValid";
            f_update_data = 0;
        }

        if (pose->eTrackingResult != vr::TrackingResult_Running_OK)
        {
            state =
                vr::TrackingResult_Uninitialized == pose->eTrackingResult ? "Uninitialized" :
                vr::TrackingResult_Calibrating_InProgress == pose->eTrackingResult ? "Calibrating In Progress" :
                vr::TrackingResult_Calibrating_OutOfRange == pose->eTrackingResult ? "Calibrating Out Of Range" :
                vr::TrackingResult_Running_OutOfRange == pose->eTrackingResult ? "Running Out Of Range" :
                "Unknown";
            f_update_data = 0;
        }


        // get device class
        vr::ETrackedDeviceClass device_class_id = vr->GetTrackedDeviceClass(unTrackedDevice);
        const std::string device_class_name = getDeviceClassName(device_class_id);

        // find serial
        std::string device_serial = getDeviceSerial(unTrackedDevice, vr.get());

        // build name
        const std::string device_name = "openvr/" + device_class_name + "/" + (device_serial == "" ? std::to_string(unTrackedDevice) : device_serial);

        /* output name */
        buf = NULL;  asprintf(&buf, "[%2d] => %-40s | %-40s", unTrackedDevice, device_name.c_str(), state);
        console_put(buf);
        if (buf) free(buf);

        /* create new device or get early added */
        vrpn_Tracker_OpenVR *dev{ nullptr };
        auto dev_srch = devices.find(unTrackedDevice);
        if (dev_srch == devices.end())
        {
            std::unique_ptr<vrpn_Tracker_OpenVR> newDEV;

            switch (device_class_id)
            {
                case vr::TrackedDeviceClass_GenericTracker:     /// https://github.com/ValveSoftware/openvr/wiki/IVRSystem_Overview
                case vr::TrackedDeviceClass_TrackingReference:
                case vr::TrackedDeviceClass_HMD:
                    newDEV = std::make_unique<vrpn_Tracker_OpenVR_HMD>(device_name, connection, vr.get(), unTrackedDevice);
                    break;

                case vr::TrackedDeviceClass_Controller:
                    newDEV = std::make_unique<vrpn_Tracker_OpenVR_Controller>(device_name, connection, vr.get(), unTrackedDevice);
                    break;

                default:
                    newDEV = std::make_unique<vrpn_Tracker_OpenVR>(device_name, connection, vr.get(), unTrackedDevice);
            }

            dev = newDEV.get();
            devices[unTrackedDevice] = std::move(newDEV);
        }
        else
            dev = dev_srch->second.get();

        /* update tracking data */
        if (f_update_data)
        {
            dev->updateTracking(pose);
            dev->mainloop();
        };

        /* display position and rot */
        q_xyz_quat_type mnt;
        dev->getPose(&mnt);
        console_put_xyz_quat("pos", &mnt);

        /* find camera assiciated with that tracker */
        for (const auto& ci : cameras)
        {
            /* check for serial */
            if (ci->getTrackerSerial() != device_serial)
                continue;

            /* do some precomputation */
            ci->updateTracking(&stk, &mnt, &ref, &timestamp);
            ci->mainloop();
        }

        /* empty line */
        console_put("");

        /* save tracker data as reference position */
        if (ref_tracker_idx == unTrackedDevice)
            stk = mnt;
    }

    console_put("Virtual space:");
    console_put("");

    {
        /* display reference point */
        console_put_xyz_quat("ref", &ref);

        /* display reference tracker position and rot */
        console_put_xyz_quat("stk", &stk);

        /* empty line */
        console_put("");
    }

    console_put("Virtual cameras:");
    console_put("");

    /* dump all cameras state */
    for (const auto& ci : cameras)
    {
        /* output name */
        buf = NULL;  asprintf(&buf, "        %-40s | %-40s", ci->getName().c_str(), ci->getTrackerSerial().c_str());
        console_put(buf);
        if (buf) free(buf);

        /* display position and rot */
        q_xyz_quat_type pos;
        ci->getPose(&pos);
        console_put_xyz_quat("pos", &pos);

        /* empty line */
        console_put("");
    }

    /* empty line */
    console_put("");

    vrpn_gettimeofday(&timestamp3, NULL);

    // Send and receive all messages.
    connection->mainloop();

    // Bail if the connection is in trouble.
    if (!connection->doing_okay()) {
        std::cerr << "Connection is not doing ok. Should we bail?" << std::endl;
    }

    vrpn_gettimeofday(&timestamp4, NULL);

    // show some timings
    buf = NULL;
    asprintf(&buf, "stat: %8lld us, %8lld us, %8lld us, %8lld us",
        utils_timeval_diff_us(&timestamp2, &timestamp),
        utils_timeval_diff_us(&timestamp3, &timestamp),
        utils_timeval_diff_us(&timestamp4, &timestamp),
        utils_timeval_diff_us(&timestamp, &timestamp5)
    );
    timestamp5 = timestamp;
    console_put(buf);
    if (buf) free(buf);

    /* empty line */
    console_put("");

    // setup cusrsor to top
    console_swap_fb();
}

const std::string vrpn_Server_OpenVR::getDeviceClassName(vr::ETrackedDeviceClass device_class_id)
{
    return
        vr::TrackedDeviceClass_HMD == device_class_id ? "HMD" :
        vr::TrackedDeviceClass_Controller == device_class_id ? "Controller" :
        vr::TrackedDeviceClass_GenericTracker == device_class_id ? "GenericTracker" :
        vr::TrackedDeviceClass_TrackingReference == device_class_id ? "TrackingReference" :
        vr::TrackedDeviceClass_DisplayRedirect == device_class_id ? "DisplayRedirect" :
        "Invalid";
}

const std::string vrpn_Server_OpenVR::getDeviceSerial(vr::TrackedDeviceIndex_t trackedDeviceIndex, vr::IVRSystem * vr)
{
    std::string device_serial = "";
    {
        /// https://steamcommunity.com/app/358720/discussions/0/1353742967802223832/
        unsigned int unRequiredBufferLen = 128;
        char* pchBuffer = new char[unRequiredBufferLen];
        unRequiredBufferLen = vr->GetStringTrackedDeviceProperty(trackedDeviceIndex, vr::Prop_SerialNumber_String, pchBuffer, unRequiredBufferLen, nullptr);
        device_serial = pchBuffer;
        delete[] pchBuffer;
    };
    return device_serial;
}

HANDLE console_in, console_out;


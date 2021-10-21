#include "vrpn_Tracker_Camera.h"
#include <openvr.h>
#include <quat.h>
#include <iostream>

#include "FreeD.h"
#include "wt.h"

void vrpn_Tracker_Camera::filterAdd(filter_abstract* flt)
{
    filters_list[filters_cnt] = flt;
    filters_cnt++;
};

vrpn_Tracker_Camera::vrpn_Tracker_Camera(int idx, const std::string& name, vrpn_Connection* connection, const std::string& tracker_serial, const q_xyz_quat_type *_arm) :
	vrpn_Tracker(name.c_str(), connection), name(name), tracker_serial(tracker_serial), freed_socket(-1), idx(idx)
{
    arm = *_arm;
    filters_cnt = 0;
    // Initialize the vrpn_Tracker
    // We track each device separately so this will only ever have one sensor
    vrpn_Tracker::num_sensors = 1;
}

std::string vrpn_Tracker_Camera::getName()
{
    return name;
}

std::string vrpn_Tracker_Camera::getTrackerSerial()
{
    return tracker_serial;
}

void vrpn_Tracker_Camera::mainloop() {
//    vrpn_gettimeofday( &(vrpn_Tracker_Camera::timestamp), NULL );
	vrpn_Tracker::server_mainloop();
}

void vrpn_Tracker_Camera::freedAdd(char *host_port)
{
    char *port, *host = strdup(host_port);

    port = strrchr(host, ':');
    if (port)
    {
        struct sockaddr_in addr;

        *port = 0; port++;

        /* prepare address */
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr(host);
        addr.sin_port = htons((unsigned short)atoi(port));

        /* store address */
        freed_sockaddr.push_back(addr);
    }

    free(host);
}

void vrpn_Tracker_Camera::freedSend()
{
    FreeD_D1_t freed;
    unsigned char buf[FREE_D_D1_PACKET_SIZE];

    /* init socket */
    if (freed_socket <= 0)
        freed_socket = socket(AF_INET, SOCK_DGRAM, 0);

    if (freed_socket <= 0)
        return;

    memset(&freed, 0, sizeof(freed));

    freed.ID = idx + 1;

    q_xyz_quat_type pose;
    getPose(&pose);
    freed.X = pose.xyz[1] * 1000.0;
    freed.Y = pose.xyz[0] * 1000.0;
    freed.Z = pose.xyz[2] * 1000.0;

    q_vec_type yawPitchRoll;
    pose.quat[2] *= -1.0;
    q_to_euler(yawPitchRoll, pose.quat);
    freed.Pan  = yawPitchRoll[0] * 180.0 / 3.1415926;
    freed.Tilt = yawPitchRoll[1] * 180.0 / 3.1415926;
    freed.Roll = yawPitchRoll[2] * 180.0 / 3.1415926;

    FreeD_D1_pack(buf, FREE_D_D1_PACKET_SIZE, &freed);

    for (const struct sockaddr_in /*auto&*/ addr : freed_sockaddr)
    {
        sendto
        (
            freed_socket,               /* Socket to send result */
            (char*)buf,                 /* The datagram buffer */
            sizeof(buf),                /* The datagram lngth */
            0,                          /* Flags: no options */
            (struct sockaddr *)&addr,   /* addr */
            sizeof(struct sockaddr_in)  /* Server address length */
        );
    }
}

void vrpn_Tracker_Camera::updateTracking(const q_xyz_quat_type *_stk, const q_xyz_quat_type *_mnt, const q_xyz_quat_type *ref, struct timeval *tv)
{
    int f;
    q_xyz_quat_type stk, mnt, trk, cam;

    wt_openvr_to_ue(_stk, &stk);
    wt_openvr_to_ue(_mnt, &mnt);

    for (f = 0; f < filters_cnt; f++)
        filters_list[f]->process_data(&mnt);

    wt_calc_two_trackers_model_a(&stk, &mnt, &arm, ref, &trk, &cam);

    d_quat[0] = cam.quat[0];
    d_quat[1] = cam.quat[1];
    d_quat[2] = cam.quat[2];
    d_quat[3] = cam.quat[3];

    pos[0] = cam.xyz[0];
    pos[1] = cam.xyz[1];
    pos[2] = cam.xyz[2];

    // Pack message
    timestamp.tv_sec = tv->tv_sec;
    timestamp.tv_usec = tv->tv_usec;

    char msgbuf[1000];
    vrpn_int32 len = vrpn_Tracker::encode_to(msgbuf);
    if (d_connection->pack_message(len, timestamp, position_m_id, d_sender_id, msgbuf, vrpn_CONNECTION_LOW_LATENCY)) {
        std::cerr << " Can't write message";
    }
}

void vrpn_Tracker_Camera::getPose(q_xyz_quat_type *pose)
{
    pose->quat[0] = d_quat[0];
    pose->quat[1] = d_quat[1];
    pose->quat[2] = d_quat[2];
    pose->quat[3] = d_quat[3];

    pose->xyz[0] = pos[0];
    pose->xyz[1] = pos[1];
    pose->xyz[2] = pos[2];
}

#pragma once

#include <list>
#include <string>
#include <vrpn_Tracker.h>
#include <quat.h>

#include "filter.h"

class vrpn_Tracker_Camera :
    public vrpn_Tracker
{
public:
    vrpn_Tracker_Camera(int idx, const std::string& name, vrpn_Connection* connection, const std::string& tracker_serial, const q_xyz_quat_type *_arm);
    void mainloop();
    void updateTracking(const q_xyz_quat_type *_stk, const q_xyz_quat_type *_mnt, const q_xyz_quat_type *ref, struct timeval *tv);
    void getPose(q_xyz_quat_type *pose);
    std::string getName();
    std::string getTrackerSerial();
    void freedAdd(char *host_port);
    void filterAdd(filter_abstract* flt);
protected:
    void freedSend();

private:
    q_xyz_quat_type arm;
    std::string name;
    std::string tracker_serial;
    std::list<std::string> freed_targets;
    std::list<struct sockaddr_in> freed_sockaddr;
    int freed_socket;
    int idx;

    filter_abstract* filters_list[16];
    int filters_cnt;
};


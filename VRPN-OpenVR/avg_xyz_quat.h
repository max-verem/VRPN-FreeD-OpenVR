#ifndef avg_xyz_quat_h
#define avg_xyz_quat_h

#include <vector>
#include <quat.h>

void avg_xyz_quat_v0(std::vector<q_xyz_quat_type> &src, q_xyz_quat_type *dst);
void avg_xyz_quat_v1(std::vector<q_xyz_quat_type> &src, q_xyz_quat_type *dst);

#endif

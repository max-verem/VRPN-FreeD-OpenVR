#ifndef WT_H
#define WT_H

#include <quat.h>

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

void wt_cv_to_ue(const q_xyz_quat_type* cv, q_xyz_quat_type *ue);
void wt_vertical_aruco_to_ue(const q_xyz_quat_type* aruco, q_xyz_quat_type *ue);
void wt_rs2_to_ue(const q_xyz_quat_type* rs2, q_xyz_quat_type *ue);
void wt_rot_mask(const q_xyz_quat_type* src, double yaw, double pitch, double roll, q_xyz_quat_type* dst);
void wt_A_to_B(const q_xyz_quat_type* A_ref, const q_xyz_quat_type* B_ref,
    const q_xyz_quat_type* A_curr, q_xyz_quat_type* B_curr);
void wt_calc_two_trackers_model_a
(
    const q_xyz_quat_type *stk,  /* rotation/position of static tracker */
    const q_xyz_quat_type *mnt,  /* rotation/position of motion tracker */
    const q_xyz_quat_type *arm,  /* rotation of mounted tracker and arm vector*/
    const q_xyz_quat_type *ref,  /* reference position, static tracker real position */
    q_xyz_quat_type *trk,       /* montion tracker recalced */
    q_xyz_quat_type *cam        /* real camera optic center rotation and position */
);
void wt_calc_two_trackers_model_b
(
    const q_xyz_quat_type *stk,  /* rotation/position of static tracker */
    const q_xyz_quat_type *mnt,  /* rotation/position of motion tracker */
    const q_xyz_quat_type *arm,  /* rotation of mounted tracker and arm vector*/
    const q_xyz_quat_type *ref,  /* reference position, static tracker real position */
    q_xyz_quat_type *trk,       /* montion tracker recalced */
    q_xyz_quat_type *cam        /* real camera optic center rotation and position */
);
void wt_openvr_to_ue(const q_xyz_quat_type* openvr, q_xyz_quat_type *ue);

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif

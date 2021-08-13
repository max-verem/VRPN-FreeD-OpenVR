#include "wt.h"

void wt_cv_to_ue(const q_xyz_quat_type* cv, q_xyz_quat_type *ue)
{
    // translate CV quat to UE axis style
    ue->quat[0] = cv->quat[2];
    ue->quat[1] = cv->quat[0];
    ue->quat[2] = -cv->quat[1];
    ue->quat[3] = cv->quat[3];

    // translate CV pos to UE axis style
    ue->xyz[0] = cv->xyz[2];
    ue->xyz[1] = cv->xyz[0];
    ue->xyz[2] = -cv->xyz[1];
};

void wt_vertical_aruco_to_ue(const q_xyz_quat_type* aruco, q_xyz_quat_type *ue)
{
    q_type a, r = { 1, 0, 0, 0 };;

    // rotate
    q_mult(a, aruco->quat, r);

    // translate CV quat to UE axis style
    ue->quat[0] = -a[2];
    ue->quat[1] =  a[0];
    ue->quat[2] =  a[1];
    ue->quat[3] =  a[3];

    // translate CV pos to UE axis style
    ue->xyz[0] = -aruco->xyz[2];
    ue->xyz[1] =  aruco->xyz[0];
    ue->xyz[2] =  aruco->xyz[1];
};

void wt_rs2_to_ue(const q_xyz_quat_type* rs2, q_xyz_quat_type *ue)
{
    /*
        RealSense World
        https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md

        UE4 world:

            Unreal uses a right-handed, Z-up coordinate system.
            +x - forward
            +y - right
            +z - up

    */

    // pos
    ue->xyz[0] = -rs2->xyz[2];
    ue->xyz[1] =  rs2->xyz[0];
    ue->xyz[2] =  rs2->xyz[1];

    // quat
    ue->quat[0] = -rs2->quat[2];
    ue->quat[1] =  rs2->quat[0];
    ue->quat[2] =  rs2->quat[1];
    ue->quat[3] =  rs2->quat[3];
};

void wt_rot_mask(const q_xyz_quat_type* src, double yaw, double pitch, double roll, q_xyz_quat_type* dst)
{
    q_vec_type yawPitchRoll;

    q_vec_copy(dst->xyz, src->xyz);

    q_to_euler(yawPitchRoll, src->quat);

    q_from_euler(dst->quat, yawPitchRoll[0] * yaw, yawPitchRoll[1] * pitch, yawPitchRoll[2] * roll);
};

void wt_A_to_B(const q_xyz_quat_type* A_ref, const q_xyz_quat_type* B_ref,
    const q_xyz_quat_type* A_curr, q_xyz_quat_type* B_curr)
{
    q_type q;
    q_vec_type p;

    // target rotation
    q_invert(q, A_ref->quat);
    q_mult(q, A_curr->quat, q);
    q_mult(B_curr->quat, q, B_ref->quat);

    // target position
    q_invert(q, B_ref->quat);
    q_mult(q, A_ref->quat, q);

    q_vec_subtract(p, A_curr->xyz, A_ref->xyz);
    q_xform(p, q, p);
    q_vec_add(B_curr->xyz, B_ref->xyz, p);
};

void wt_calc_two_trackers_model_a
(
    const q_xyz_quat_type *stk,  /* rotation/position of static tracker */
    const q_xyz_quat_type *mnt,  /* rotation/position of motion tracker */
    const q_xyz_quat_type *arm,  /* rotation of mounted tracker and arm vector*/
    const q_xyz_quat_type *ref,  /* reference position, static tracker real position */
    q_xyz_quat_type *trk,       /* montion tracker recalced */
    q_xyz_quat_type *cam        /* real camera optic center rotation and position */
)
{
    q_xyz_quat_type stk_z, arm_r;

    wt_rot_mask(stk, 1.0, 0.0, 0.0, &stk_z);

    // calc tracker
    wt_A_to_B
    (
        &stk_z,  // static tracker
        ref,
        mnt,    // mounted tracker
        trk
    );

    // do prerot of motion tracker
    q_mult(cam->quat, arm->quat, trk->quat);

    // arm vec rotation
    q_invert(arm_r.quat, trk->quat);
    q_xform(arm_r.xyz, arm_r.quat, arm->xyz);
    q_vec_add(cam->xyz, trk->xyz, arm_r.xyz);
};

void wt_calc_two_trackers_model_b
(
    const q_xyz_quat_type *stk,  /* rotation/position of static tracker */
    const q_xyz_quat_type *mnt,  /* rotation/position of motion tracker */
    const q_xyz_quat_type *arm,  /* rotation of mounted tracker and arm vector*/
    const q_xyz_quat_type *ref,  /* reference position, static tracker real position */
    q_xyz_quat_type *trk,       /* montion tracker recalced */
    q_xyz_quat_type *cam        /* real camera optic center rotation and position */
)
{
    q_xyz_quat_type stk_z, arm_r, ref_z;

    wt_rot_mask(stk, 1.0, 0.0, 0.0, &stk_z);
    wt_rot_mask(ref, 1.0, 0.0, 0.0, &ref_z);

    // calc tracker
    wt_A_to_B
    (
        &stk_z,  // static tracker
        &ref_z,
        mnt,    // mounted tracker
        trk
    );

    // do prerot of motion tracker
    q_mult(cam->quat, arm->quat, trk->quat);

    // arm vec rotation
    q_invert(arm_r.quat, trk->quat);
    q_xform(arm_r.xyz, arm_r.quat, arm->xyz);
    q_vec_add(cam->xyz, trk->xyz, arm_r.xyz);
};

/*
    OpenVR world:
        right-handed system
        +y is up
        +x is to the right
        -z is forward
        Distance unit is  meters
    UE4 world:
        Unreal uses a right-handed, Z-up coordinate system.
        +x - forward
        +y - right
        +z - up
*/
void wt_openvr_to_ue(const q_xyz_quat_type* openvr, q_xyz_quat_type *ue)
{
    ue->quat[0] = -openvr->quat[2];
    ue->quat[1] = openvr->quat[0];
    ue->quat[2] = openvr->quat[1];
    ue->quat[3] = openvr->quat[3];

    ue->xyz[0] = -openvr->xyz[2];
    ue->xyz[1] = openvr->xyz[0];
    ue->xyz[2] = openvr->xyz[1];
}

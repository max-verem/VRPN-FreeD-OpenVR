#include "avg_xyz_quat.h"

void avg_xyz_quat_v0(std::vector<q_xyz_quat_type> &src, q_xyz_quat_type *dst)
{
    int a;
    double w = 1.0 / src.size();
    q_type q_ident = Q_ID_QUAT;

    memset(dst, 0, sizeof(q_xyz_quat_type));
    q_copy(dst->quat, q_ident);

    for (a = 0; a < src.size(); a++)
    {
        q_type q;

        q_slerp(q, q_ident, src[a].quat, w);
        q_mult(dst->quat, dst->quat, q);

        q_vec_add(dst->xyz, dst->xyz, src[a].xyz);
    }
    q_vec_scale(dst->xyz, w, dst->xyz);
}

void avg_xyz_quat_v1(std::vector<q_xyz_quat_type> &src, q_xyz_quat_type *dst)
{
    int a;
    double w = 1.0 / src.size();
    q_type q_ident = Q_ID_QUAT;

    memset(dst, 0, sizeof(q_xyz_quat_type));
    q_copy(dst->quat, q_ident);

    for (a = 0; a < src.size(); a++)
    {
        q_slerp(dst->quat, dst->quat, src[a].quat, 1.0 / (a + 1));

        q_vec_add(dst->xyz, dst->xyz, src[a].xyz);
    }
    q_vec_scale(dst->xyz, w, dst->xyz);
}

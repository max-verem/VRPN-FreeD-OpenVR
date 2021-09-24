#include "filter.h"
#include <math.h>
#include "avg_xyz_quat.h"

filter_median::filter_median(int sz)
{
    window = sz;
    samples = 0;
}

void filter_median::process_data(q_xyz_quat_type *pose)
{

}

filter_avg::filter_avg(int sz)
{
    if (sz > FILTER_MEDIAN_MAX_WINDOW)
        sz = FILTER_MEDIAN_MAX_WINDOW;
    window = sz;
    samples = 0;
}

void filter_avg::process_data(q_xyz_quat_type *pose)
{
    q_xyz_quat_type tmp;
    std::vector<q_xyz_quat_type> src;

    poses[samples % window] = *pose;
    samples++;

    if (samples < window)
        return;

    for (int i = 0; i < window; i++)
        src.push_back(poses[i]);

    avg_xyz_quat_v1(src, &tmp);

    *pose = tmp;
}

filter_exp1::filter_exp1(double a_pos, double a_rot)
{
    alpha_pos = a_pos;
    alpha_rot = a_rot;
    samples = 0;
}

void filter_exp1::process_data(q_xyz_quat_type *pose)
{
    int i;
    q_xyz_quat_type tmp;

    if (samples)
    {
        q_type A, B, I = Q_ID_QUAT;

        for (i = 0; i < 3; i++)
            tmp.xyz[i] = alpha_pos * pose->xyz[i] + (1.0 - alpha_pos) * pose_prev.xyz[i];

        q_slerp(A, I, pose->quat, alpha_pos);
        q_slerp(B, I, pose_prev.quat, 1.0 - alpha_pos);
        q_mult(tmp.quat, A, B);

        *pose = tmp;
        pose_prev = tmp;
    }
    else
        pose_prev = *pose;

    samples++;
}

filter_kalman::filter_kalman(double _pos_E_est, double _pos_E_mea)
{
#if 0
    pos_E_est[0] = pos_E_est[1] = pos_E_est[2] = _pos_E_est;
    pos_E_mea[0] = pos_E_mea[1] = pos_E_mea[2] = _pos_E_mea;
    samples = 0;
#endif
}

void filter_kalman::process_data(q_xyz_quat_type *pose)
{
#if 0
    int i;
    q_vec_type KG, pos_EST;

    if (samples == 0)
    {
        samples++;
        q_vec_copy(pos_EST_prev, pos);
        return;
    }

    for (i = 0; i < 3; i++)
    {
        KG[i] = pos_E_est[i] / (pos_E_est[i] + pos_E_mea[i]);

        pos_EST[i] = pos_EST_prev[i] + KG[i] * (pos[i] - pos_EST_prev[i]);

        pos_E_est[i] = (1 - KG[i]) * pos_E_est[i];
    };

    q_vec_copy(pos, pos_EST);
    q_vec_copy(pos_EST_prev, pos_EST);
#endif
}

filter_exp1dyn::filter_exp1dyn(double a_pos, double d_pos)
{
    k = -log(1.0 - a_pos) / d_pos;
    samples = 0;
}

void filter_exp1dyn::process_data(q_xyz_quat_type *pose)
{

    if (!samples)
        pose_prev = *pose;
    else
    {
        int i;
        double d, alpha_pos;
        q_xyz_quat_type pose_tmp;

        for (d = 0.0, i = 0; i < 3; i++)
            d += (pose->xyz[i] - pose_prev.xyz[i]) * (pose->xyz[i] - pose_prev.xyz[i]);
        d = sqrt(d);

        alpha_pos = (1 - exp(-k * d));

        for (i = 0; i < 3; i++)
            pose_tmp.xyz[i] = alpha_pos * pose->xyz[i] + (1.0 - alpha_pos) * pose_prev.xyz[i];

        q_vec_copy(pose_prev.xyz, pose_tmp.xyz);
        q_vec_copy(pose->xyz, pose_tmp.xyz);
    }

    samples++;
}

filter_exp1pasha::filter_exp1pasha(double a, double b)
{
    alpha.xyz[0] = alpha.xyz[1] = alpha.xyz[2] = a;
    betta = b;
    samples = 0;
}

void filter_exp1pasha::process_data(q_xyz_quat_type *pose)
{
    if (!samples)
        pose_prev = *pose;
    else
    {
        int i;
        q_vec_type pos_tmp;

        for (i = 0; i < 3; i++)
        {
            double t;

            pos_tmp[i] = alpha.xyz[i] * pose->xyz[i] + (1.0 - alpha.xyz[i]) * pose_prev.xyz[i];

            t = fabs(pose->xyz[i] - pos_tmp[i]);

            alpha.xyz[i] = betta * t + (1 - betta) * alpha.xyz[i];
        };

        q_vec_copy(pose->xyz, pos_tmp);
        q_vec_copy(pose_prev.xyz, pos_tmp);
    };

    samples++;
}


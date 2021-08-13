#pragma once

#include <quat.h>

class filter_abstract
{
    public:
        filter_abstract() {};
        virtual void process_data(q_xyz_quat_type *pose) = 0;
};

#define FILTER_MEDIAN_MAX_WINDOW 1024

class filter_median : public filter_abstract
{
public:
    filter_median(int win_size);
    virtual void process_data(q_xyz_quat_type *pose);

private:
    q_xyz_quat_type poses[FILTER_MEDIAN_MAX_WINDOW];
    int window, samples;
};

#define FILTER_AVG_MAX_WINDOW 1024

class filter_avg : public filter_abstract
{
public:
    filter_avg(int win_size);
    virtual void process_data(q_xyz_quat_type *pose);

private:
    q_xyz_quat_type poses[FILTER_MEDIAN_MAX_WINDOW];
    long long window, samples;
};

class filter_exp1 : public filter_abstract
{
public:
    filter_exp1(double a_pos, double a_rot);
    virtual void process_data(q_xyz_quat_type *pose);
private:
    int samples;
    double alpha_pos, alpha_rot;
    q_xyz_quat_type pose_prev;
};

class filter_kalman : public filter_abstract
{
public:
    filter_kalman(double _pos_E_est, double _pos_E_mea);
    virtual void process_data(q_xyz_quat_type *pose);
private:
    int samples;
    q_xyz_quat_type pose_E_est, pose_E_mea, pose_EST_prev;
};

class filter_exp1dyn : public filter_abstract
{
public:
    filter_exp1dyn(double a_pos, double d_pos);
    virtual void process_data(q_xyz_quat_type *pose);
private:
    int samples;
    double k;
    q_xyz_quat_type pose_prev;
};

class filter_exp1pasha : public filter_abstract
{
public:
    filter_exp1pasha(double a_pos, double d_pos);
    virtual void process_data(q_xyz_quat_type *pose);
private:
    int samples;
    double betta;
    q_xyz_quat_type pose_prev, alpha;
};


#include "trajectory.h"
#include <cmath>

trajectory::trajectory()
{
    squat_T_pause = 1.5;
    freq_squat = 1;
    squat_r0 = 0.3536;
    squat_rc = 0.2;

    K_thrust = 500;
    zeta_thrust = 0;
    K_land = 1000;
    zeta_land = 1;

    jump_r0 = 0.3526;
    jump_rc = 0.2;
    jump_rt = 0.4;

    jump_T_stand = 2;
    jump_T_crouch = 1;
    jump_T_pause = 0.5;
    jump_T_land = 1;
    jump_T_recover = 1;
    jump_qd_max = 20;
};

trajectory::~trajectory(){};

void trajectory::Squat(double t,StateModel_* state_model)
{
    double deltaR = squat_r0 - squat_rc;

    double T_squat = 1 / freq_squat;
    double T_period = T_squat + squat_T_pause;

    double t_norm = t - T_period * floor(t / T_period); // nominalized time
    double t1 = squat_T_pause;
    double t2 = t1 + T_squat;

    if (0 <= t_norm && t_norm < t1)
    {
        state_model->posRW_ref[0] = squat_r0;
        state_model->posRW_ref[1] = pi / 2;
    }
    else if (t1 <= t_norm && t_norm < t2)
    {
        state_model->posRW_ref[0] = squat_r0 - 0.5 * deltaR * (1 - cos(2 * pi * freq_squat * (t_norm - t1)));
        state_model->posRW_ref[1] = pi / 2;
        state_model->velRW_ref[0] = -deltaR * (pi * freq_squat) * sin(2 * pi * freq_squat * (t_norm - t1));
    }
    state_model->forRW_ref[0] = 0.1*sin(2*pi*t);
    state_model->forRW_ref[1] = 0.1*sin(2*pi*t);
};

//Jumaping need debugging. it need admittance parameter
void trajectory::Jumping(double t, StateModel_* state_model, int mode_admitt)
{
    // double L = 0.25;
    // double deltaR = jump_rt - jump_rc;

    // double vel_max = (2 * L * pow(jump_qd_max, 2) * pow(sin(acos(jump_rt / (2 * L))), 2)) / g;

    // double T_thrust = 3 * deltaR / vel_max;
    // double T_peak = vel_max / g;    
    // double T_flight = 0.7;

    // double t0 = jump_T_stand;          // crouching timing
    // double t1 = t0 + jump_T_crouch;    // pause timing
    // double t2 = t1 + jump_T_pause;     // thrust timing
    // double t3 = t2 + T_thrust;          // flight timing
    // double t4 = t3 + T_flight;          // (Expected) landing timing
    // double t5 = t4 + jump_T_land;      // (Expected) recovery timing
    // double t6 = t5 + jump_T_recover;   // (Expected) stance timing
    // double T_period = t6;               // whole jumping period

    // double t_norm = t - T_period * floor(t / T_period); // nominalized time

    // if (mode_admitt == 1)
    // {
    //     param_tuning->Ma_new = param_tuning->Ma;
    //     param_tuning->Ka_new = param_tuning->K_thrust * param_tuning->Ka;
    //     param_tuning->Ba_new = 2 * param_tuning->zeta_thrust * sqrt(param_tuning->Ma_new * param_tuning->Ka_new);
    // }

    // if (t < t0)    // stance phase
    // {
    //     state_model->posRW_ref[0] = jump->r0;
    //     state_model->posRW_ref[1] = pi / 2.;
    //     state_model->velRW_ref[0] = .0;
    //     state_model->velRW_ref[1] = .0;

    //     if (mode_admitt == 1)
    //     {
    //         param_tuning->Ma_new = param_tuning->Ma;
    //         param_tuning->Ka_new = param_tuning->Ka;
    //         param_tuning->Ba_new = param_tuning->Ba;
    //     }
    // }
    // else if (t0 <= t && t < t1)   // crouch phase
    // {
    //     state_model->posRW_ref[0] = 0.5 * (jump->r0 + jump->rc) + 0.5 * (jump->r0 - jump->rc) * cos((pi / jump->T_crouch) * (t_norm - t0));
    //     state_model->posRW_ref[1] = pi / 2.;
    //     state_model->velRW_ref[0] = -0.5 * (pi / jump->T_crouch) * (jump->r0 - jump->rc) * sin((pi / jump->T_crouch) * (t_norm - t0));
    //     state_model->velRW_ref[1] = .0;

    //     if (mode_admitt == 1)
    //     {
    //         param_tuning->Ma_new = param_tuning->Ma;
    //         param_tuning->Ka_new = param_tuning->Ka;
    //         param_tuning->Ba_new = param_tuning->Ba;
    //     }
    // }
    // else if (t1 <= t && t < t2)   // pause phase
    // {
    //     state_model->posRW_ref[0] = jump->rc;
    //     state_model->posRW_ref[1] = pi / 2.;
    //     state_model->velRW_ref[0] = .0;
    //     state_model->velRW_ref[1] = .0;

    //     if (mode_admitt == 1)
    //     {
    //         param_tuning->Ma_new = param_tuning->Ma;
    //         param_tuning->Ka_new = param_tuning->K_thrust* param_tuning->Ka;
    //         param_tuning->Ba_new = 2 * param_tuning->zeta_thrust * sqrt(param_tuning->Ma_new * param_tuning->Ka_new);
    //     }
    // }
    // else if (t2 <= t && t < t3)   // thrust phase
    // {
    //     double a0 = jump->rc;
    //     double a1 = .0;
    //     double a2 = .0;
    //     double a3 = deltaR / (T_thrust * T_thrust * T_thrust);

    //     state_model->posRW_ref[0] = a0 + a1 * (t_norm - t2) + a2 * pow(t_norm - t2, 2) + a3 * (t_norm - t2) * pow(t_norm - t2, 2);
    //     state_model->posRW_ref[1] = pi / 2.;
    //     state_model->velRW_ref[0] = a1 + 2 * a2 * (t_norm - t2) + 3 * a3 * pow(t_norm - t2, 2);

    //     if (mode_admitt == 1)
    //     {
    //         param_tuning->Ma_new = param_tuning->Ma;
    //         param_tuning->Ka_new = param_tuning->K_thrust* param_tuning->Ka;
    //         param_tuning->Ba_new = 2 * param_tuning->zeta_thrust * sqrt(param_tuning->Ma_new * param_tuning->Ka_new);
    //     }
    // }
    // else if (t3 <= t && t < 3.7)    // flight phase
    // {
    //     state_model->posRW_ref[0] = jump->rt;
    //     state_model->posRW_ref[1] = pi / 2.;
    //     state_model->velRW_ref[0] = .0;
    //     state_model->velRW_ref[1] = .0;

    //     if (mode_admitt == 1)
    //     {
    //         param_tuning->Ma_new = param_tuning->Ma;
    //         param_tuning->Ka_new = param_tuning->K_land * param_tuning->Ka;
    //         param_tuning->Ba_new = 2 * param_tuning->zeta_land * sqrt(param_tuning->Ma_new * param_tuning->Ka_new);
    //     }
    // }
    // else if (3.7 <= t && t < 8)   // landing phase
    // {
    //     state_model->posRW_ref[0] = jump->rt;//0.5 * (jump->rc + jump->rt) + 0.5 * (jump->rt - jump->rc) * cos((pi / jump->T_land) * (t_norm - t4));
    //     state_model->posRW_ref[1] = pi / 2.;
    //     state_model->velRW_ref[0] = -0.5 * (pi / jump->T_land) * (jump->rt - jump->rc) * sin((pi / jump->T_land) * (t_norm - t4));
    //     state_model->velRW_ref[1] = .0;

    //     if (mode_admitt == 1)
    //     {
    //         param_tuning->Ma_new = param_tuning->Ma;
    //         param_tuning->Ka_new = 1500; //critically damper system �� �� Ka= 15000 �� �� �׳��� ���� �׸��� �׶� ���� 382
    //         param_tuning->Ba_new = 2 * 0.95 * sqrt(param_tuning->Ma_new * param_tuning->Ka_new);
    //     }
    // }
    // else if (8 <= t && t < t6)   // recovery phase
    // {
    //     state_model->posRW_ref[0] = 0.5 * (jump->r0 + jump->rc) + 0.5 * (jump->rc - jump->r0) * cos((pi / jump->T_recover) * (t_norm - t5));
    //     state_model->posRW_ref[1] = pi / 2.;
    //     state_model->velRW_ref[0] = -0.5 * (pi / jump->T_recover) * (jump->rc - jump->r0) * sin((pi / jump->T_recover) * (t_norm - t5));
    //     state_model->velRW_ref[1] = .0;

    //     if (mode_admitt == 1)
    //     {
    //         param_tuning->Ma_new = param_tuning->Ma;
    //         param_tuning->Ka_new = param_tuning->Ka;
    //         param_tuning->Ba_new = 2 * param_tuning->zeta * sqrt(param_tuning->Ma_new * param_tuning->Ka_new);
    //     }
    // }
    // else
    // {
    //     state_model->posRW_ref[0] = jump->r0;
    //     state_model->posRW_ref[1] = pi / 2.;
    //     state_model->velRW_ref[0] = .0;
    //     state_model->velRW_ref[1] = .0;

    //     if (mode_admitt == 1)
    //     {
    //         param_tuning->Ma_new = param_tuning->Ma;
    //         param_tuning->Ka_new = param_tuning->K_land * param_tuning->Ka;
    //         param_tuning->Ba_new = 2 * param_tuning->zeta_land * sqrt(param_tuning->Ma_new * param_tuning->Ka_new);
    //     }
    // }
}; // Jumping;

void trajectory::Hold(StateModel_* state_model)
{
    state_model->posRW_ref[0] = 0.3536;
    state_model->posRW_ref[1] = pi / 2;
    state_model->forRW_ref[0] = 0;
    state_model->forRW_ref[1] = 0;
};
#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "globVariable.h"
#include "kinematics.h"
#include "trajectory.h"
#include <mujoco/mujoco.h> //Caution Crash
#include "filter.h"
#include "nlopt.h"

//전역변수로 Leg_num 있어야함, PID제어기 몇개 인지도 있어야함

class controller
{

private:

    // Pos_PID
    Vector2d Kp_pos; // [Leg_num][dimension] ->ex) [RL][r,theta]
    Vector2d Kd_pos;
    double cut_off_D_pos;
    Vector2d PID_output_pos;
    Vector2d error_pos;
    Vector2d error_old_pos;
    Vector2d error_dot_pos;
    Vector2d error_dot_old_pos;

    // Vel_PID
    Vector2d Kp_vel; // [Leg_num][dimension] ->ex) [RL][r,theta]
    Vector2d Kd_vel;
    double cut_off_D_vel;
    Vector2d PID_output_vel;
    Vector2d error_vel;
    Vector2d error_old_vel;
    Vector2d error_dot_vel;
    Vector2d error_dot_old_vel;

    //Admittance
    double ad_M;
    double ad_B;
    double ad_K;
    double deltaPos[NDOF_LEG];
    double deltaPos_old[NDOF_LEG];
    double deltaPos_old2[NDOF_LEG];

    //DOB
    Vector2d rhs_dob;
    Vector2d rhs_dob_old;
    Vector2d lhs_dob;
    Vector2d lhs_dob_old;
    double lhs_dob_LPF[NDOF_LEG];
    double lhs_dob_LPF_old[NDOF_LEG];
    double rhs_dob_LPF[NDOF_LEG];
    double rhs_dob_LPF_old[NDOF_LEG];
    Vector2d tauDist_hat;
    double cut_off_dob;

    //QLSLIP DOB
    double L = 0.25; // Leg parameter
    Matrix2d I_d;

    //FOB
    Vector2d rhs_fob;
    Vector2d rhs_fob_old;
    Vector2d lhs_fob_LPF;
    Vector2d lhs_fob_LPF_old;
    Vector2d rhs_fob_LPF;
    Vector2d rhs_fob_LPF_old;
    Vector2d tauExt_hat;
    Vector2d forceExt_hat;
    Vector2d forceExt_hat_old;
    Vector2d forceExt_hat_old2;
    double cut_off_fob;

    //QLSLIP
    double qlslip_init;
    string phase_id;
    double pivot_time;
    double norm_time;

    double Ts;
    double Tb;

    double v_to_y;
    double Tf;
    double a;
    double b;
    double epsilon1;
    double epsilon2;
    double theta_r_sw;
    double theta_r_to_des;
    double theta_r_to;
    double theta_r_to_dot;
    double theta_r_td_des;
    double theta_r_td;
    double theta_r_td_dot;

    double alpha[6];
    double beta[6];

    Vector2d QLSLIP_output;
    Vector2d QLSLIP_output_tau;

    typedef struct {
        double top_value;
        double final_time;
    }trajectory_cost_data;

    typedef struct {
        double final_time;
        double ceq_1;
        double ceq_2;
        double ceq_3;
        double ceq_4;
    }trajectory_equalityconstraints_data;

public:
    controller(); //생성자
    ~controller(); //소멸자

    void pid_gain_pos(double kp, double kd, double cut_off_pos, int flag);
    void pid_gain_vel(double kp, double kd, double cut_off_vel, int flag);
    Vector2d PID_pos(StateModel_* state_model);
    void PID_vel(StateModel_* state_model);
    void admittanceCtrl(StateModel_* state_model, double m, double b, double k, int flag);
    Vector2d DOBRW(StateModel_* state_model, double cut_off ,int flag);
    Vector2d QLSLIPDOBRW(StateModel_* state_model, double cut_off , double m_d, int flag);
    void FOBRW(StateModel_* state_model, double cut_off);
    void ctrl_update(); // 이걸 사용할지에 대해서 생각해보기

    Vector2d QLSLIP(StateModel_* state_model, double time_run, double v_d, double r_d, double m_d, double k, double b_theta, double h1, double rTop);
    void flightTrajOpt(StateModel_* state_model, double *coefficients, Vector2d init, Vector2d final, double valueTop, double time_interval);
    static double trajectorycost(unsigned n, const double *x, double *grad, void *costdata);
    static void trajectoryequalityconstraints(unsigned m, double *result, unsigned n, const double *x,  double *grad, void *equalitydata);

};


#endif // !__CONTROLLER_H__

#include "controller.h"
#include <iostream>
using namespace std;


controller::controller()
{
    for (int i = 0; i < NDOF_LEG; i++)
    {
        // Pos PID
        Kp_pos[i] = 0.0;
        Kd_pos[i] = 0.0;

        error_pos[i] = 0.0;
        error_old_pos[i] = error_pos[i];
        error_dot_pos[i] = 0.0;
        error_dot_old_pos[i] = error_dot_pos[i];
        PID_output_pos[i] = 0;

        // Vel PID
        Kp_vel[i] = 0.0;
        Kd_vel[i] = 0.0;

        error_vel[i] = 0.0;
        error_old_vel[i] = error_vel[i];
        error_dot_vel[i] = 0.0;
        error_dot_old_vel[i] = error_dot_vel[i];
        PID_output_vel[i] = 0;

        //Admittance
        deltaPos[i] = 0.0;
        deltaPos_old[i] = deltaPos[i];
        deltaPos_old2[i] = deltaPos_old[i];

        //RWDOB
        rhs_dob[i] = 0.0;
        rhs_dob_old[i] = rhs_dob[i];
        lhs_dob[i] = 0.0;
        lhs_dob_old[i] = lhs_dob[i];
        lhs_dob_LPF[i] = 0.0;
        lhs_dob_LPF_old[i] = lhs_dob_LPF[i];
        rhs_dob_LPF[i] = 0.0;
        rhs_dob_LPF_old[i] = rhs_dob_LPF[i];
        tauDist_hat[i] = 0.0;

        //RWFOB
        rhs_fob[i] = 0.0;
        rhs_fob_old[i] = rhs_fob[i];
        lhs_fob_LPF[i] = 0.0;
        lhs_fob_LPF_old[i] = lhs_fob_LPF[i];
        rhs_fob_LPF[i] = 0.0;
        rhs_fob_LPF_old[i] = rhs_fob_LPF[i];
        tauExt_hat[i] = 0.0;
        forceExt_hat[i] = 0.0;
        forceExt_hat_old[i] = forceExt_hat[i];
        forceExt_hat_old2[i] = forceExt_hat_old[i];

        //QLSLIP
        qlslip_init = 0;
        QLSLIP_output_tau[i] = 0;
    }

};
controller::~controller(){}

void controller::pid_gain_pos(double kp, double kd, double cut_off, int flag)
{
    if (flag == 1) {
      for (int i = 0; i < NDOF_LEG; i++)
      {
          Kp_pos[i] = kp;
          Kd_pos[i] = kd;
          cut_off_D_pos = cut_off;
      }
    }
    else {
      for (int i = 0; i < NDOF_LEG; i++)
      {
          Kp_pos[i] = 0;
          Kd_pos[i] = 0;
          cut_off_D_pos = cut_off;
      }
    }
};

void controller::pid_gain_vel(double kp, double kd, double cut_off, int flag)
{
    if (flag == 1) {
      for (int i = 0; i < NDOF_LEG; i++)
      {
          Kp_vel[i] = kp;
          Kd_vel[i] = kd;
          cut_off_D_vel = cut_off;
      }
    }
    else {
      for (int i = 0; i < NDOF_LEG; i++)
      {
          Kp_vel[i] = 0;
          Kd_vel[i] = 0;
          cut_off_D_vel = cut_off;
      }
    }
};

void controller::ctrl_update()
{
    for (int i = 0; i < NDOF_LEG; i++)
    {
        //PID pos
        error_old_pos[i] = error_pos[i];
        error_dot_old_pos[i] = error_dot_pos[i];
        //PID vel
        error_old_vel[i] = error_vel[i];
        error_dot_old_vel[i] = error_dot_vel[i];

        //admittance
        deltaPos_old2[i] = deltaPos_old[i];
        deltaPos_old[i] = deltaPos[i];

        //DOB
        rhs_dob_old[i] = rhs_dob[i];
        lhs_dob_old[i] = lhs_dob[i];
        rhs_dob_LPF_old[i] = rhs_dob_LPF[i];
        lhs_dob_LPF_old[i] = lhs_dob_LPF[i];

        //FOB
        rhs_fob_old[i] = rhs_fob[i];
        rhs_fob_LPF_old[i] = rhs_fob_LPF[i];
        lhs_fob_LPF_old[i] = lhs_fob_LPF[i];
        forceExt_hat_old2[i] = forceExt_hat_old[i];
        forceExt_hat_old[i] = forceExt_hat[i];
    }

};

Vector2d controller::PID_pos(StateModel_* state_model)
{
    for (int i = 0; i < NDOF_LEG; i++) // Error를 state 모델에 넣을 필요 있는지 생각해봐야함. error는 여기에 있어도 됨. //error들 update 해줘야함
    {
        error_pos[i] = state_model->posRW_ref[i] - state_model->posRW[i];
        error_old_pos[i] = state_model->posRW_ref_old[i] - state_model->posRW_old[i];

        error_dot_pos[i] = tustin_derivative(error_pos[i], error_old_pos[i], error_dot_old_pos[i], cut_off_D_pos);

        // DOB 끄면 PID만 사용해야하는데 state model에 넣지 않아도 되는지 생각해봐야함.
        PID_output_pos[i] = Kp_pos[i] * error_pos[i] + Kd_pos[i] * error_dot_pos[i]; // 이걸 return을 사용하면?
    }
    return PID_output_pos;

};

void controller::PID_vel(StateModel_* state_model)
{
    for (int i = 0; i < NDOF_LEG; i++)
    {
        error_vel[i] = state_model->velRW_ref[i] - state_model->velRW[i]; //error is no vector
        error_old_vel[i] = state_model->velRW_ref_old[i] - state_model->velRW_old[i];

        error_dot_vel[i] = tustin_derivative(error_vel[i], error_old_vel[i], error_dot_old_vel[i], cut_off_D_vel);

        // DOB 끄면 PID만 사용해야하는데 state model에 넣지 않아도 되는지 생각해봐야함.
        PID_output_vel[i] = Kp_vel[i] * error_vel[i] + Kd_vel[i] * error_dot_vel[i]; // 이걸 return을 사용하면?
    }
}; // negative velocity PID feedback

void controller::admittanceCtrl(StateModel_* state_model, double omega_n , double zeta, double k, int flag)
{
    // 현재 omega_n, zeta, k 로 tunning 하고 있는데, 변환식을 통해 아래에 적어주면 된다
    ad_M = k/(pow(omega_n,2));
    ad_B = 2*zeta*k/omega_n;
    ad_K = k;

    double c1 = 4 * ad_M + 2 * ad_B * Ts + ad_K * pow(Ts, 2);
    double c2 = -8 * ad_M + 2 * ad_K * pow(Ts, 2);
    double c3 = 4 * ad_M - 2 * ad_B * Ts + ad_K * pow(Ts, 2);

    deltaPos[0] =
        (pow(Ts, 2) * forceExt_hat[0] + 2 * pow(Ts, 2) * forceExt_hat_old[0] +
            pow(Ts, 2) * forceExt_hat_old2[0] - c2 * deltaPos_old[0] - c3 * deltaPos_old2[0]) / c1;

    //cout<< " deltaPos: " << deltaPos[0]<< endl;

        if (flag == true)
            state_model->posRW_ref[0] = state_model->posRW_ref[0] + deltaPos[0];
};

Vector2d controller::DOBRW(StateModel_* state_model, double cut_off ,int flag)
{
    cut_off_dob = cut_off;
    lhs_dob = state_model->tau_bi_old;
    rhs_dob = state_model->Lamda_nominal_DOB * state_model->qddot_bi_tustin;

    if (flag == true)
    {
        for (int i = 0; i < NDOF_LEG; i++)
        {
            lhs_dob_LPF[i] = lowpassfilter(lhs_dob[i], lhs_dob_old[i], lhs_dob_LPF_old[i], cut_off_dob);
            rhs_dob_LPF[i] = lowpassfilter(rhs_dob[i], rhs_dob_old[i], rhs_dob_LPF_old[i], cut_off_dob);

            tauDist_hat[i] = lhs_dob_LPF[i] - rhs_dob_LPF[i];
        }
    }
    else
    {
        for (int i = 0; i < NDOF_LEG; i++)
            tauDist_hat[i] = 0;
    }

    return tauDist_hat;

}; // Rotating Workspace DOB

Vector2d controller::QLSLIPDOBRW(StateModel_* state_model, double cut_off , double m_d, int flag)
{
    cut_off_dob = cut_off;

    I_d(0, 0) = m_d * L * (1 - cos(state_model->q[1]));
    I_d(0, 1) = 0;
    I_d(1, 0) = 0;
    I_d(1, 1) = m_d * L * (1 - cos(state_model->q[1]));
    
    lhs_dob = state_model->tau_bi_old;
    rhs_dob = I_d * state_model->qddot_bi_tustin;

    if (flag == true)
    {
        for (int i = 0; i < NDOF_LEG; i++)
        {
            lhs_dob_LPF[i] = lowpassfilter(lhs_dob[i], lhs_dob_old[i], lhs_dob_LPF_old[i], cut_off_dob);
            rhs_dob_LPF[i] = lowpassfilter(rhs_dob[i], rhs_dob_old[i], rhs_dob_LPF_old[i], cut_off_dob);

            tauDist_hat[i] = lhs_dob_LPF[i] - rhs_dob_LPF[i];
        }
    }
    else
    {
        for (int i = 0; i < NDOF_LEG; i++)
            tauDist_hat[i] = 0;
    }

    return tauDist_hat;

}; // Rotating Workspace DOB

void controller::FOBRW(StateModel_* state_model, double cut_off)
{
    cut_off_fob = cut_off;

    // Corioli & Gravity term 만들어 놓음 필요하면 쓰면 됩니닷

    rhs_fob = state_model->Lamda_nominal_FOB * state_model->qddot_bi_tustin;

    for (int i = 0; i < NDOF_LEG; i++)
    {
        lhs_fob_LPF[i] = lowpassfilter(state_model->tau_bi_old[i], state_model->tau_bi_old2[i], lhs_fob_LPF_old[i], cut_off_fob);
        rhs_fob_LPF[i] = lowpassfilter(rhs_fob[i], rhs_fob_old[i], rhs_fob_LPF_old[i], cut_off_fob);

        tauExt_hat[i] = rhs_fob_LPF[i] - lhs_fob_LPF[i];

    }
    forceExt_hat = state_model->jacbRW_trans_inv * tauExt_hat;

    for (int i = 0; i < NDOF_LEG; i++)
    {
      state_model->GRF_FOB[i] = forceExt_hat[i];
    }

} // Rotating WorkspaceForce Observer

Vector2d controller::QLSLIP(StateModel_* state_model, double time_run, double v_d, double r_d, double m_d, double k, double b_theta, double h1, double rTop) 
{
    double omega_d = - v_d / r_d;

    if (qlslip_init == 0)
    {
        // QLSLIP initialization
        phase_id = "Stance";
        theta_r_td_des = state_model->posRW[1] - pi/2;
        theta_r_td = state_model->posRW[1]  - pi/2;
        theta_r_td_dot = 0;//- sqrt(k / m_d) * m_d * g * tan(sqrt(k / m_d) * theta_r_td_des / omega_d) / k;
        pivot_time = 0;
        Ts = - (2 * theta_r_td) / omega_d;
        qlslip_init = 1;
        printf("init setting \n");
    }

    norm_time = time_run - pivot_time;

    // Touch Down / Take Off event Detection
    // Touch Down event catch
    if ((phase_id == "Flight") & (norm_time >= Tf)) 
    {
        printf("touch down event \n");
        phase_id = "Stance"; // Setting Phase ID to Stance
        pivot_time = time_run;
        norm_time = 0;
        theta_r_td = state_model->posRW[1] - pi/2;
        theta_r_td_dot = state_model->velRW[1] / state_model->posRW[0];
        Ts = - (2 * theta_r_td) / omega_d;
        if (Ts < 0) {
            printf("Error: Stance Time is negative value, %f \n", Ts);
        }
    }
    // Take Off event catch
    if ((phase_id == "Stance") & (norm_time >= Ts)) 
    {
        printf("take off event \n");
        phase_id = "Flight"; // Setting Phase ID to Flight
        pivot_time = time_run;
        norm_time = 0;
        theta_r_to = state_model->posRW[1] - pi/2;
        theta_r_to_dot = state_model->velRW[1] / state_model->posRW[0];
        v_to_y = - (state_model->velRW[0] * sin(theta_r_to + pi/2) + state_model->velRW[1] * cos(theta_r_to + pi/2));
        if (v_to_y < 0) {
            printf("Error: y-direction take off velocity is negative value, %f \n", v_to_y);
        }
        Tf = (2 * v_to_y) / g;
        if (Tf < 0) {
            printf("Error: Flight Time is negative value, %f \n", Tf);
        }

        // Trajectory Optimization
        a = b_theta / (m_d * r_d);
        b = g / r_d;
        epsilon1 = (- a + sqrt(pow(a, 2) - 4 * b)) / 2; // a^2 - 4b must be positive
        epsilon2 = (- a - sqrt(pow(a, 2) - 4 * b)) / 2;
        if (pow(a, 2) - 4 * b < 0) {
            printf("Error: theta_r_sw is not calculatable, because of Imagine value \n");
        }
        //theta_r_to_des = (1 / (epsilon2 - epsilon1)) * (epsilon2 * exp(epsilon1 * Ts) - epsilon1 * exp(epsilon2 * Ts)) * theta_r_td
        //                + (1 / (epsilon2 - epsilon1)) * (- exp(epsilon1 * Ts) + exp(epsilon2 * Ts)) * theta_r_td_dot
        //                + (b_theta / (b * m_d * r_d)) * ((a * (exp(epsilon1 * Ts) - exp(epsilon2 * Ts)) + (epsilon1 * exp(epsilon1 * Ts) - epsilon2 * exp(epsilon2 * Ts)) + 1) / (epsilon2 - epsilon1)) * omega_d;
        //if (theta_r_to_des > 0) {
        //    printf("Error: desired take off angle is positive value, %f \n", theta_r_to_des);
        //}
        //theta_r_sw = - 2 * theta_r_to_des;
        theta_r_sw = 2 * omega_d * ((epsilon2 - epsilon1) + (epsilon1 * exp(epsilon1 * Ts) - epsilon2 * exp(epsilon2 * Ts)) + a * (exp(epsilon1 * Ts) - exp(epsilon2 * Ts))) / (epsilon1 * epsilon2 * (exp(epsilon1 * Ts) - exp(epsilon2 * Ts)));
        if (theta_r_sw < 0) {
            printf("Error: swing angle is negative value, %f \n", theta_r_sw);
        }
        theta_r_to_des = - (1/2) * theta_r_sw;
        theta_r_td_des = theta_r_to + h1 * (theta_r_to_des - theta_r_to) + theta_r_sw;
        Vector2d R0;
        Vector2d Rf;
        Vector2d Theta_r0;
        Vector2d Theta_rf;
        R0[0] = state_model->posRW[0];
        R0[1] = state_model->velRW[0];
        Rf[0] = r_d;
        Rf[1] = - sqrt(k / m_d) * m_d * g * tan(sqrt(k / m_d) * theta_r_td_des / omega_d) / k;
        Theta_r0[0] = theta_r_to;
        Theta_r0[1] = theta_r_to_dot;
        Theta_rf[0] = theta_r_td_des;
        Theta_rf[1] = omega_d;
        double theta_rTop = 0;
        flightTrajOpt(state_model, alpha, R0, Rf, rTop, Tf);
        flightTrajOpt(state_model, beta, Theta_r0, Theta_rf, theta_rTop, Tf);
    }

    // Stance Phase Control
    if (phase_id == "Stance")
    {
        printf("stance phase \n");
        QLSLIP_output[0] = k * (r_d - state_model->posRW[0])
                           + (1/2) * m_d * (cos(state_model->q[1]) - 1) * (state_model->qdot_bi_tustin[1] - state_model->qdot_bi_tustin[0]) * state_model->velRW[0]
                           + m_d * g; // r-direction impedance control

        QLSLIP_output[1] = b_theta * (omega_d - (state_model->velRW[1] / state_model->posRW[0]))
                           - m_d * g * (state_model->posRW[1] - pi/2); // theta_r-direction velocity feedback control

        QLSLIP_output_tau = state_model->jacbRW_trans * QLSLIP_output; // inverse statics calculation
    }
    // Flight Phase Control
    if (phase_id == "Flight") 
    {
        //printf("flight phase \n");
        state_model->posRW_ref[0] = alpha[0] * 1 
                                  + alpha[1] * norm_time
                                  + alpha[2] * pow(norm_time, 2)
                                  + alpha[3] * pow(norm_time, 3)
                                  + alpha[4] * pow(norm_time, 4)
                                  + alpha[5] * pow(norm_time, 5); // r-direction desired position calculation at current time

        state_model->posRW_ref[1] = beta[0] * 1 
                                  + beta[1] * norm_time
                                  + beta[2] * pow(norm_time, 2)
                                  + beta[3] * pow(norm_time, 3)
                                  + beta[4] * pow(norm_time, 4)
                                  + beta[5] * pow(norm_time, 5)
                                  + pi/2; // theta_r-direction desired position calculation at current time

        QLSLIP_output_tau = state_model->jacbRW_trans * PID_pos(state_model); // position PID calculation & inverse statics calculation
    }
    //printf("pass \n");

    return QLSLIP_output_tau;
};

void controller::flightTrajOpt(StateModel_* state_model, double *coefficients, Vector2d init, Vector2d final, double valueTop, double time_interval)
{
    nlopt_opt opt;

    //establish sizes
    unsigned n = 6; //number of decision variables
    unsigned m_eq = 4; //number of equality constraints

    //bounds for decision variables
    double lb[] = { -HUGE_VAL, -HUGE_VAL, -HUGE_VAL, -HUGE_VAL, -HUGE_VAL, -HUGE_VAL }; /* lower bounds */
    double ub[] = { HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL }; /* upper bounds */

    //Set the algorithm and dimensionality
    //G,L = global/local
    //D,N = derivative / no derivative
    opt = nlopt_create(NLOPT_GN_MLSL_LDS, n); /* algorithm and dimensionality: set Local No derivative, NLOPT_LD_LBFGS*/

    //Set the lower and upper bounds
    nlopt_set_lower_bounds(opt, lb);
    nlopt_set_upper_bounds(opt, ub);

    //Set up cost
    trajectory_cost_data costdata;
    costdata.top_value = valueTop;
    costdata.final_time = time_interval;
    nlopt_set_min_objective(opt, trajectorycost, &costdata);

    //set up equality constraint
    double tol_eq[]={1e-4,1e-4,1e-4,1e-4};
    trajectory_equalityconstraints_data equalitydata;
    equalitydata.final_time = time_interval;
    equalitydata.ceq_1 = init[0];
    equalitydata.ceq_2 = final[0];
    equalitydata.ceq_3 = init[1];
    equalitydata.ceq_4 = final[1];
    nlopt_add_equality_mconstraint(opt, m_eq, trajectoryequalityconstraints, &equalitydata, tol_eq);

    nlopt_set_xtol_rel(opt, 1e-4);
    double x[] = { 1, 1, 1, 1, 1, 1 };  // initial guess
    double minf; /* `*`the` `minimum` `objective` `value,` `upon` `return`*` */
    if (nlopt_optimize(opt, x, &minf) < 0) {
        printf("nlopt failed!\n");
        printf("%f \n", minf);
        for (int i=0; i<6; i++) {
            coefficients[i] = 0;
        }
    }
    else {
        for (int i=0; i<6; i++) {
            coefficients[i] = x[i];
        }
    }

    nlopt_destroy(opt);
};

double controller::trajectorycost(unsigned n, const double *x, double *grad, void *costdata)
{
    trajectory_cost_data *data = (trajectory_cost_data *) costdata;

    double cost = 0;
    double t = data->final_time;
    double r = data->top_value;
    cost = pow(r, 2) * t
            - (1/3) * r * pow(t, 6) * x[5]
            - (2/5) * r * pow(t, 5) * x[4]
            - (1/2) * r * pow(t, 4) * x[3]
            - (2/3) * r * pow(t, 3) * x[2]
            - r * pow(t, 2) * x[1]
            - 2 * r * t * x[0]
            + (1/11) * pow(t, 11) * pow(x[5], 2)
            + (1/5) * pow(t, 10) * x[4] * x[5]
            + (1/9) * pow(t, 9) * pow(x[4], 2)
            + (2/9) * pow(t, 9) * x[3] * x[5]
            + (1/4) * pow(t, 8) * x[3] * x[4]
            + (1/4) * pow(t, 8) * x[2] * x[5]
            + (1/7) * pow(t, 7) * pow(x[3], 2)
            + (2/7) * pow(t, 7) * x[2] * x[4]
            + (2/7) * pow(t, 7) * x[1] * x[5]
            + (1/3) * pow(t, 6) * x[2] * x[3]
            + (1/3) * pow(t, 6) * x[1] * x[4]
            + (1/3) * pow(t, 6) * x[0] * x[5]
            + (1/5) * pow(t, 5) * pow(x[2], 2)
            + (2/5) * pow(t, 5) * x[1] * x[3]
            + (2/5) * pow(t, 5) * x[0] * x[4]
            + (1/2) * pow(t, 4) * x[1] * x[2]
            + (1/2) * pow(t, 4) * x[0] * x[3]
            + (1/3) * pow(t, 3) * pow(x[1], 2)
            + (2/3) * pow(t, 3) * x[0] * x[2]
            + pow(t, 2) * x[0] * x[1]
            + t * pow(x[0], 2);

    return cost;
};

void controller::trajectoryequalityconstraints(unsigned m, double *result, unsigned n, const double *x,  double *grad, void *equalitydata)
{
    trajectory_equalityconstraints_data *data = (trajectory_equalityconstraints_data *) equalitydata;
    
    double c1 = data->ceq_1; // init pos
    double c2 = data->ceq_2; // final pos
    double c3 = data->ceq_3; // init vel
    double c4 = data->ceq_4; // final vel
    double t = data->final_time;

    result[0] = x[0] - c1;
    result[1] = x[0] * 1
              + x[1] * t
              + x[2] * pow(t, 2)
              + x[3] * pow(t, 3)
              + x[4] * pow(t, 4)
              + x[5] * pow(t, 5)
              - c2;
    result[2] = x[1] - c3;
    result[3] = x[1] * 1
              + 2 * x[2] * t
              + 3 * x[3] * pow(t, 2)
              + 4 * x[4] * pow(t, 3)
              + 5 * x[5] * pow(t, 4)
              - c4;
 };
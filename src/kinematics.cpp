#include "kinematics.h"

kinematics::kinematics(){};

kinematics::~kinematics(){};

void kinematics::state_update(StateModel_* state_model)
{

    //printf("%f, %f %f \n\n", state_model->forceExt_hat[0], state_model->forceExt_hat_old[0], state_model->forceExt_hat_old2[0]);
    for (int i = 0; i < NDOF_LEG; i++)
    {
        // Joint
        state_model->q_old[i] = state_model->q[i];
        state_model->q_bi_old[i] = state_model->q_bi[i]; //okay
        state_model->qdot_bi_old[i] = state_model->qdot_bi[i];
        state_model->qdot_bi_tustin_old[i] = state_model->qdot_bi_tustin[i]; //okay
        state_model->qddot_bi_tustin_old[i] = state_model->qddot_bi_tustin[i]; //okay

        // Feedback - RW Kinematics
        state_model->posRW_old[i] = state_model->posRW[i];
        state_model->velRW_old[i] = state_model->velRW[i];
        state_model->posRW_ref_old2[i] = state_model->posRW_ref_old[i];
        state_model->posRW_ref_old[i] = state_model->posRW_ref[i];
        state_model->velRW_ref_old[i] = state_model->velRW_ref[i];
        state_model->forRW_old[i] = state_model->forRW[i];
        state_model->forRW_ref[i] = state_model->forRW_ref_old[i];

        // control input
        state_model->ctrl_input_RW_old[i] = state_model->ctrl_input_RW[i];
        state_model->tau_bi_old2[i] = state_model->tau_bi_old[i];
        state_model->tau_bi_old[i] = state_model->tau_bi[i];
    }

    state_model->H_old = state_model->H;
    //printf("%f, %f \n", state_model->lhs_fob_LPF[0], state_model->lhs_fob_LPF_old[0]);
};


void kinematics::model_param_cal(const mjModel* m, mjData* d, StateModel_* state_model)
{
    cut_off_cal = 1/(2*pi*150);

    /* Trunk Parameters */
    m_hip = 2.5;
    m_trunk_front = 0.;
    m_trunk_rear = 0.;
    m_trunk = 4 * m_hip + m_trunk_front + m_trunk_rear;

    /* Leg Parameters */
    L = 0.25;
    d_thigh = 0.11017; // local position of CoM of thigh
    d_shank = 0.12997; // local position of CoM of shank
    // printf("d_thigh : %f, d_shank : %f \n", d_thigh, d_shank);

    m_thigh = 1.017; // mass of thigh link
    m_shank = 0.143; // mass of shank link
    m_leg = m_thigh + m_shank;
    m_total = m_trunk + 4 * m_leg;
    // printf("m_thigh : %f, m_shank : %f \n", m_thigh, m_shank);

    Izz_thigh = 0.0057;     // MoI of thigh w.r.t. CoM
    Izz_shank = 8.0318e-04; // MoI of shank w.r.t. CoM
    // printf("Izz_thigh : %f, Izz_shank : %f \n", Izz_thigh, Izz_shank);

    Jzz_thigh =
        Izz_thigh + m_thigh * pow(d_thigh, 2); // MoI of thigh w.r.t. HFE
    Jzz_shank =
        Izz_shank + m_shank * pow(d_shank, 2); // MoI of thigh w.r.t. KFE
    // printf("Jzz_thigh : %f, Jzz_shank : %f \n", Jzz_thigh, Jzz_shank);

    double M1 = Jzz_thigh + m_shank * pow(L, 2);
    double M2 = m_shank * d_shank * L * cos(state_model->q[1]);
    double M12 = Jzz_shank;

    MatInertia_bi(0,0) = M1;
    MatInertia_bi(0,1) = M12;
    MatInertia_bi(1,0) = M12;
    MatInertia_bi(1,1) = M2;

    state_model->Lamda_nominal_FOB(0,0) = M1;
    state_model->Lamda_nominal_FOB(0,1) = M12;
    state_model->Lamda_nominal_FOB(1,0) = M12;
    state_model->Lamda_nominal_FOB(1,1) = M2;



    JzzR_thigh  = Jzz_thigh + Jzz_shank + m_shank * pow(L, 2) - 2 * m_shank * d_shank * L * cos(state_model->q[1]);
    JzzR_couple = Jzz_thigh + m_shank * pow(L, 2) - Jzz_shank;
    JzzR_shank = Jzz_thigh + Jzz_shank+ m_shank * pow(L, 2) + 2 * m_shank * d_shank * L * cos(state_model->q[1]);
    // printf("JzzR_thigh : %f, JzzR_shank : %f, JzzR_couple : %f \n", JzzR_thigh, JzzR_shank,
    // JzzR_couple);

    MatInertia_RW(0,0) = JzzR_thigh / (4 * pow(L, 2) * pow(sin(state_model->q[1] / 2), 2));
    MatInertia_RW(0,1) = JzzR_couple / (2 * pow(L, 2) * sin(state_model->q[1]));
    MatInertia_RW(1,0) = JzzR_couple / (2 * pow(L, 2) * sin(state_model->q[1]));
    MatInertia_RW(1,1) = JzzR_shank / (4 * pow(L, 2) * pow(cos(state_model->q[1] / 2), 2));


    Inertia_DOB(0,0) = MatInertia_RW(0,0);
    Inertia_DOB(0,1) = 0;
    Inertia_DOB(1,0) = 0;
    Inertia_DOB(1,1) = MatInertia_RW(1,1);

    double check[4] = { 0 };

    state_model->Lamda_nominal_DOB = state_model->jacbRW_trans*Inertia_DOB*state_model->jacbRW;

    //Coriolis & Gravity
    H[0] = -m_shank * d_shank * L * sin(state_model->q[1]) * pow(state_model->qdot_bi[1], 2)
         - g * (m_thigh * d_thigh + m_shank * L) * cos(state_model->q_bi[0]);

    H[1] = m_shank * d_shank * L * sin(state_model->q[1]) * pow(state_model->qdot_bi[0], 2)
         - g * m_shank * d_shank * cos(state_model->q_bi[1]);

}; // param_model parameter

void kinematics::sensor_measure(const mjModel* m, mjData* d, StateModel_* state_model, double sensor_cutoff)
{
    cut_off_cal = sensor_cutoff;
    /*** (Serial) Joint position ***/
    state_model->q[0] = d->qpos[2];  // (relative) HFE angle
    state_model->q[1] = d->qpos[3];  // (relative) KFE angle

    // state_model->q[0] = d->sensordata[6]; // (relative) HFE angle
    // state_model->q[1] = d->sensordata[7]; // (relative) KFE angle
    state_model->touch_sensor = d->sensordata[8];

    /*** Biarticular Transformation ***/
    state_model->q_bi[0] = state_model->q[0];              // (absolute) HFE angle
    state_model->q_bi[1] = state_model->q[0] + state_model->q[1]; // (absolute) KFE angle

    // state_model->q_bi[0] = d->sensordata[6];                    // (absolute) HFE angle
    // state_model->q_bi[1] = d->sensordata[6] + d->sensordata[7]; // (absolute) KFE angle

    // printf("q1 : %f, q2 : %f \n", d->qpos[0], d->qpos[1]);
    // printf("qm : %f, qb : %f \n\n", q_bi[0], q_bi[1]);

    state_model->qdot_bi[0] = d->qvel[2];
    state_model->qdot_bi[1] = d->qvel[2] + d->qvel[3];
    // printf("qd1 : %f, qd2 : %f \n", d->qvel[0], d->qvel[1]);
    // printf("qdm : %f, qdb : %f \n\n", qdot_bi[0], qdot_bi[1]);

    state_model->qddot_bi[0] = d->qacc[2];
    state_model->qddot_bi[1] = d->qacc[2] + d->qacc[3];
    // printf("qdd1 : %f, qdd2 : %f \n", d->qacc[0], d->qacc[1]);S
    // printf("qddm : %f, qddb : %f \n\n", qddot_bi[0], qddot_bi[1]);

    for (int i = 0; i < NDOF_LEG; i++)
    {
        state_model->qdot_bi_tustin[i] =
            tustin_derivative(state_model->q_bi[i], state_model->q_bi_old[i], state_model->qdot_bi_tustin_old[i],
                cut_off_cal);
        state_model->qddot_bi_tustin[i] =
            tustin_derivative(state_model->qdot_bi_tustin[i], state_model->qdot_bi_tustin_old[i],
                state_model->qddot_bi_tustin_old[i], cut_off_cal);
    }
    // printf("qdot_bi[0]: %f, qdot_bi[1]: %f \n qdot_bi_tust[0]: %f, qdot_bi_tust[1]: %f \n\n",
    // state_Model_FL.qdot_bi[0],
    //        state_Model_FL.qdot_bi[1], state_Model_FL.qdot_bi_tustin[0], state_Model_FL.qdot_bi_tustin[1]);
    // printf("qddot_bi[0]: %f, qddot_bi[1]: %f \n qddot_bi_tust[0]: %f, qddot_bi_tust[1]: %f \n\n",
    //        state_Model_FL.qddot_bi[0], state_Model_FL.qddot_bi[1], state_Model_FL.qddot_bi_tustin[0],
    //        state_Model_FL.qddot_bi_tustin[1]);
};

void kinematics::jacobianRW(StateModel_* state_model)
{
    /*** Rotating Workspace ***/
    state_model->jacbRW(0,0) =  L * sin(state_model->q[1] / 2);
    state_model->jacbRW(0,1) = -L * sin(state_model->q[1] / 2);
    state_model->jacbRW(1,0) =  L * cos(state_model->q[1] / 2);
    state_model->jacbRW(1,1) =  L * cos(state_model->q[1] / 2);

    state_model->jacbRW_trans = state_model->jacbRW.transpose();

    state_model->jacbRW_trans_inv = state_model->jacbRW_trans.inverse();
};

void kinematics::fwdKinematics_cal(StateModel_* state_model)
{
    state_model->posRW[0] = 2 * L * cos((state_model->q_bi[1] - state_model->q_bi[0]) / 2); // r
    state_model->posRW[1] = (state_model->q_bi[0] + state_model->q_bi[1]) / 2;                           // qr

    state_model->velRW = state_model->jacbRW*state_model->qdot_bi_tustin;

    state_model->forRW = state_model->jacbRW_trans_inv * (state_model->Lamda_nominal_DOB * state_model-> qddot_bi_tustin); // force_est

};

void kinematics::state_init(const mjModel* m, mjData* d, StateModel_* state_model)
{
    state_model->q[0] = d->qpos[2];
    state_model->q[1] = d->qpos[3];

    state_model->q_bi[0] = state_model->q[0];
    state_model->q_bi[1] = state_model->q[0] + state_model->q[1];

    // RW coordinates initialization
    state_model->r0 = 2 * L * cos((state_model->q_bi[1] - state_model->q_bi[0]) / 2);

    state_model->posRW[0] = 2 * L * cos((state_model->q_bi[1] - state_model->q_bi[0]) / 2);
    state_model->posRW[1] = (state_model->q_bi[0] + state_model->q_bi[1]) / 2;

    state_model->posRW_ref[0] = 2 * L * cos((state_model->q_bi[1] - state_model->q_bi[0]) / 2);
    state_model->posRW_ref[1] = (state_model->q_bi[0] + state_model->q_bi[1]) / 2;


    state_model->touch_sensor = 0;

    state_model->Lamda_nominal_DOB(0,0)= 0.0;
    state_model->Lamda_nominal_DOB(0,1)= 0.0;
    state_model->Lamda_nominal_DOB(1,0)= 0.0;
    state_model->Lamda_nominal_DOB(1,1)= 0.0;

    //printf("%f \n", state_model->r0);
    for (int i = 0; i < NDOF_LEG; i++)
    {
        // Joint coordinates [k-1] values
        state_model->q_bi_old[i] = state_model->q_bi[i];

        state_model->qdot_bi[i] = 0.0;
        state_model->qdot_bi_tustin[i] = 0.;
        state_model->qdot_bi_tustin_old[i] = state_model->qdot_bi_tustin[i];

        state_model->qddot_bi[i] = 0.;
        state_model->qddot_bi_tustin[i] = 0.;
        state_model->qddot_bi_tustin_old[i] = state_model->qddot_bi_tustin[i];

        state_model->tau_bi[i] = 0.;
        state_model->tau_bi_old[i] = state_model->tau_bi[i];
        state_model->tau_bi_old2[i] = state_model->tau_bi_old[i];

        // RW coordinates [k-1] values
        state_model->posRW_old[i] = state_model->posRW[i];
        state_model->posRW_ref_old[i] = state_model->posRW_ref[i];
        state_model->posRW_ref_old2[i] = state_model->posRW_ref_old[i];

        state_model->velRW[i] = .0;
        state_model->velRW_old[i] = state_model->velRW[i];
        state_model->velRW_ref[i] = 0.;
        state_model->velRW_ref_old[i] = state_model->velRW_ref[i];

        state_model->forRW[i] = 0;
        state_model->forRW_old[i] = state_model->forRW[i];
        state_model->forRW_ref[i] = 0;
        state_model->forRW_ref[i] = 0;
        state_model->forRW_ref_old[i] = state_model->forRW[i];

        state_model->ctrl_input_RW[i] = 0.;
        state_model->ctrl_input_RW_old[i] = state_model->ctrl_input_RW[i];

        // Mg Trajectory
        state_model->tau_ff[i]=0.;
    }
    // printf("%f, %f \n", state_model->q_bi_old[0], state_model->q_bi_old[1]);
};

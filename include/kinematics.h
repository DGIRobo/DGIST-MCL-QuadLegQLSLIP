#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "globVariable.h"
#include "filter.h"
#include <mujoco/mujoco.h> //Caution Crash


class kinematics
{

private:
    /* Trunk Parameter */
    double m_hip;         // mass of hip torso
    double m_trunk_front; // mass of front trunk
    double m_trunk_rear;  // mass of rear trunk
    double m_trunk;       // total mass of trunk
    double m_total;       // total robot mass

    /* Leg Parameter */
    double L; // leg length : thigh and shank links' length are assumed to be the same

    double m_thigh; // mass of thigh link
    double m_shank; // mass of shank link
    double m_leg;   // mass of leg

    double d_thigh; // CoM pos of thigh w.r.t HFE
    double d_shank; // CoM pos of shank w.r.t KFE

    double Izz_thigh; // MoI(z) of thigh w.r.t its CoM
    double Izz_shank; // MoI(z) of shank w.r.t its CoM

    double Jzz_thigh; // MoI(z) of thigh w.r.t HFE
    double Jzz_shank; // MoI(z) of shank w.r.t KFE

    double JzzR_thigh;
    double JzzR_shank;
    double JzzR_couple;

    double cut_off_cal; // Using in calculate vel,acc

    Matrix2d MatInertia_bi;
    Matrix2d MatInertia_RW;
    Matrix2d Inertia_DOB;

    Vector2d H;
    Vector2d H_old;

public:
    kinematics(); //생성자
    ~kinematics(); //소멸자

    void state_update(StateModel_* state_model);
    void model_param_cal(const mjModel* m, mjData* d, StateModel_* state_model);
    void sensor_measure(const mjModel* m, mjData* d, StateModel_* state_model, double sensor_cutoff);
    void jacobianRW(StateModel_* state_model);
    void fwdKinematics_cal(StateModel_* state_model);
    void state_init(const mjModel* m, mjData* d, StateModel_* state_model);

    //void state_init(const mjModel* m, mjData* d, StateModel_* state_model, ParamModel_* param_model); -> subtitude param_model to kinematic Class variable

};

#endif // !__KINEMATICS_H__

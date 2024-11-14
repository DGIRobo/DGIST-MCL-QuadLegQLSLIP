#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h> //for bool
//#include<unistd.h> //for usl eep
#include <math.h>
//#include <resource.h>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "controller.h"
#include "dataLogging.h"
#include "animation.h"
#include "kinematics.h"
#include "trajectory.h"


mjvFigure figPosRW;         // RW position tracking plot
mjvFigure figFOB;           // RWFOB GRF estimation plot
mjvFigure figTrunkState;    // Trunk state plot

double simEndtime = 5;	// Simulation End Time
StateModel_  state_Model_FL;
controller ctrl_FL; // other class is in main loop
kinematics kin_FL;
trajectory tra_FL;

int cmd_motion_type = 0;
int flag_pid = 1;           // flag for switching ON/OFF PID
int flag_DOB = 1;           // flag for switching ON/OFF RWDOB
int flag_admitt = 0;        // flag for switching ON/OFF admittance control

double sensor_cutoff = 300;

double p_gain = 1;
double d_gain = 0.8;
double pid_cutoff = 100;

double v_d = 0.4;
double r_d = 0.35;
double m_d = 0.1;
double k = 20;
double b_theta = 0.4;
double h1 = 1;
double rTop = 0.1;

double RWDOB_cutoff = 100;

//double FOB_cutoff = 100;

//double omega_n = 50000;
//double zeta = 5;
//double k = 10000;

/***************** Main Controller *****************/
void mycontroller(const mjModel* m, mjData* d)
{

    /* Controllers */
    double time_run = d->time;

    // Force Observer
    //ctrl_FL.FOBRW(&state_Model_FL, FOB_cutoff); // Rotating Workspace Force Observer (RWFOB)

    //Admittance Control
    //ctrl_FL.admittanceCtrl(&state_Model_FL, omega_n, zeta, k, flag_admitt); //parameter(omega_n,zeta,k)

    // PID Control
    ctrl_FL.pid_gain_pos(p_gain, d_gain, pid_cutoff, flag_pid); //(kp,kd,freq)
    //state_Model_FL.tau_bi = state_Model_FL.jacbRW_trans * ctrl_FL.PID_pos(&state_Model_FL); // RW position feedback

    // QLSLIP Control
    state_Model_FL.tau_bi = ctrl_FL.QLSLIP(&state_Model_FL, time_run, v_d, r_d, m_d, k, b_theta, h1, rTop); // RW force feedback

    // DOB control
    state_Model_FL.tau_bi = state_Model_FL.tau_bi + ctrl_FL.QLSLIPDOBRW(&state_Model_FL, RWDOB_cutoff, m_d, flag_DOB);

   // Torque input Biarticular
    d->ctrl[0] = state_Model_FL.tau_bi[0] + state_Model_FL.tau_bi[1] ;
    d->ctrl[1] = state_Model_FL.tau_bi[1];

    if (loop_index % data_frequency == 0) {
        save_data(m, d, &state_Model_FL);
    }
    loop_index += 1;
}


/***************** Main Function *****************/
int main(int argc, const char** argv)
{
    // activate software
    mj_activate("mjkey.txt");

    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if (argc < 2)
        m = mj_loadXML(filename, 0, error, 1000);

    else
        if (strlen(argv[1]) > 4 && !strcmp(argv[1] + strlen(argv[1]) - 4, ".mjb"))
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if (!m)
        mju_error_s("Load model error: %s", error);


    // make data
    d = mj_makeData(m);

    // Initialize GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    double arr_view[] = {-88.95, -17.5, 1.8, 0.04, 0.000000, 0.27};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    fid = fopen(datapath, "w");
    init_save_data();


    // Initialization
    mju_copy(d->qpos, m->key_qpos + 0 * m->nq, m->nq); // 얘가 젤 처음에 와야함
    d->qpos[2] = pi/2;
    d->qpos[3] = pi/3;

    kin_FL.model_param_cal(m, d, &state_Model_FL); // state init is before. Caution Error.
    kin_FL.state_init(m,d, &state_Model_FL);

    // custom controller
    mjcb_control = mycontroller;


    /***************** Simulation Loop *****************/
    // use the first while condition if you want to simulate for a period.
    int i = 0;
    while (!glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        // Assuming MuJoCo can simulate faster than real-time, which it usually can,
        // this loop will finish on time for the next frame to be rendered at 60 fps.
        // Otherwise add a cpu timer and exit this loop when it is time to render.

        mjtNum simstart = d->time;
        //printf(" %f  %f \n", d->ctrl[0], d->ctrl[1]);
        state_Model_FL.time = d->time;
        while (d->time - simstart < 1.0 / 60.0)
        {
            kin_FL.sensor_measure(m, d, &state_Model_FL, sensor_cutoff); // get joint sensor data & calculate biarticular angles
            kin_FL.model_param_cal(m, d,&state_Model_FL); // calculate model parameters
            kin_FL.jacobianRW(&state_Model_FL);            // calculate RW Jacobian

            if (d->time < 10)
            {
                //printf("ref: %f \n", state_Model_FL.posRW_ref[1]);
                //printf(" qddot_bi_tustin(0) = %f ,%f ", state_Model_FL.qddot_bi_tustin[0],state_Model_FL.qddot_bi_tustin[1]);
                //printf(" qddot_bi(1) = %f ,%f \n", state_Model_FL.qddot_bi[0],state_Model_FL.qddot_bi[1]);
            }

            kin_FL.fwdKinematics_cal(&state_Model_FL);     // calculate RW Kinematics

            /* Trajectory Generation */
            /*
            if (cmd_motion_type == 0)   // Squat
            {
                tra_FL.Squat(d->time, &state_Model_FL);
            }
            else
            {
                tra_FL.Hold(&state_Model_FL);  // Hold stance

            }
            */

            mj_step(m, d);

            kin_FL.state_update(&state_Model_FL);
            ctrl_FL.ctrl_update();

        }

        if (d->time >= simEndtime) {
            fclose(fid);
            break;
        }
        //printf("%f \n", state_Model_FL.deltaPos[0]);
        // get framebuffer viewport
        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        //opt.frame = mjFRAME_WORLD;
        cam.lookat[0] = d->qpos[0];
        cam.lookat[2] = d->qpos[1];
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    return 1;
}

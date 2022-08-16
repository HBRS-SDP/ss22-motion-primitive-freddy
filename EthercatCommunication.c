/**
 * @file EthercatCommunication.c
 * @author Kavya Shankar (kavya.shankar@smail.inf.h-brs.de)
 * @brief Establishing connection with ehtercat and performing data transfer with robot
 * @date 2022-03-12
 * 
 */
#include "ethercat.h"
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatconfig.h"
#include "ethercatcoe.h"
#include "ethercatdc.h"
#include "ethercatprint.h"
#include "KeloDriveAPI.h"
#include <stdio.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_sf_trig.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit.h>
#include <unistd.h>
#include "PlatformToWheelInverseKinematicsSolver.h"
#include "KELORobotKinematics.h"
#include <gsl/gsl_matrix_double.h>
#include <string.h> 
#include <math.h>

/**
 * @brief Establishing connection with ehtercat and performing data transfer with robot
 * 
 * @param argc 
 * @param argv 
 * @return int to signify successful execution of the function
 */
int main(int argc, char *argv[])
{
    ec_slavet ecx_slave[EC_MAXSLAVE];
    int ecx_slavecount;
    ec_groupt ec_group[EC_MAXGROUP];
    uint8 esibuf[EC_MAXEEPBUF];
    uint32 esimap[EC_MAXEEPBITMAP];
    ec_eringt ec_elist;
    ec_idxstackT ec_idxstack;

    ec_SMcommtypet ec_SMcommtype;
    ec_PDOassignt ec_PDOassign;
    ec_PDOdesct ec_PDOdesc;
    ec_eepromSMt ec_SM;
    ec_eepromFMMUt ec_FMMU;
    boolean EcatError;
    int64 ec_DCtime;
    ecx_portt ecx_port;
    ecx_redportt ecx_redport;
    ecx_contextt ecx_context;
    char IOmap[4096];

    ecx_context.port = &ecx_port;
    ecx_context.slavelist = &ecx_slave[0];
    ecx_context.slavecount = &ecx_slavecount;
    ecx_context.maxslave = EC_MAXSLAVE;
    ecx_context.grouplist = &ec_group[0];
    ecx_context.maxgroup = EC_MAXGROUP;
    ecx_context.esibuf = &esibuf[0];
    ecx_context.esimap = &esimap[0];
    ecx_context.esislave = 0;
    ecx_context.elist = &ec_elist;
    ecx_context.idxstack = &ec_idxstack;

    ecx_context.ecaterror = &EcatError;
    ecx_context.DCtime = &ec_DCtime;
    ecx_context.SMcommtype = &ec_SMcommtype;
    ecx_context.PDOassign = &ec_PDOassign;
    ecx_context.PDOdesc = &ec_PDOdesc;
    ecx_context.eepSM = &ec_SM;
    ecx_context.eepFMMU = &ec_FMMU;
    ecx_context.manualstatechange = 0; // should be 0

    int nWheels = 4; 
    int index_to_EtherCAT[4] = {5, 7, 9, 3};
    bool debug = false;
    char arg[] = "debug";
    if (strcmp(argv[1],arg) == 0)
    {
        debug = true;
    }
    
    /**
     * @brief port name on our PC to initiate connection
     * 
     */
    if (!ecx_init(&ecx_context, "enp61s0"))
    { 
        printf("Failed to initialize EtherCAT\n");
        return 0;
    }

    /**
     * @brief checking establishment of first connection with slave or autoconfig slaves
     * 
     */
    if (!ecx_config_init(&ecx_context, TRUE))
    { 
        printf("NO SLAVES!\n");
        return 0;
    }
    ecx_config_map_group(&ecx_context, IOmap, 0); // PDO - process data object

    printf("%i\n", ecx_slavecount);
    printf("%s\n", ecx_slave[1].name);

    /**
     * @brief Reading all slave names w.r.t their no.
     * 
     */
    for (int i = 1; i <= ecx_slavecount; i++)
    {
        printf("slave \t%i has name \t%s\n", i, ecx_slave[i].name);
    }

    /**
     * @brief waiting for all slaves to reach SAFE_OP state
     * 
     */
    ecx_statecheck(&ecx_context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

    if (ecx_slave[0].state != EC_STATE_SAFE_OP)
    {
        printf("EtherCAT slaves have not reached safe operational state\n");
        ecx_readstate(&ecx_context);

        /**
         * @brief if not all slaves operational, find out which one
         * 
         */
        for (int i = 1; i <= ecx_slavecount; i++)
        {
            if (ecx_slave[i].state != EC_STATE_SAFE_OP)
            {
                printf("Slave %i State= %i\n", i, ecx_slave[i].state);
            }
        }
        return 0;
    }

    rxpdo1_t msg;
    msg.timestamp = 1;
    msg.command1 = 0;
    msg.limit1_p = 0;
    msg.limit1_n = 0;
    msg.limit2_p = 0;
    msg.limit2_n = 0;
    msg.setpoint1 = 0;
    msg.setpoint2 = 0;

    for (unsigned int i = 0; i < nWheels; i++)
    {
        rxpdo1_t *ecData = (rxpdo1_t *)ecx_slave[index_to_EtherCAT[i]].outputs;
        *ecData = msg;
    }

    /**
     * @brief sending process data
     * 
     */
    ecx_send_processdata(&ecx_context); 

    /**
     * @brief setting state to operational
     * 
     */
    ecx_slave[0].state = EC_STATE_OPERATIONAL; 

    /**
     * @brief receiving response from slaves
     * 
     */
    ecx_send_processdata(&ecx_context);
    ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);

    ecx_writestate(&ecx_context, 0);

    /**
     * @brief checking if the slaves have reached operational state
     * 
     */
    ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE); 

    if (ecx_slave[0].state != EC_STATE_OPERATIONAL)
    {
        printf("EtherCAT slaves have not reached operational state\n");
        return 0;
    }
    else
    {
        printf("Operational state reached for all EtherCAT slaves.\n");
    }

    /**
     * @brief initialising pointers to variables used for solving the problem of inverse kinematics
     * 
     */
    int cnt = 0;
    const unsigned int N = 3;
    const unsigned int M = 8;
    double motor_const = 3.5714; //units: (Ampere/Newton-meter)
    gsl_matrix *A = gsl_matrix_alloc(N, M);
    gsl_matrix *A_inv_T = gsl_matrix_alloc(M, N);
    gsl_matrix *A_tmp = gsl_matrix_alloc(N, M);
    gsl_matrix *A_inv_T_tmp = gsl_matrix_alloc(M, N);
    gsl_vector *work = gsl_vector_alloc(N);
    gsl_matrix *W = gsl_matrix_alloc(N, N); 
    gsl_matrix *K = gsl_matrix_alloc(M, M); 
    gsl_vector *u = gsl_vector_alloc(N);
    gsl_matrix *V = gsl_matrix_alloc(N, N);
    gsl_matrix *u_inv = gsl_matrix_alloc(N, N);
    gsl_matrix *b = gsl_matrix_alloc(N, 1);
    gsl_matrix *b_verify = gsl_matrix_alloc(N, 1);

    /**
     * @brief initialising arrays to store pivot angles and wheel torques
     * 
     */
    double pivot_angles[4];
    // //double pivot_position[4];
    // double pivot_velocity[4];
    // double wheel_torques[8];
    // double wheel_velocity[8];
    // double wheel_current[8];
    // double imu_ang_vel[12]; //x,y,z for each wheel unit 
    // double imu_lin_acc[12]; //x,y,z for each wheel unit
    
    //logging def
    FILE *fpt;
    // fpt = fopen("ramp_log_02.csv", "w+");
    // fprintf(fpt,"drive id,f_var,wheel_velocity1,wheel_velocity2,wheel_current1,wheel_current2,pivot_angle,pivot_velocity,imu_ang_vel_x,imu_ang_vel_y,imu_ang_vel_z,imu_lin_acc_x,imu_lin_acc_y,imu_lin_acc_z\n");

    /**
     * @brief setting input platform force values
     * 
     */
    gsl_matrix_set(b, 0, 0, 0.); // force is set in X-direction
    gsl_matrix_set(b, 1, 0, 200.); // force is set in Y-direction
    gsl_matrix_set(b, 2, 0, 0.); // moment is set in anti-clockwise direction

    /**
     * @brief reading data from individual wheels
     * 
     */
    for (unsigned int i = 0; i < nWheels; i++)
    {
        txpdo1_t *ecData = (txpdo1_t *)ecx_slave[index_to_EtherCAT[i]].inputs;
        pivot_angles[i] = ecData->encoder_pivot;
        
        // pivot_velocity[i] = ecData->velocity_pivot;
        // wheel_velocity[i] = ecData->velocity_1;
        // wheel_velocity[i+1] = ecData->velocity_2;
        // wheel_current[i] = ecData->current_1_d;
        // wheel_current[i+1]= ecData->current_2_d;


        // imu_ang_vel[i] = ecData->gyro_x;
        // imu_ang_vel[i+1] = ecData->gyro_y;
        // imu_ang_vel[i+2] = ecData->gyro_z;
        // imu_lin_acc[i] = ecData->accel_x;
        // imu_lin_acc[i+1] = ecData->accel_y;
        // imu_lin_acc[i+2] = ecData->accel_z;
        
        
    }

    /**
     * @brief setting the weght matrix
     * 
     */
    size_t i;
    for (i = 0; i < M; i++)
    {

        gsl_matrix_set(K, i, i, 1.0);
        if (i < N)
        {
            gsl_matrix_set(W, i, i, 1.0);
        }
    }

    /**
     * @brief setting number of iterations until which the force has to be applied
     * 
     */
    double pivot_velocity[4];
    double wheel_torques[8];
    double wheel_velocity[8];
    double wheel_current[8];
    double imu_ang_vel[12]; //x,y,z for each wheel unit 
    double imu_lin_acc[12]; //x,y,z for each wheel unit

    double f_max = 200;
    double f_var = 0;
    double vel_threshold = 10;
    double damping_const = f_max/vel_threshold;

    while (cnt < 400)
    {
        /**
         * @brief setting sleep time between iterations to achieve communication frequency of 1000Hz
         * 
         */

        


        usleep(10000);
        gsl_matrix_set(b, 0, 0, 0.); // force is set in X-direction
        gsl_matrix_set(b, 1, 0, f_var); // force is set in Y-direction
        gsl_matrix_set(b, 2, 0, 0.); // moment is set in anti-clockwise direction


        /**
         * @brief finding wheel torques for each iteration parameterised by pivot angles 
         * 
         */
        functions_main(wheel_torques,
                       pivot_angles,
                       b,
                       b_verify,
                       A,
                       A_inv_T,
                       A_tmp,
                       A_inv_T_tmp,
                       work,
                       W,
                       K,
                       u,
                       V,
                       u_inv,
                       M,
                       N,
                       debug);
        cnt += 1;
        rxpdo1_t msg;
        msg.timestamp = time(NULL);
        msg.command1 = COM1_ENABLE1 | COM1_ENABLE2 | COM1_MODE_TORQUE;
        msg.limit1_p = 3;  // upper limit for first wheel
        msg.limit1_n = -3; // lower limit for first wheel
        msg.limit2_p = 3;  // upper limit for second wheel
        msg.limit2_n = -3; // lower limit for second wheel

        /**
         * @brief setting calculated torque values to individual wheels
         * 
         */
        if (debug)
        {
            //printf("\nsetpoint values:\n");
            
        }
        for (unsigned int i = 0; i < nWheels; i++) // runs all wheels
        {
            msg.setpoint1 = -motor_const * wheel_torques[2 * i]; // units: (rad/sec)
            msg.setpoint2 = motor_const * wheel_torques[2 * i + 1];
            rxpdo1_t *ecData = (rxpdo1_t *)ecx_slave[index_to_EtherCAT[i]].outputs;
            *ecData = msg;

            txpdo1_t *InecData = (txpdo1_t *)ecx_slave[index_to_EtherCAT[i]].inputs;
            /**
             * @brief printing angles after offsetting the pivots
             * 
             */
            pivot_angles[i] = InecData->encoder_pivot;
            pivot_velocity[i] = InecData->velocity_pivot;
            wheel_velocity[i] = InecData->velocity_1;
            wheel_velocity[i+1] = InecData->velocity_2;
            wheel_current[i] = InecData->current_1_d;
            wheel_current[i+1]= InecData->current_2_d;


            imu_ang_vel[i] = InecData->gyro_x;
            imu_ang_vel[i+1] = InecData->gyro_y;
            imu_ang_vel[i+2] = InecData->gyro_z;
            imu_lin_acc[i] = InecData->accel_x;
            imu_lin_acc[i+1] = InecData->accel_y;
            imu_lin_acc[i+2] = InecData->accel_z;
            
            if (debug)
            {
                //printf("%f\t", -motor_const * wheel_torques[2 * i]);
                //printf("%f\t", motor_const * wheel_torques[2 * i + 1]);

                //printf("\nlogging\n");
                
                // fprintf(fpt,"%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                // i,f_var,wheel_velocity[i],wheel_velocity[i+1],wheel_current[i],wheel_current[i+1],
                // pivot_angles[i],pivot_velocity[i],
                // imu_ang_vel[i],imu_ang_vel[i+1],imu_ang_vel[i+2],
                // imu_lin_acc[i],imu_lin_acc[i+1],imu_lin_acc[i+2]);
                
                
            }

            f_var = f_max - damping_const*((fabs(wheel_velocity[i])+fabs(wheel_velocity[i+1]))/2);
            printf("force values: %f \n",f_var);
        }
        printf("log DONE!");
        /**
         * @brief Construct a new ecx send processdata object
         * 
         */
        ecx_send_processdata(&ecx_context); 

        /**
         * @brief Construct a new ecx receive processdata object
         * 
         */
        ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);

        /**
         * @brief receiving updated pivot angles
         * 
         */
        for (unsigned int i = 0; i < nWheels; i++)
        {
            txpdo1_t *ecData = (txpdo1_t *)ecx_slave[index_to_EtherCAT[i]].inputs;
            pivot_angles[i] = ecData->encoder_pivot;
            
        }
    }

    /**
     * @brief releasing memory from all initialised pointers
     * 
     */
    //fclose(fpt);
    gsl_matrix_free(b);
    gsl_matrix_free(b_verify);

    gsl_matrix_free(A);
    gsl_matrix_free(A_inv_T);
    gsl_matrix_free(A_tmp);
    gsl_matrix_free(A_inv_T_tmp);
    gsl_matrix_free(W);
    gsl_matrix_free(K);
    gsl_vector_free(u);
    gsl_matrix_free(u_inv);
    gsl_matrix_free(V);
    gsl_vector_free(work);

    return 0;
}

// SPDX-License-Identifier: LGPL-3.0
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <robif2b/functions/ethercat.h>
#include <robif2b/functions/kelo_drive.h>
#include <robif2b/functions/kelo_power_board.h>
#include <PlatformToWheelInverseKinematicsSolver.h>

static long timespec_to_usec(const struct timespec *t)
{
    const int NSEC_IN_USEC = 1000;
    const int USEC_IN_SEC = 1000000;

    return t->tv_sec * USEC_IN_SEC + t->tv_nsec / NSEC_IN_USEC;
}

//#define NUM_DRIVES 2
//#define NUM_SLAVES 3
#define NUM_DRIVES 4
#define NUM_SLAVES 5

static struct
{
    int num_drives;
    struct
    {
        struct timespec cycle_start;
        struct timespec cycle_end;
        long cycle_time_msr; // [us]
        long cycle_time_exp; // [us]
    } time;
    struct
    {
        struct robif2b_kelo_drive_api_msr_pdo drv_msr_pdo[NUM_DRIVES];
        struct robif2b_kelo_drive_api_cmd_pdo drv_cmd_pdo[NUM_DRIVES];
        struct robif2b_kelo_power_board_api_msr_pdo pb_msr_pdo;
        struct robif2b_kelo_power_board_api_cmd_pdo pb_cmd_pdo;
    } ecat_comm;
    struct
    {
        const char *ethernet_if;
        int error_code;
        int num_exposed_slaves;
        int num_found_slaves;
        int num_active_slaves;
        int slave_idx[NUM_SLAVES];
        const char *name[NUM_SLAVES];
        unsigned int prod_code[NUM_SLAVES];
        size_t input_size[NUM_SLAVES];
        size_t output_size[NUM_SLAVES];
        bool is_connected[NUM_SLAVES];
    } ecat;
    struct
    {
        double pvt_off[NUM_DRIVES];
        double pvt_pos[NUM_DRIVES];
        double pvt_vel[NUM_DRIVES];
        double whl_pos[NUM_DRIVES * 2];
        double whl_vel[NUM_DRIVES * 2];
        double imu_ang_vel[NUM_DRIVES * 3];
        double imu_lin_acc[NUM_DRIVES * 3];
        double bat_volt;
        double bat_cur;
        double bat_pwr;
        int bat_lvl;
    } kelo_msr;
    struct
    {
        enum robif2b_ctrl_mode ctrl_mode[NUM_DRIVES];
        double vel[NUM_DRIVES * 2];
        double trq[NUM_DRIVES * 2];
        double cur[NUM_DRIVES * 2];
        double max_current[NUM_DRIVES * 2];
        double trq_const[NUM_DRIVES * 2];
    } kelo_cmd;
} state;

int main(int argc, char *argv[])
// int main()
{

    // Configuration
    state.num_drives = NUM_DRIVES;
    state.time.cycle_time_exp = 1000;  // [us]
    state.ecat.ethernet_if = "enp5s0"; // Put your ethernet interface here
    state.ecat.num_exposed_slaves = NUM_SLAVES;
    state.ecat.slave_idx[0] = 1;
    state.ecat.slave_idx[1] = 3; // 3
    state.ecat.slave_idx[2] = 5; // 5
    state.ecat.slave_idx[3] = 7; // 7
    state.ecat.slave_idx[4] = 9; // 9

    for (int i = 0; i < NUM_DRIVES; i++)
    {
        state.kelo_cmd.ctrl_mode[i] = ROBIF2B_CTRL_MODE_FORCE,
        state.kelo_cmd.max_current[i * 2 + 0] = 10; // [A]
        state.kelo_cmd.max_current[i * 2 + 1] = 10; // [A]
        state.kelo_cmd.trq_const[i * 2 + 0] = 0.29; // [Nm/A]
        state.kelo_cmd.trq_const[i * 2 + 1] = 0.29; // [Nm/A]
    }
    // state.kelo_msr.pvt_off[0] = 1.523339;
    // state.kelo_msr.pvt_off[1] = 2.399158;
    state.kelo_msr.pvt_off[0] = 0;
    state.kelo_msr.pvt_off[1] = 0;

    state.ecat.name[0] = "KELO_ROBILE";   // KELO_ROBILE / CU1128
    state.ecat.prod_code[0] = 0x02100101; // 0x02100101 / 0x04685432
    state.ecat.input_size[0] = sizeof(state.ecat_comm.pb_msr_pdo);
    state.ecat.output_size[0] = sizeof(state.ecat_comm.pb_cmd_pdo);
    for (int i = 1; i < NUM_DRIVES + 1; i++)
    {
        state.ecat.name[i] = "KELOD105";      // KELOD105 / SWMC
        state.ecat.prod_code[i] = 0x02001001; // 0x02001001 / 0x01705011

        state.ecat.input_size[i] = sizeof(state.ecat_comm.drv_msr_pdo[i - 1]);
        state.ecat.output_size[i] = sizeof(state.ecat_comm.drv_cmd_pdo[i - 1]);
    }

    // Connections
    struct robif2b_ethercat ecat = {
        .ethernet_if = &state.ecat.ethernet_if[0],
        .num_exposed_slaves = &state.ecat.num_exposed_slaves,
        .slave_idx = &state.ecat.slave_idx[0],
        .name = &state.ecat.name[0],
        .product_code = &state.ecat.prod_code[0],
        .input_size = &state.ecat.input_size[0],
        .output_size = &state.ecat.output_size[0],
        .error_code = &state.ecat.error_code,
        .num_initial_slaves = &state.ecat.num_found_slaves,
        .num_current_slaves = &state.ecat.num_active_slaves,
        .is_connected = &state.ecat.is_connected[0],
        .input = (void *[NUM_SLAVES]){
            &state.ecat_comm.pb_msr_pdo,
            &state.ecat_comm.drv_msr_pdo[0],
            &state.ecat_comm.drv_msr_pdo[1],
            &state.ecat_comm.drv_msr_pdo[2],
            &state.ecat_comm.drv_msr_pdo[3]},
        .output = (const void *[NUM_SLAVES]){&state.ecat_comm.pb_cmd_pdo, &state.ecat_comm.drv_cmd_pdo[0], &state.ecat_comm.drv_cmd_pdo[1], &state.ecat_comm.drv_cmd_pdo[2], &state.ecat_comm.drv_cmd_pdo[3]}};

    struct robif2b_kelo_drive_encoder drive_enc = {
        .num_drives = &state.num_drives,
        .msr_pdo = &state.ecat_comm.drv_msr_pdo[0],
        .wheel_pos_msr = &state.kelo_msr.whl_pos[0],
        .wheel_vel_msr = &state.kelo_msr.whl_vel[0],
        .pivot_pos_msr = &state.kelo_msr.pvt_pos[0],
        .pivot_vel_msr = &state.kelo_msr.pvt_vel[0],
        .pivot_pos_off = &state.kelo_msr.pvt_off[0]};

    struct robif2b_kelo_drive_imu imu = {
        .num_drives = &state.num_drives,
        .msr_pdo = &state.ecat_comm.drv_msr_pdo[0],
        .imu_ang_vel_msr = &state.kelo_msr.imu_ang_vel[0],
        .imu_lin_acc_msr = &state.kelo_msr.imu_lin_acc[0]};

    struct robif2b_kelo_drive_actuator wheel_act = {
        .num_drives = &state.num_drives,
        .cmd_pdo = &state.ecat_comm.drv_cmd_pdo[0],
        .ctrl_mode = &state.kelo_cmd.ctrl_mode[0],
        .act_vel_cmd = &state.kelo_cmd.vel[0],
        .act_trq_cmd = &state.kelo_cmd.trq[0],
        .act_cur_cmd = &state.kelo_cmd.cur[0],
        .max_current = &state.kelo_cmd.max_current[0],
        .trq_const = &state.kelo_cmd.trq_const[0]};

    struct robif2b_kelo_power_board power_board = {
        .msr_pdo = &state.ecat_comm.pb_msr_pdo,
        .cmd_pdo = &state.ecat_comm.pb_cmd_pdo,
        .soc_msr = &state.kelo_msr.bat_lvl,
        .voltage_msr = &state.kelo_msr.bat_volt,
        .current_msr = &state.kelo_msr.bat_cur,
        .power_msr = &state.kelo_msr.bat_pwr};

    // Schedule
    robif2b_ethercat_configure(&ecat);
    if (state.ecat.error_code < 0)
        return -1;

    robif2b_ethercat_start(&ecat);
    if (state.ecat.error_code < 0)
        return -1;

    // init
    double pi = 3.14;
    double max_angle = 6.28;
    char *ptr;
    int angle = strtol(argv[1], &ptr, 10);
    // printf(fmod((2.10+pi),max_angle));
    double setpoint[4] = {0, 0, 0, 0};
    if (angle == 0)
    {
        setpoint[0] = 2.10;
        setpoint[1] = 3.47;
        setpoint[2] = 2.94;
        setpoint[3] = 0.6;
    }

    else if (angle == 90)
    {
        setpoint[0] = 2.10 + pi / 2; // 3.67
        setpoint[1] = 3.47 + pi / 2; // 5.04
        setpoint[2] = 2.94 + pi / 2; // 4.51
        setpoint[3] = 0.60 + pi / 2; // 2.17
    }
    else if (angle == 180)
    {
        setpoint[0] = fmod(2.10 + pi, max_angle); // 5.24
        setpoint[1] = fmod(3.47 + pi, max_angle);
        setpoint[2] = 2.94 + 3.14; // 2.94 + 3.14; //fmod(2.94 + pi, max_angle); //6.08
        setpoint[3] = fmod(0.60 + pi, max_angle);
    }
    else if (angle == 270)
    {
        setpoint[0] = fmod(2.10 + 1.5 * pi, max_angle);
        setpoint[1] = fmod(3.47 + 1.5 * pi, max_angle);
        setpoint[2] = fmod(2.94 + 1.5 * pi, max_angle);
        setpoint[3] = fmod(0.60 + 1.5 * pi, max_angle);
    }

    int stop_wheel_counter[4] = {0, 0, 0, 0};
    bool isAligned = false;
    int loop_counter = 0;
    double error_margin = 0.07;      // radians
    double motor_torque_value = 1.5; // Nm/A

    while (true)
    {
        if (state.ecat.error_code < 0)
            return -1;

        /*
            Align the wheels
        */

        for (int i = 0; i < NUM_DRIVES; i++)
        {
            if (isAligned == false && setpoint[i] < state.kelo_msr.pvt_pos[i] && state.kelo_msr.pvt_pos[i] < setpoint[i] + 3.14)
            {
                if (stop_wheel_counter[2] == 1 && i == 2)
                {
                    printf("!!!wheel unit 2 stopped\n");
                    state.kelo_cmd.trq[4] = 0.00;
                    state.kelo_cmd.trq[5] = 0.00;
                }
                else
                {
                    state.kelo_cmd.trq[2 * i] = motor_torque_value; // clockwise
                    state.kelo_cmd.trq[2 * i + 1] = motor_torque_value;
                    printf("!!!CLOCKWISE %d\n", i);
                }
            }
            else
            {
                if (isAligned == false)
                {
                    if (stop_wheel_counter[2] == 1 && i == 2)
                    {
                        printf("!!!wheel unit 2 stopped\n");
                        state.kelo_cmd.trq[4] = 0.00;
                        state.kelo_cmd.trq[5] = 0.00;
                    }
                    else
                    {
                        state.kelo_cmd.trq[2 * i] = -motor_torque_value; // counterclockwise
                        state.kelo_cmd.trq[2 * i + 1] = -motor_torque_value;
                        printf("!!!ANTI-CLOCKWISE %d\n", i);
                    }
                }
            }

            if (setpoint[i] - error_margin < state.kelo_msr.pvt_pos[i] && state.kelo_msr.pvt_pos[i] < setpoint[i] + error_margin)
            {
                printf("!!!wheel unit %d stopped\n", i);
                stop_wheel_counter[i] = 1;
                state.kelo_cmd.trq[2 * i] = 0.00;
                state.kelo_cmd.trq[2 * i + 1] = 0.00;
            }
            else
            {
                stop_wheel_counter[i] = 0;
                isAligned = false;
            }
        }

        // count number of ones in stop_wheel_counter
        int wheel_align_counter = 0;
        for (int i = 0; i < NUM_DRIVES; i++)
        {
            if (stop_wheel_counter[i] == 1)
            {
                wheel_align_counter++;
            }
        }

        // print number of aligned wheels
        printf("\n!!!Number of aligned wheels: %d\n", wheel_align_counter);

        loop_counter = loop_counter + 1;

        if (loop_counter < 5000)
        {
            if (stop_wheel_counter[0] == 1 && stop_wheel_counter[1] == 1 && stop_wheel_counter[2] == 1 && stop_wheel_counter[3] == 1)
            {

                printf("\n*********");
                printf("\n Platform is aligned!!!!!!");
                printf("\n*********\n\n");
                isAligned = true;
            }

            else
            {
                printf("\nSTOP WHEEL COUNTER 0: %d", stop_wheel_counter[0]);
                printf("\nSTOP WHEEL COUNTER 1: %d", stop_wheel_counter[1]);
                printf("\nSTOP WHEEL COUNTER 2: %d", stop_wheel_counter[2]);
                printf("\nSTOP WHEEL COUNTER 3: %d", stop_wheel_counter[3]);

                // print isAligned status
                printf("\n\n isAligned: %d\n", isAligned);

                printf("\n*********");
                printf("\n ** WHEELS ARE NOT ALIGNED **");
                printf("\n*********\n\n");
            }
        }
        else
        {
            printf("!!!100 iterations completed\n");
            // any of the 3 wheels are aligned, then the robot is aligned
            if (wheel_align_counter >= NUM_DRIVES - 1)
            // if (stop_wheel_counter[0] == 1 && stop_wheel_counter[1] == 1 && stop_wheel_counter[2] == 1 && stop_wheel_counter[3] == 1)
            {
                printf("!!!100 iterations completed\n");
                printf("\n*********");
                printf("\n Platform is aligned!!!!!!");
                printf("\n*********\n\n");
                isAligned = true;
            }

            else
            {
                printf("!!!100 iterations completed\n");
                printf("\nSTOP WHEEL COUNTER 0: %d", stop_wheel_counter[0]);
                printf("\nSTOP WHEEL COUNTER 1: %d", stop_wheel_counter[1]);
                printf("\nSTOP WHEEL COUNTER 2: %d", stop_wheel_counter[2]);
                printf("\nSTOP WHEEL COUNTER 3: %d", stop_wheel_counter[3]);

                // print isAligned status
                printf("\n\n isAligned: %d\n", isAligned);

                printf("\n*********");
                printf("\n ** WHEELS ARE NOT ALIGNED **");
                printf("\n*********\n\n");
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &state.time.cycle_start);
        robif2b_ethercat_update(&ecat);
        robif2b_kelo_drive_encoder_update(&drive_enc);
        robif2b_kelo_drive_imu_update(&imu);
        robif2b_kelo_drive_actuator_update(&wheel_act);
        robif2b_kelo_power_board_update(&power_board);

        // for (int i = 0; i < NUM_DRIVES; i++)
        // {

        //     printf("drive [id=%i, conn=%i]: "
        //            "w_vel[0]=%5.2f - w_vel[1]=%5.2f - p_pos=%5.2f - w_pos[0]=%5.2f - w_pos[1]=%5.2f\n",
        //            i, state.ecat.is_connected[i + 1],
        //            state.kelo_msr.whl_vel[i * 2 + 0],
        //            state.kelo_msr.whl_vel[i * 2 + 1],
        //            state.kelo_msr.pvt_pos[i],
        //            state.kelo_msr.whl_pos[i * 2 + 0],
        //            state.kelo_msr.whl_pos[i * 2 + 1]);
        //     // double pvangle[NUM_DRIVES] = state.kelo_msr.pvt_pos[i]
        // }

        // printf("power board conn=%i: "
        //        "b_volt=%5.2f - b_cur[1]=%5.2f - b_pwr=%5.2f - b_lvl=%5.2i\n",
        //        state.ecat.is_connected[0],
        //        state.kelo_msr.bat_volt,
        //        state.kelo_msr.bat_cur,
        //        state.kelo_msr.bat_pwr,
        //        state.kelo_msr.bat_lvl);
        // printf("\n");

        clock_gettime(CLOCK_MONOTONIC, &state.time.cycle_end);
        state.time.cycle_time_msr = timespec_to_usec(&state.time.cycle_end) - timespec_to_usec(&state.time.cycle_start);

        if ((state.time.cycle_time_exp - state.time.cycle_time_msr) > 0)
        {
            usleep(state.time.cycle_time_exp - state.time.cycle_time_msr);
        }
    }

    robif2b_kelo_drive_actuator_stop(&wheel_act);
    robif2b_ethercat_stop(&ecat);
    robif2b_ethercat_shutdown(&ecat);

    return 0;
}
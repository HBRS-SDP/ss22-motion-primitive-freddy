// SPDX-License-Identifier: LGPL-3.0
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <robif2b/functions/ethercat.h>
#include <robif2b/functions/kelo_drive.h>
#include <robif2b/functions/kelo_power_board.h>
#include <state_machine.h>
#include <robif2b_state.h>
#include <wheel_aligment.h>
#include <base_aligment.h>
#include <ramp.h>
#include <behaviour_connection.h>
// #define NUM_DRIVES 4
// #define NUM_SLAVES 5


struct state;
enum state_machine current_state;

static long timespec_to_usec(const struct timespec *t)
{
    const int NSEC_IN_USEC = 1000;
    const int USEC_IN_SEC = 1000000;

    return t->tv_sec * USEC_IN_SEC + t->tv_nsec / NSEC_IN_USEC;
}

//#define NUM_DRIVES 2
//#define NUM_SLAVES 3

// no need to make a source file only doing stop condition
void stop(){
    for (int i = 0; i < 8 ; i++) state.kelo_cmd.trq[i] = 0;
}

int main(int argc, char *argv[]){
// int main(){
    // Configuration
    state.num_drives = NUM_DRIVES;
    state.time.cycle_time_exp = 1000; // [us]
    state.ecat.ethernet_if = "enp61s0";
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
    double setpoint[4] = {0, 0, 0, 0};
    int stop_wheel_counter[4] = {0,0,0,0};
    current_state = STATE_WHEEL_ALIGN;

    // set point def
    if (angle == 0)
    {
        // POS X
        setpoint[0] = 2.10;
        setpoint[1] = 3.47;
        setpoint[2] = 2.94;
        setpoint[3] = 0.6;
    }
    else if (angle == 90)
    {
        // POS Y
        setpoint[0] = fmod(2.10 + pi / 2, max_angle);
        setpoint[1] = fmod(3.47 + pi / 2, max_angle);
        setpoint[2] = fmod(2.94 + pi / 2, max_angle);
        setpoint[3] = fmod(0.60 + pi / 2, max_angle);
    }
    else if (angle == 180)
    {
        // NEG X
        setpoint[0] = fmod(2.10 + pi, max_angle);
        setpoint[1] = fmod(3.47 + pi, max_angle);
        setpoint[2] = fmod(2.94 + pi, max_angle);
        setpoint[3] = fmod(0.60 + pi, max_angle);
    }
    else if (angle == 270)
    {
        // NEG Y
        setpoint[0] = fmod(2.10 + 1.5 * pi, max_angle);
        setpoint[1] = fmod(3.47 + 1.5 * pi, max_angle);
        setpoint[2] = fmod(2.94 + 1.5 * pi, max_angle);
        setpoint[3] = fmod(0.60 + 1.5 * pi, max_angle);
    }
    else angle = 0;  // restrict to 0,90,180,270 degree
    while (true)
    {
        if (state.ecat.error_code < 0)
            return -1;

    /*
        if (state_machine.current_state == START)
        {
            printf("%d", state_machine.current_state);
            state_machine.is_wheel_align = false;
            state_machine.current_state = WHEEL_ALIGN;
        }
        else
        {
            if (state_machine.is_wheel_align == false)
            {
                printf("WHEEL\n");
                wheel_alignment(angle, setpoint,stop_wheel_counter);
            }
            else if (state_machine.is_base_align == false && state_machine.is_wheel_align == true)
            {
                state_machine.current_state = BASED_ALIGN;
                printf("BASE\n");
                based_alignment();
            }
            else if (state_machine.is_base_align == true && state_machine.isStop == false)
            {
                state_machine.current_state = RAMP;
                printf("RAMP\n");
                ramp();
            }
            else if (state_machine.isStop == true)
            {
                state_machine.current_state = STOP;
                printf("STOP\n");
                stop();
            }
            else
            {
                state_machine.current_state = STOP;
                printf("STOP\n");
                stop(); // stop when the robot is in seraching mode (unidentify state)
            }
        }

        // print all motor and sensor data
        /*
        for (int i = 0; i < NUM_DRIVES; i++) {
            printf("drive [id=%i, conn=%i]: "
                    "w_vel[0]=%5.2f - w_vel[1]=%5.2f - p_pos=%5.2f ,pvt_vel=%5.2f, imu_ang_vel= %5.2f ,imu_lin_acc=%5.2f\n",
                    i, state.ecat.is_connected[i + 1],
                    state.kelo_msr.whl_vel[i * 2 + 0],
                    state.kelo_msr.whl_vel[i * 2 + 1],
                    state.kelo_msr.pvt_pos[i],
                    state.kelo_msr.pvt_vel[i],
                    state.kelo_msr.imu_ang_vel[i],
                    state.kelo_msr.imu_lin_acc[i]);
        }

        printf("power board conn=%i: "
               "b_volt=%5.2f - b_cur[1]=%5.2f - b_pwr=%5.2f - b_lvl=%5.2i\n",
                state.ecat.is_connected[0],
                state.kelo_msr.bat_volt,
                state.kelo_msr.bat_cur,
                state.kelo_msr.bat_pwr,
                state.kelo_msr.bat_lvl);
        printf("\n");
        */
       wheel_monitor(stop_wheel_counter);
       base_monitor(setpoint,stop_wheel_counter);
       //ramp_monitor();
       printf("current state: %d\n",current_state);
       switch (current_state)
       {
        case STATE_WHEEL_ALIGN:
           wheel_alignment(angle,setpoint,stop_wheel_counter);
  
           break;
        case STATE_BASE_ALIGN:
           base_alignment();

           break;
        case STATE_RAMP:
           //ramp();
           break;
        case STATE_STOP:
            stop();
            break;
       default:
            stop();
            break;
       }
       
        clock_gettime(CLOCK_MONOTONIC, &state.time.cycle_end);
        state.time.cycle_time_msr = timespec_to_usec(&state.time.cycle_end) - timespec_to_usec(&state.time.cycle_start);

        if ((state.time.cycle_time_exp - state.time.cycle_time_msr) > 0)
        {
            usleep(state.time.cycle_time_exp - state.time.cycle_time_msr);
        }


        clock_gettime(CLOCK_MONOTONIC, &state.time.cycle_start);

        robif2b_ethercat_update(&ecat);
        robif2b_kelo_drive_encoder_update(&drive_enc);
        robif2b_kelo_drive_imu_update(&imu);
        robif2b_kelo_drive_actuator_update(&wheel_act);
        robif2b_kelo_power_board_update(&power_board);
    }

    robif2b_kelo_drive_actuator_stop(&wheel_act);
    robif2b_ethercat_stop(&ecat);
    robif2b_ethercat_shutdown(&ecat);

    return 0;
}

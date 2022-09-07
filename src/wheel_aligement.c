// SPDX-License-Identifier: LGPL-3.0
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <wheel_aligment.h>
#include <state_machine.h>
#include <robif2b_state.h>


double ERROR_MARGIN = 0.07;      // radians
double MOTOR_TORQUE_VALUE = 0.8; // Nm/A (only for wheel alignment)


enum state_machine current_state;
struct state;

void wheel_alignment(double target_angle, double setpoint[4],int* stop_wheel_counter){

    for (int i = 0; i <= 4; i++) stop_wheel_counter[i] = 0;
    for (int i = 0; i < NUM_DRIVES; i++){
        if (setpoint[i] < state.kelo_msr.pvt_pos[i] && state.kelo_msr.pvt_pos[i] < setpoint[i] + 3.14){
            if (stop_wheel_counter[2] == 1 && i == 2){
                printf("!!!wheel unit 2 stopped\n");
                // state.kelo_cmd.trq[4] = 0.00;
                // state.kelo_cmd.trq[5] = 0.00;
            }
            else{
                state.kelo_cmd.trq[2 * i] = MOTOR_TORQUE_VALUE; // clockwise
                state.kelo_cmd.trq[2 * i + 1] = MOTOR_TORQUE_VALUE;
                printf("!!!CLOCKWISE %d\n", i);
            }
        }
        else{
            if (stop_wheel_counter[2] == 1 && i == 2){
                printf("!!!wheel unit 2 stopped\n");
                // state.kelo_cmd.trq[4] = 0.00;
                // state.kelo_cmd.trq[5] = 0.00;
            }
            else{
                state.kelo_cmd.trq[2 * i] = -MOTOR_TORQUE_VALUE; // counterclockwise
                state.kelo_cmd.trq[2 * i + 1] = -MOTOR_TORQUE_VALUE;
                printf("!!!ANTI-CLOCKWISE %d\n", i);
            }
        }

        if (setpoint[i] - ERROR_MARGIN < state.kelo_msr.pvt_pos[i] && state.kelo_msr.pvt_pos[i] < setpoint[i] + ERROR_MARGIN){
            printf("!!!wheel unit %d stopped\n", i);
            stop_wheel_counter[i] = 1;
            // state.kelo_cmd.trq[2 * i] = 0.00;
            // state.kelo_cmd.trq[2 * i + 1] = 0.00;
        }
        else{
            stop_wheel_counter[i] = 0;
        }
        // printf("In source: %f,%f \n",state.kelo_cmd.trq[2*i],state.kelo_cmd.trq[2*i+1]);

    }
}
void wheel_monitor(int stop_wheel_counter[4]){
    if (stop_wheel_counter[0] == 1 && stop_wheel_counter[1] == 1 && stop_wheel_counter[2] == 1 && stop_wheel_counter[3] == 1){
        printf("All wheels are aligned!!!!!!");
        current_state = STATE_BASE_ALIGN;
    }
    else current_state = STATE_WHEEL_ALIGN;
}
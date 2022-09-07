// SPDX-License-Identifier: LGPL-3.0
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <robif2b/functions/ethercat.h>
#include <robif2b/functions/kelo_drive.h>
#include <base_aligment.h>
#include <state_machine.h>
#include <robif2b_state.h>

enum state_machine current_state;
struct state;
#define BASE_SPEED 0.5
#define ERROR_MARGIN 0.07

void base_alignment(){
        state.kelo_cmd.trq[0] = -BASE_SPEED;
        state.kelo_cmd.trq[1] =  BASE_SPEED; 
        state.kelo_cmd.trq[2] = -BASE_SPEED;
        state.kelo_cmd.trq[3] =  BASE_SPEED;
        state.kelo_cmd.trq[4] = -BASE_SPEED;
        state.kelo_cmd.trq[5] =  BASE_SPEED; 
        state.kelo_cmd.trq[6] = -BASE_SPEED;
        state.kelo_cmd.trq[7] =  BASE_SPEED; 
        printf("\n Moving FORWARD!!!!! \n");
}

void base_monitor(double setpoint[4],int* stop_wheel_counter){
    //for (int i = 0; i <= 4; i++) stop_wheel_counter[i] = 0;
    if (current_state == STATE_BASE_ALIGN){
        for (int i = 0; i <= 4; i++) {
            if (setpoint[i] - ERROR_MARGIN < state.kelo_msr.pvt_pos[i] && state.kelo_msr.pvt_pos[i] < setpoint[i] + ERROR_MARGIN){
                printf("yee !!!wheel unit %d stopped\n", i);
                stop_wheel_counter[i] = 1;
            }
            else{
                stop_wheel_counter[i] = 0;
            }
        }
        if (stop_wheel_counter[0] == 1 && stop_wheel_counter[1] == 1 && stop_wheel_counter[2] == 1 && stop_wheel_counter[3] == 1){
            printf("yeee All wheels are aligned!!!!!!");
            current_state = STATE_BASE_ALIGN;
        }
        else current_state = STATE_WHEEL_ALIGN;
        
        if ((fabs(state.kelo_msr.whl_vel[0]) < 1.0) && (fabs(state.kelo_msr.whl_vel[2]) < 1.0)){
            printf("\n $$$$$$$$ Robot is aligned   $$$$$$$$$$$$$$$ \n");
            current_state = STATE_RAMP;

        }
    }
}
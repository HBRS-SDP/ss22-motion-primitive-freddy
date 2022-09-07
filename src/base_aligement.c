// SPDX-License-Identifier: LGPL-3.0
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <robif2b/functions/ethercat.h>
#include <robif2b/functions/kelo_drive.h>
#include <base_aligment.h>
#include <state_machine.h>
#include <robif2b_state.h>

enum state_machine current_state;
// struct state;


void base_alignment()
{
    if (state.ecat.error_code < 0) return -1;
    if (isAligned){

        if (inMotion == false){
                state.kelo_cmd.trq[0] = -0.3;
                state.kelo_cmd.trq[1] = 0.3; 
                state.kelo_cmd.trq[2] = -0.3;
                state.kelo_cmd.trq[3] = 0.3;
                state.kelo_cmd.trq[4] = -0.3;
                state.kelo_cmd.trq[5] = 0.3; 
                state.kelo_cmd.trq[6] = -0.3;
                state.kelo_cmd.trq[7] = 0.3; 
                printf("\n Moving FORWARD!!!!! \n");
            }

        if (fabs(state.kelo_msr.whl_vel[5]) > 2.0){
                inMotion = true;
                printf("\n Initial momentom reached!!!!! \n");
            }
        if (inMotion == true){

                printf("\n I AM STILL MOVING FORWARD!!!!! \n");

                if ((fabs(state.kelo_msr.whl_vel[0]) < 1.0)){
                    printf("\n Wheel 1 stopppppppeddddddddd!!!!! \n");
                    state.kelo_cmd.trq[0] = 0.0;
                    state.kelo_cmd.trq[1] = 0.0; 
                    state.kelo_cmd.trq[6] = 0.0;
                    state.kelo_cmd.trq[7] = 0.0; 

                } 
                if (fabs(state.kelo_msr.whl_vel[2]) < 1.0){
                    printf("\n Wheel 2 stopppppppeddddddddd!!!!! \n");
                    state.kelo_cmd.trq[2] = 0.0;
                    state.kelo_cmd.trq[3] = 0.0;
                    state.kelo_cmd.trq[4] = 0.0;
                    state.kelo_cmd.trq[5] = 0.0; 
                }

    }

}

void base_monitor(){
    /*some condition*/
<<<<<<< HEAD
    //current_state = STATE_RAMP;
    current_state = STATE_STOP;
=======
    if ((fabs(state.kelo_msr.whl_vel[0]) < 1.0) && (fabs(state.kelo_msr.whl_vel[2]) < 1.0)){
        printf("\n $$$$$$$$ Robot is aligned   $$$$$$$$$$$$$$$ \n");
            state.kelo_cmd.trq[0] = 0;
            state.kelo_cmd.trq[1] = 0; 
            state.kelo_cmd.trq[2] = 0;
            state.kelo_cmd.trq[3] = 0;
            state.kelo_cmd.trq[4] = 0;
            state.kelo_cmd.trq[5] = 0; 
            state.kelo_cmd.trq[6] = 0;
            state.kelo_cmd.trq[7] = 0;
        current_state = STATE_RAMP;

    }
>>>>>>> 30a065a76b1ca033a9c00635d2358fcc21e1d331
}
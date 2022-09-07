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
    // 0.6 in defalt
    state.kelo_cmd.trq[0] = 0.6;
    state.kelo_cmd.trq[1] = 0.6;
    state.kelo_cmd.trq[2] = 0.6;
    state.kelo_cmd.trq[3] = 0.6;
    state.kelo_cmd.trq[4] = 0.6;
    state.kelo_cmd.trq[5] = 0.6;
    state.kelo_cmd.trq[6] = 0.6;
    state.kelo_cmd.trq[7] = 0.6;

}

void base_monitor(){
    /*some condition*/
    //current_state = STATE_RAMP;
    current_state = STATE_STOP;
}
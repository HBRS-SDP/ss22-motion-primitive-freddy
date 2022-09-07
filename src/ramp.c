#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <robif2b/functions/ethercat.h>
#include <robif2b/functions/kelo_drive.h>
#include <ramp.h>
#include <state_machine.h>
#include <robif2b_state.h>

enum state_machine current_state;
struct state;

void ramp(){
    int temp =0;
}
void ramp_monitor(){

    /*some condition*/
    current_state = STATE_STOP;
}
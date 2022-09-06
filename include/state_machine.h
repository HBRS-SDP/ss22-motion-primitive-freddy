#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

enum state_machine{
    STATE_START,
    STATE_WHEEL_ALIGN,
    STATE_BASE_ALIGN,
    STATE_RAMP,
    STATE_STOP
};


extern enum state_machine current_state;

#endif
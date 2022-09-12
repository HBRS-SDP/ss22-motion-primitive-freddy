#ifndef BEHAVIOUR_CONNECTION_H
#define BEHAVIOUR_CONNECTION_H

#define NUM_DRIVES 4
#define NUM_SLAVES 5


struct behaviour_connection {
    // const indicates a read-only property
    const double *pivot_angles;
    double *wheel_commands;
    // int *flag;
};


#endif
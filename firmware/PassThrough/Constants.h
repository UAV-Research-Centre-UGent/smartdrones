#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

// General defines --------------------------------------------------------------------
#define INIT_ERROR
#define NUM_CHANNELS 6
#define SHOW_RC_OUTPUT 1

//Connection status
#define RC_DISCONNECTED 0
#define RC_CONNECTED 1
#define RC_FAILSAFE 2

//LED
#define STARTING_BLINK 100
#define ERROR_BLINK 2000
#define RUNNING_BLINK 1000

//System status
#define WAITING_FOR_DEVICES 0
#define READY 1
#define FAILSAFE 2

//Sticks configuration TEAR1234 
#define THROTTLE 0
#define PITCH 1
#define ROLL 2
#define YAW 3
#define MODES 4
#define AUX1 5
#define AUX2 6
#define AUX3 7
#define AUX4 8

// Constants-------------------------------------------------------------------
/* Min and max value from receiver */
#define MIN_SBUS_VALUE 172
#define MAX_SBUS_VALUE 1811
#define MID_SBUS_VALUE 992

#endif // __CONSTANTS_H__

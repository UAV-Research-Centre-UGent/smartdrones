#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

// General defines
// --------------------------------------------------------------------
#define INIT_ERROR

// Channels transmitted by Bluetooth
#define NUM_CHANNELS 6       // TEAR(4 CHANNELS) + AUX (2 CHANNELS)
#define BT_DATA_FRAME_SIZE 7 // DATA(6) + CRC(1)
#define SHOW_RC_OUTPUT 1
#define BT_BYTE_SIZE 5

// Connection status
#define BT_DISCONNECTED 0
#define BT_CONNECTED 1
#define BT_ERROR 2
#define RC_DISCONNECTED 0
#define RC_CONNECTED 1
#define RC_FAILSAFE 2

// MSP data
#define RSS '\r'
#define VOL '\v'
#define ALT_LIDAR '\l'
#define GYR '\g'
#define MSP_INT32_ERROR -1

// LED
#define STARTING_BLINK 100
#define ERROR_BLINK 2000
#define RUNNING_BLINK 1000

// Drone status
#define DRONE_DISARMED 0
#define DRONE_ARMED 1

// System status
#define WAITING_FOR_DEVICES 0
#define READY_TO_FLIGHT 1
#define FAILSAFE 2

// Flight modes
#define AUTONOMOUS_MODE 1
#define MANUAL_MODE 0 // PILOT TAKE THE CONTROL OF THE DRONE USING AUX3
#define FAILSAFE_MODE 2

// Sticks configuration TEAR1234
#define THROTTLE 0
#define PITCH 1
#define ROLL 2
#define YAW 3
#define AUX1 4 // ARM
#define AUX2 5 // CHANGE MODES
#define AUX3 6 // TAKE CONTROL OF THE DRONE

// Constants for BT
#define START_BT 32767 //
#define END_BT -32767  //

// Constants-------------------------------------------------------------------
/* Min and max value from receiver */
#define MIN_SBUS_VALUE 172
#define MAX_SBUS_VALUE 1811
#define MID_SBUS_VALUE 992

#endif // __CONSTANTS_H__
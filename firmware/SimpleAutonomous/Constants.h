#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

//General
#define M_PI 3.14159265358979323846

//Channels 
#define NUM_CHANNELS 16  //Channels from the receiver
#define NUM_RC_DATA 8 //T(T),P(E),R(A),Y(R),AUX1,AUX2,AUX3,AUX4
#define SHOW_RC_OUTPUT 1

//Connection status BT and RC
#define BT_DISCONNECTED 0
#define BT_CONNECTED 1
#define RC_DISCONNECTED 0
#define RC_CONNECTED 1
#define RC_FAILSAFE 2

//LED
#define STARTING_BLINK 100
#define ERROR_BLINK 2000
#define RUNNING_BLINK 1000

//Drone commands
#define STATUS_CMD  's'
#define TAKE_OFF_CMD  't'
#define LAND_CMD  'l'
#define ARM_CMD  'a'
#define DISARM_CMD  'd'
#define RC_CMD  'r'
#define NONE '~'

//Bluetooth
#define BLUETOOTH_BUFFER_SIZE 16
#define BT_END_DELIMITER '\n'
#define BT_START_DELIMITER '$'

//System status
#define WAITING_FOR_DEVICES 0
#define DRONE_READY 1
#define FAILSAFE 2

//Flight modes
#define AUTONOMOUS_MODE 1
#define MANUAL_MODE 0 //PILOT TAKE THE CONTROL OF THE DRONE USING AUX3
#define FAILSAFE_MODE 2

//Sticks configuration TEAR1234 
//Sticks configuration TEAR1234 
#define THROTTLE 0
#define PITCH 1
#define ROLL 2
#define YAW 3
#define ARM 4 //ARM
#define MODES 5 //CHANGE MODES
#define CTRL 6 //TAKE CONTROL OF THE DRONE
#define BEEP 7 //BEEPING

// Constants-------------------------------------------------------------------
/* Min and max value from receiver */
#define MIN_SBUS_VALUE 172
#define MAX_SBUS_VALUE 1811
#define MID_SBUS_VALUE 992
#define MIN_ALT 30

#endif // __CONSTANTS_H__

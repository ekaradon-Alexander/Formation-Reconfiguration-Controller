#ifndef MAIN_H
#define MAIN_H

#define BUFSIZE         1024
#define CONTROL_NUMBER  3
#define STATE_NUMBER    6
#define MAX_UAV_NUMBER  10
#define MAX_CONTROL_NUMBER      3
#define MAX_STATE_NUMBER        6
#define LOC_STATE_NUMBER        3       // location state number = 3 (x, y, z)
#define PREDICT_HORIZON         3

#include <inttypes.h>

// device ID of this UAV, it is set when connection established
extern uint8_t g_deviceID;

// number of states and controls, they are set when connection established
extern uint8_t g_nStates;
extern uint8_t g_nControls;

// number of UAVs, it is set when mission uploaded
extern uint8_t g_nDevices;

// ID of the clothest UAV, used in obstacle avoidance
extern uint8_t g_neighborID;

// predict location of all UAVs, it is sent from client before requiring for a control generation
extern float g_allPredictLocation[MAX_UAV_NUMBER][PREDICT_HORIZON][LOC_STATE_NUMBER];

// current states of this UAV
extern float g_currentState[STATE_NUMBER];

// target offset (against center of formation) location of this UAV, set when mission updated
extern float g_targetOffsetLocation[LOC_STATE_NUMBER];

// target velocy and direction of this UAV (also of the formation), set when mission updated
extern float g_targetVelocy;
extern float g_targetDirection;

// target states of this UAV
extern float g_targetState[STATE_NUMBER];

/* ---- debug switch ---- */
#define DISPLAY

#ifdef DISPLAY
//#define DISPLAY_CONNECTION
//#define DISPLAY_MISSION
//#define DISPLAY_CURR_STATE
//#define DISPLAY_PRED_LOC
//#define DISPLAY_CONTROL
#endif

#endif // MAIN_H

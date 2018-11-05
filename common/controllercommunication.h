#ifndef CONTROLLERCOMMUNICATION_H
#define CONTROLLERCOMMUNICATION_H

#include <inttypes.h>
#include <memory.h>

#define MAX_CONTROL_NUMBER      3
#define MAX_STATE_NUMBER        6
#define LOC_STATE_NUMBER        3       // location state number = 3 (x, y, z)
#define PREDICT_HORIZON         3

enum CONTROLLER_TO_CLIENT_MESSAGE_TYPE
{
    CO2CL_CONNECTION,
    CO2CL_MISSION,
    CO2CL_CONTROL_RESULT,
};

/**
 * @brief data from controller to client
 */
union ControllerToClientPayLoad
{
#ifdef __cplusplus
    ControllerToClientPayLoad() { memset(this, 0, sizeof(*this)); }
    ~ControllerToClientPayLoad() {}
#endif

    struct
    {
        uint8_t valid;      // result of controller validation, 0x01 for success
    } connectionResult;

    struct
    {
        uint8_t ack;
    } missionAck;

    struct
    {
        float controls[MAX_CONTROL_NUMBER];
        float predictLocation[PREDICT_HORIZON][LOC_STATE_NUMBER];
    } controlResult;
};

typedef struct _ControllerToClient
{
    enum CONTROLLER_TO_CLIENT_MESSAGE_TYPE type;
    uint8_t ID;
    union ControllerToClientPayLoad payLoad;
} ControllerToClient;

/**
 * @brief data from client to controller
 */

enum CLIENT_TO_CONTROLLER_MESSAGE_TYPE
{
    CL2CO_CONNECTION,
    CL2CO_MISSION,
    CL2CO_CONTROL_REQUEST,
    CL2CO_CONTROL_MSG_PRED_LOC,
    CL2CO_STOP,
};

union ClientToControllerPayLoad
{
#ifdef __cplusplus
    ClientToControllerPayLoad() { memset(this, 0, sizeof(*this)); }
    ~ClientToControllerPayLoad() {}
#endif

    struct
    {
        uint32_t clientIP;      // connection result is sent to this IP
        uint16_t clientPort;    // connection result is sent to this port
        uint8_t modelID;          // model of the device
        uint8_t nState;         // number of state
        uint8_t nControl;       // number of control
        // number of state in the CONTROLLER should be less or equal to nModelState
    } connectionRequest;

    struct
    {
        uint8_t nDevices;
        float targetOffsetLocation[LOC_STATE_NUMBER];
        float targetVelocy;
        float targetDirection;
    } missionInfo;

    struct
    {
        float currStates[MAX_STATE_NUMBER];
        float centerLoc[LOC_STATE_NUMBER];
    } controlRequest;

    struct
    {
        uint8_t deviceID;
        float predictedLocation[PREDICT_HORIZON][LOC_STATE_NUMBER];
    } uavPredictLocation;
};

typedef struct ClientToController
{
    enum CLIENT_TO_CONTROLLER_MESSAGE_TYPE type;
    uint8_t ID;
    union ClientToControllerPayLoad payLoad;
} ClientToController;


#endif // CONTROLLERCOMMUNICATION_H

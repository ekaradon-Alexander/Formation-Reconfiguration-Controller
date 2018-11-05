#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <inttypes.h>
#include <time.h>

#include "controllercommunication.h"
#include "handlers.h"
#include "main.h"

#define ERROR(msg) perror((msg));exit(1);

/* global variables */
// device ID of this UAV, it is set when connection established
uint8_t g_deviceID;

// number of states and controls, they are set when connection established
uint8_t g_nStates;
uint8_t g_nControls;

// number of UAVs, it is set when mission uploaded
uint8_t g_nDevices;

// ID of the clothest UAV, used in obstacle avoidance
uint8_t g_neighborID;

// predict location of all UAVs, it is sent from client before requiring for a control generation
float g_allPredictLocation[MAX_UAV_NUMBER][PREDICT_HORIZON][LOC_STATE_NUMBER];

// current states of this UAV
float g_currentState[STATE_NUMBER];

// current location of formation center
float g_currentCenterLocation[LOC_STATE_NUMBER];

// target offset (against center of formation) location of this UAV, set when mission updated
float g_targetOffsetLocation[LOC_STATE_NUMBER];

// target velocy and direction of this UAV (also of the formation), set when mission updated
float g_targetVelocy;
float g_targetDirection;

int main(int argc, char **argv)
{
    int sockFd;		// socket file discriber
    int portNo;		// port number to listen to
    struct sockaddr_in serverAddr;		// server (controller) address
    struct sockaddr_in clientAddr;		// client address
    unsigned int clientLen;             // client address length
    struct sockaddr_in tempClientAddr;
    unsigned int tempClientLen;
    char buf[BUFSIZE];		// message buffer
    int rcvSize;
    ClientToController *msgIn;
    ControllerToClient *msgOut = (ControllerToClient *)malloc(sizeof(ControllerToClient));

    g_deviceID = 0;
    printf("Compiled at %s %s\n", __DATE__, __TIME__);
    printf("%d\n", CLOCKS_PER_SEC);

    if (argc != 2)
    {
        fprintf(stderr, "usage: %s <port> \n", argv[0]);
        exit(-1);
    }
    else
    {
        portNo = atoi(argv[1]);
    }

    // create socket
    sockFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockFd < 0)
    {
        ERROR("ERROR opening socket");
    }

    // build the server's Internet address
    // to listen to portNo of any IP address
    bzero((char *)&serverAddr, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr.sin_port = htons((uint16_t)portNo);

    // bind
    if (bind(sockFd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
        ERROR("ERROR on binding");
    }

    // init clientAddr
    bzero((char *)&clientAddr, sizeof(clientAddr));
    clientAddr.sin_family = AF_INET;
    clientLen = sizeof(clientAddr);

    bzero((char *)&tempClientAddr, sizeof(tempClientAddr));
    tempClientAddr.sin_family = AF_INET;
    tempClientLen = sizeof(tempClientAddr);

    uint8_t (*messageHandler)(ClientToController *, ControllerToClient *,
                              int, struct sockaddr_in *);
    bool running = true;
    while (running)
    {
        bzero(buf, BUFSIZE);
        rcvSize = recvfrom(sockFd, buf, BUFSIZE, 0,
                           (struct sockaddr *)&tempClientAddr, &tempClientLen);
        if (rcvSize < 0)
        {
            ERROR("ERROR in recvfrom");
        }

        msgIn = (ClientToController *)buf;

        switch (msgIn->type)
        {
        case CL2CO_CONNECTION: messageHandler = onConnectionRecvd; break;
        case CL2CO_MISSION: messageHandler = onMissionRecvd; break;
        case CL2CO_CONTROL_REQUEST: messageHandler = onControlRequestRecvd; break;
        case CL2CO_CONTROL_MSG_PRED_LOC: messageHandler = onControlMsgPredLocRecvd; break;
        case CL2CO_STOP: running = false; break;
        }

        if (messageHandler(msgIn, msgOut, sockFd, &clientAddr) == 0)
        {
            ERROR("Error in handling message");
        }

    }

    printf("Program on exit...\n");
}

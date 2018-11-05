#include "handlers.h"
#include <float.h>

void generateControl(const float currSelfStates[],
                     const float currCenterLoc[],
                     const float targetOffsetLocation[],
                     const float targetVelocy,
                     const float targetDirection,
                     const float obstacleLoc[][LOC_STATE_NUMBER],
                     float controls[],
                     float predictLocation[][LOC_STATE_NUMBER]);

uint8_t onControlRequestRecvd(ClientToController *msgIn,
                              ControllerToClient *msgOut,
                              int sockFd,
                              struct sockaddr_in *ptr_clientAddr)
{
    uint8_t i;
    for (i = 0; i < g_nStates; i++)
    {
        g_currentState[i] = msgIn->payLoad.controlRequest.currStates[i];
    }
    for (i = 0; i < LOC_STATE_NUMBER; i++)
    {
        g_currentCenterLocation[i] = msgIn->payLoad.controlRequest.centerLoc[i];
    }

    // find the clothest neighbor
    float minDistance = FLT_MAX;
    for (i = 0; i < g_nDevices; i++)
    {
        if (i + 1 == g_deviceID) { continue; }
        float *neighborPredictLoc = g_allPredictLocation[i][0];
        float *selfPredictLoc = g_allPredictLocation[g_deviceID - 1][0];
        float distance = (neighborPredictLoc[0] - selfPredictLoc[0]) * (neighborPredictLoc[0] - selfPredictLoc[0])
                + (neighborPredictLoc[1] - selfPredictLoc[1]) * (neighborPredictLoc[1] - selfPredictLoc[1])
                + (neighborPredictLoc[2] - selfPredictLoc[2]) * (neighborPredictLoc[2] - selfPredictLoc[2]);
        if (distance < minDistance)
        {
            minDistance = distance;
            g_neighborID = i + 1;
        }
    }

    // generate control
    float controls[MAX_CONTROL_NUMBER];
    float predictLocation[PREDICT_HORIZON][LOC_STATE_NUMBER];
    generateControl(g_currentState,
                    g_currentCenterLocation,
                    g_targetOffsetLocation,
                    g_targetVelocy,
                    g_targetDirection,
                    g_allPredictLocation[g_neighborID - 1],
            controls,
            predictLocation);

#ifdef DISPLAY_CONTROL
    printf("----------------------\n");
    printf("Message Type: %d\n", msgIn->type);
    printf("Message to ID: %d\n", msgIn->ID);
    printf("with message:\n");
    printf("  current states:\n");
    for (i = 0; i < g_nStates; i++)
    {
        printf("  %g", g_currentState[i]);
    }
    printf("\n");
    printf("  center location:\n");
    for (i = 0; i < LOC_STATE_NUMBER; i++)
    {
        printf("  %g", msgIn->payLoad.controlRequest.centerLoc[i]);
    }
    printf("\n");
    printf("  return control:\n");
    for (i = 0; i < g_nControls; i++)
    {
        printf("  %g", controls[i]);
    }
    printf("\n");
#endif

    // send control
    msgOut->type = CO2CL_CONTROL_RESULT;
    msgOut->ID = g_deviceID;

    memcpy(msgOut->payLoad.controlResult.controls,
           controls,
           sizeof(float) * MAX_CONTROL_NUMBER);
    memcpy(msgOut->payLoad.controlResult.predictLocation,
           predictLocation,
           sizeof(float) * PREDICT_HORIZON * LOC_STATE_NUMBER);

    uint clientLen = sizeof(*ptr_clientAddr);

    if (0 > sendto(sockFd, (char *)msgOut, sizeof(ControllerToClient), 0,
                   (struct sockaddr *)ptr_clientAddr, clientLen))
    {
        return 0;
    }
    else
    {
        return 1;
    }

    return 1;
}

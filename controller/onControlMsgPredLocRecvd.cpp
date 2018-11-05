#include "handlers.h"

uint8_t onControlMsgPredLocRecvd(ClientToController *msgIn,
                                 ControllerToClient *msgOut,
                                 int sockFd,
                                 struct sockaddr_in *ptr_clientAddr)
{
    uint8_t i, j;
    uint8_t ID = msgIn->payLoad.uavPredictLocation.deviceID;

    for (i = 0; i < PREDICT_HORIZON; i++)
    {
        for (j = 0; j < LOC_STATE_NUMBER; j++)
        {
            g_allPredictLocation[ID-1][i][j] =
                    msgIn->payLoad.uavPredictLocation.predictedLocation[i][j];
        }
    }

#ifdef DISPLAY_PRED_LOC
    printf("----------------------\n");
    printf("Message Type: %d\n", msgIn->type);
    printf("Message to ID: %d\n", msgIn->ID);
    printf("with message:\n");
    printf("  predict loc of UAV %d:\n", ID);
    for (i = 0; i < PREDICT_HORIZON; i++)
    {
        printf("  (%g, %g, %g)  ",
               g_allPredictLocation[ID-1][i][0], g_allPredictLocation[ID-1][i][1], g_allPredictLocation[ID-1][i][2]);
    }
    printf("\n");
#endif

    return 1;
}

#include "handlers.h"

uint8_t onMissionRecvd(ClientToController *msgIn,
                       ControllerToClient *msgOut,
                       int sockFd,
                       struct sockaddr_in *ptr_clientAddr)
{
    uint8_t i;
    uint clientLen = sizeof(*ptr_clientAddr);

    g_nDevices = msgIn->payLoad.missionInfo.nDevices;
    for (i = 0; i < LOC_STATE_NUMBER; i++)
    {
        g_targetOffsetLocation[i] = msgIn->payLoad.missionInfo.targetOffsetLocation[i];
    }
    g_targetVelocy = msgIn->payLoad.missionInfo.targetVelocy;
    g_targetDirection = msgIn->payLoad.missionInfo.targetDirection;

#ifdef DISPLAY_MISSION
    printf("----------------------\n");
    printf("Message Type: %d\n", msgIn->type);
    printf("Message to ID: %d\n", msgIn->ID);
    printf("with message:\n");
    printf("  #devices: %d\n", g_nDevices);
    printf("  target offset loc: ");
    for (i = 0; i < LOC_STATE_NUMBER; i++)
    {
        printf("  %g  ", g_targetOffsetLocation[i]);
    }
    printf("\n");
    printf("  target velocy: %g, direction: %g\n", g_targetVelocy, g_targetDirection);
    printf("\n");
#endif

    msgOut->type = CO2CL_MISSION;
    msgOut->ID = g_deviceID;

    msgOut->payLoad.missionAck.ack = 1;

    if (0 > sendto(sockFd, (char *)msgOut, sizeof(ControllerToClient), 0,
                   (struct sockaddr *)ptr_clientAddr, clientLen))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

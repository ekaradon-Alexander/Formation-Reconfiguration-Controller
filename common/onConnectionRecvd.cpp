#include "handlers.h"

uint8_t onConnectionRecvd(ClientToController *msgIn,
                          ControllerToClient *msgOut,
                          int sockFd,
                          struct sockaddr_in *ptr_clientAddr)
{
#ifdef DISPLAY_CONNECTION
    printf("----------------------\n");
    printf("Message Type: %d\n", msgIn->type);
    printf("Message to ID: %d\n", msgIn->ID);
    printf("with message:\n");
    printf("  client address: %d\n", msgIn->payLoad.connectionRequest.clientIP);
    printf("  client port: %d\n", msgIn->payLoad.connectionRequest.clientPort);
    printf("  model: %d\n", msgIn->payLoad.connectionRequest.modelID);
    printf("  #state: %d\n", msgIn->payLoad.connectionRequest.nModelState);
    printf("  #control: %d\n", msgIn->payLoad.connectionRequest.nModelControl);
#endif

    msgOut->type = CO2CL_CONNECTION;
    msgOut->ID = msgIn->ID;

    if (g_deviceID == 0)      // initial connection
    {
        if (msgIn->payLoad.connectionRequest.nState >= STATE_NUMBER &&
                msgIn->payLoad.connectionRequest.nControl == CONTROL_NUMBER)
        {
            msgOut->payLoad.connectionResult.valid = 1;
            g_deviceID = msgIn->ID;
            g_nStates = msgIn->payLoad.connectionRequest.nState;
            g_nControls = msgIn->payLoad.connectionRequest.nControl;
        }
        else
        {
            msgOut->payLoad.connectionResult.valid = 0;
        }
    }
    else        // reconnection
    {
        if (msgIn->ID == g_deviceID)
        {
            msgOut->payLoad.connectionResult.valid = 1;
            g_nStates = msgIn->payLoad.connectionRequest.nState;
            g_nControls = msgIn->payLoad.connectionRequest.nControl;
        }
        else
        {
            msgOut->payLoad.connectionResult.valid = 0;
        }
    }

    ptr_clientAddr->sin_port = htons(msgIn->payLoad.connectionRequest.clientPort);
    ptr_clientAddr->sin_addr.s_addr = htonl(msgIn->payLoad.connectionRequest.clientIP);

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
}

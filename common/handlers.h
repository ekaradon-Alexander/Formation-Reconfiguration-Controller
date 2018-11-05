#ifndef HANDLERS_H
#define HANDLERS_H

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <inttypes.h>

#include "controllercommunication.h"
#include "main.h"

uint8_t onConnectionRecvd(ClientToController *msgIn,
                          ControllerToClient *msgOut,
                          int sockFd,
                          struct sockaddr_in *ptr_clientAddr);

uint8_t onMissionRecvd(ClientToController *msgIn,
                       ControllerToClient *msgOut,
                       int sockFd,
                       struct sockaddr_in *ptr_clientAddr);

uint8_t onControlRequestRecvd(ClientToController *msgIn,
                              ControllerToClient *msgOut,
                              int sockFd,
                              struct sockaddr_in *ptr_clientAddr);

uint8_t onControlMsgPredLocRecvd(ClientToController *msgIn,
                                 ControllerToClient *msgOut,
                                 int sockFd,
                                 struct sockaddr_in *ptr_clientAddr);

#endif // HANDLERS_H

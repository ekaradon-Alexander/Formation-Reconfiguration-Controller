#include <iostream>
#include <math.h>

#include "main.h"
#include <float.h>
#include <time.h>

using namespace std;

#define STATE_NUMBER        6
#define CONTROL_NUMBER      3
const float freq = 0.5;

float pastTime = 0.0;
float addTime;
float rz;


void SQPCalcControl(const float currSelfStates[],
                    const float futureTargetStates[][STATE_NUMBER],
                    const float obstacleLoc[][LOC_STATE_NUMBER],
                    float controls[],
                    float predictLocation[][LOC_STATE_NUMBER]);

void generateControl(const float currSelfStates[],
                     const float currCenterLoc[],
                     const float targetOffsetLocation[],
                     const float targetVelocy,
                     const float targetDirection,
                     const float obstacleLoc[][LOC_STATE_NUMBER],
                     float controls[],
                     float predictLocation[][LOC_STATE_NUMBER])
{
    /* calculate target states in the future horizons */
    float futureTargetStates[PREDICT_HORIZON][STATE_NUMBER];
    rz = 20.0 * ((float)g_deviceID - (0.5 * g_nDevices));

    pastTime += 0.13;
    addTime = (pastTime > 4.0) ? (4.0) : (pastTime);

    for (uint8_t i = 0; i < PREDICT_HORIZON; i++)
    {
        // target X location
        futureTargetStates[i][0] = currCenterLoc[0] + targetVelocy * (i + 1) * freq * cos(targetDirection) +
                targetOffsetLocation[0];
        // target Y location
        futureTargetStates[i][1] = currCenterLoc[1] + targetVelocy * (i + 1) * freq * sin(targetDirection) +
                targetOffsetLocation[1];
        // target Z location
        futureTargetStates[i][2] = targetOffsetLocation[2] + (4.0 - addTime) * rz / 4.0;
        // target psi
        futureTargetStates[i][3] = targetDirection;
        // target theta
        futureTargetStates[i][4] = 0;
        // target v
        futureTargetStates[i][5] = targetVelocy;
    }
    printf("%f\n", futureTargetStates[0][2]);


    // long start = clock();

    SQPCalcControl(currSelfStates,
                   futureTargetStates,
                   obstacleLoc,
                   controls,
                   predictLocation);
    // printf("%lu\n", clock() - start);
}

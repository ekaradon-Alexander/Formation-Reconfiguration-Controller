#include <iostream>
#include <math.h>

#include "main.h"
#include <time.h>

using namespace std;

#define STATE_NUMBER        6
#define CONTROL_NUMBER      3
const float freq = 0.5;

float pastTime = 0.0;
float addTime;

float rz = 80.0 * ((float)g_deviceID - (0.5 * g_nDevices));

void PSOCalcControl(const float currSelfStates[],
                    const float futureTargetStates[][STATE_NUMBER],
                    const float obstacleLoc[][LOC_STATE_NUMBER],
                    float controls[],
                    float predictLocation[][LOC_STATE_NUMBER]);

//void SQPCalcControl(const float currSelfStates[],
//                    const float futureTargetStates[][STATE_NUMBER],
//                    float controls[],
//                    float predictLocation[][LOC_STATE_NUMBER]);

void generateControl(const float currSelfStates[],
                     const float currCenterLoc[],
                     const float targetOffsetLocation[],
                     const float targetVelocy,
                     const float targetDirection,
                     const float obstacleLoc[][LOC_STATE_NUMBER],
                     float controls[],
                     float predictLocation[][LOC_STATE_NUMBER])
{
    pastTime += 0.13;
    addTime = (pastTime > 13.0) ? (13.0) : (pastTime);

    /* calculate target states in the future horizons */
    float futureTargetStates[PREDICT_HORIZON][STATE_NUMBER];
    for (uint8_t i = 0; i < PREDICT_HORIZON; i++)
    {
        // target X location
        futureTargetStates[i][0] = currCenterLoc[0] + targetVelocy * (i + 1) * freq * cos(targetDirection) +
                targetOffsetLocation[0];
        // target Y location
        futureTargetStates[i][1] = currCenterLoc[1] + targetVelocy * (i + 1) * freq * sin(targetDirection) +
                targetOffsetLocation[1];
        // target Z location
        futureTargetStates[i][2] = targetOffsetLocation[2] + (33.0 - addTime) * rz;
        // target psi
        futureTargetStates[i][3] = targetDirection;
        // target theta
        futureTargetStates[i][4] = 0;
        // target v
        futureTargetStates[i][5] = targetVelocy;

//        cout << "target (horizon " << i << ")  " <<
//                futureTargetStates[i][0] << "  " << futureTargetStates[i][1] << "  " <<
//                futureTargetStates[i][2] << "  " << futureTargetStates[i][3] << "  " <<
//                futureTargetStates[i][4] << "  " << futureTargetStates[i][5] << endl;
    }

    long start = clock();

    PSOCalcControl(currSelfStates,
                   futureTargetStates,
                   obstacleLoc,
                   controls,
                   predictLocation);
    printf("%lu\n", clock() - start);
//    cout << "controls = " << controls[0] << "    " << controls[1] << "    " << controls[2] << endl;
}

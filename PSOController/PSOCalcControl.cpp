#include "main.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <memory.h>
#include <float.h>

const float dt = 0.05;           // delta t in model
const float freq = 0.5;         // freq of control

#define PARTICLE_NUMBER     50
#define ITER_NUMBER         60

extern float pastTime;
extern float addTime;

float weights[PREDICT_HORIZON][STATE_NUMBER] = {
    {   15.0,    15.0,    15.0,    900.0,        100.0,      10.0    },
    {   13.0,    13.0,    13.0,    900.0,        100.0,      10.0     },
    {   11.0,    11.0,    11.0,    900.0,        100.0,      10.0     },
};
float weights1[PREDICT_HORIZON][STATE_NUMBER] = {
    {   5.0,    5.0,    5.0,    90000.0,        100.0,      10.0    },
    {   3.0,    3.0,    3.0,    90000.0,        100.0,      10.0     },
    {   1.0,    1.0,    1.0,    90000.0,        100.0,      10.0     },
};

void greedyUpdate(const float currSelfStates[],
                  const float futureTargetStates[][STATE_NUMBER],
                  const float particleNextLocation[][CONTROL_NUMBER],
                  const float obstacleLoc[][LOC_STATE_NUMBER],
                  float *personalBestValue,
                  float particleCurrLocation[][CONTROL_NUMBER],
                  float personalBestLocation[][CONTROL_NUMBER]);
float evaluateParticle(const float currSelfStates[],
                       const float futureTargetStates[][STATE_NUMBER],
                       const float particleNextLocation[][CONTROL_NUMBER],
                       const float obstacleLoc[][LOC_STATE_NUMBER]);
uint8_t simulate(float initStates[],
                 const float controls[],
                 uint8_t nStep);
void updateMovingDirection(const float groupBestLocation[][CONTROL_NUMBER],
                           const float personalBestLocation[][CONTROL_NUMBER],
                           const float particleCurrLocation[][CONTROL_NUMBER],
                           float particleVelocy[][CONTROL_NUMBER],
                           float particleNextLocation[][CONTROL_NUMBER]);

void PSOCalcControl(const float currSelfStates[],
                    const float futureTargetStates[][STATE_NUMBER],
                    const float obstacleLoc[][LOC_STATE_NUMBER],
                    float controls[],
                    float predictLocation[][LOC_STATE_NUMBER])
{
    // here `location' and `velocy' are both in the term of PSO algorithm.
    float particlesCurrLocation[PARTICLE_NUMBER][PREDICT_HORIZON][CONTROL_NUMBER];
    float particlesNextLocation[PARTICLE_NUMBER][PREDICT_HORIZON][CONTROL_NUMBER];

    float groupBestLocation[PREDICT_HORIZON][CONTROL_NUMBER];
    float groupBestValue = FLT_MAX;

    float personalBestLocation[PARTICLE_NUMBER][PREDICT_HORIZON][CONTROL_NUMBER];
    float personalBestValue[PARTICLE_NUMBER];

    float particlesVelocy[PARTICLE_NUMBER][PREDICT_HORIZON][CONTROL_NUMBER];

    // randomly initial particles location
    for (uint8_t i = 0; i < PARTICLE_NUMBER; i++)
    {
        for (uint8_t j = 0; j < PREDICT_HORIZON; j++)
        {
            for (uint8_t k = 0; k < CONTROL_NUMBER; k++)
            {
                float temp = static_cast<float>(rand()) / RAND_MAX;
                temp = 10.0 * (temp - 0.5);
                particlesCurrLocation[i][j][k] = temp;
                particlesNextLocation[i][j][k] = temp;
            }
        }
        personalBestValue[i] = FLT_MAX;
    }

    // set initial velocy 0
    memset(particlesVelocy, 0, sizeof(float) * PARTICLE_NUMBER * PREDICT_HORIZON * CONTROL_NUMBER);

    if (pastTime > 1.0)
    {
        weights[0][2] = 15.0;
        weights[1][2] = 13.0;
        weights[2][2] = 11.0;

    }
    if (pastTime > 2.0 && pastTime < 3.5)
    {
        weights1[0][3] -= 8000;
        weights1[1][3] -= 8000;
        weights1[2][3] -= 8000;
    }

    // main iteration
    for (uint8_t iter = 0; iter < ITER_NUMBER; iter++)
    {
        // evaluate particles' next location and greddy update
        for (uint8_t i = 0; i < PARTICLE_NUMBER; i++)
        {
            greedyUpdate(currSelfStates,
                         futureTargetStates,
                         particlesNextLocation[i],
                         obstacleLoc,
                         &personalBestValue[i],
                         particlesCurrLocation[i],
                         personalBestLocation[i]);
        }

        // update group best
        for (uint8_t i = 0; i < PARTICLE_NUMBER; i++)
        {
            float temp = personalBestValue[i];
            if (temp < groupBestValue)
            {
                groupBestValue = temp;
                memcpy(groupBestLocation,
                       particlesCurrLocation[i],
                       sizeof(float) * PREDICT_HORIZON * CONTROL_NUMBER);
            }
        }

        // generate particle moving direction
        for (uint8_t i = 0; i < PARTICLE_NUMBER; i++)
        {
            updateMovingDirection(groupBestLocation,
                                  personalBestLocation[i],
                                  particlesCurrLocation[i],
                                  particlesVelocy[i],
                                  particlesNextLocation[i]);
        }
    }

    memcpy(controls, groupBestLocation[0], sizeof(float) * CONTROL_NUMBER);

    float simStates[STATE_NUMBER];
    memcpy(simStates, currSelfStates, sizeof(float) * STATE_NUMBER);
    for (uint8_t i = 0; i < PREDICT_HORIZON; i++)
    {
        simulate(simStates, groupBestLocation[i], static_cast<uint8_t>(freq / dt));
        predictLocation[i][0] = simStates[0];
        predictLocation[i][1] = simStates[1];
        predictLocation[i][2] = simStates[2];
    }
}

void greedyUpdate(const float currSelfStates[],
                  const float futureTargetStates[][STATE_NUMBER],
                  const float particleNextLocation[][CONTROL_NUMBER],
                  const float obstacleLoc[][LOC_STATE_NUMBER],
                  float *personalBestValue,
                  float particleCurrLocation[][CONTROL_NUMBER],
                  float personalBestLocation[][CONTROL_NUMBER])
{
    float temp = evaluateParticle(currSelfStates,
                                  futureTargetStates,
                                  particleNextLocation,
                                  obstacleLoc);
    if (temp < *personalBestValue)
    {
        *personalBestValue = temp;
        memcpy(personalBestLocation, particleNextLocation,
               sizeof(float) * PREDICT_HORIZON * CONTROL_NUMBER);
        memcpy(particleCurrLocation, particleNextLocation,
               sizeof(float) * PREDICT_HORIZON * CONTROL_NUMBER);
    }
}

float evaluateParticle(const float currSelfStates[],
                       const float futureTargetStates[][STATE_NUMBER],
                       const float particleNextLocation[][CONTROL_NUMBER],
                       const float obstacleLoc[][LOC_STATE_NUMBER])
{
    float punish = 0.0;
    float result = 0.0;
    float simStates[STATE_NUMBER];
    float horizonSimStates[PREDICT_HORIZON][STATE_NUMBER];
    memcpy(simStates, currSelfStates, sizeof(float) * STATE_NUMBER);

    // do simulation
    for (uint8_t i = 0; i < PREDICT_HORIZON; i++)
    {
        simulate(simStates, particleNextLocation[i], static_cast<uint8_t>(freq / dt));
        memcpy(horizonSimStates[i], simStates, sizeof(float) * STATE_NUMBER);
    }

    // evaluate particle based on the difference between simulated states and target states
    float dist1 = (horizonSimStates[0][0] - obstacleLoc[0][0]) * (horizonSimStates[0][0] - obstacleLoc[0][0]) +
            (horizonSimStates[0][1] - obstacleLoc[0][1]) * (horizonSimStates[0][1] - obstacleLoc[0][1]) +
            (horizonSimStates[0][2] - obstacleLoc[0][2]) * (horizonSimStates[0][2] - obstacleLoc[0][2]);
    float dist2 = (horizonSimStates[1][0] - obstacleLoc[1][0]) * (horizonSimStates[1][0] - obstacleLoc[1][0]) +
            (horizonSimStates[1][1] - obstacleLoc[1][1]) * (horizonSimStates[1][1] - obstacleLoc[1][1]) +
            (horizonSimStates[1][2] - obstacleLoc[1][2]) * (horizonSimStates[1][2] - obstacleLoc[1][2]);

    if (dist1 < 400.0)
    {
        result = FLT_MAX;
        return result;
    }
    if (dist2 < 400.0)
    {
        result = FLT_MAX;
        return result;
    }

    for (uint8_t i = 0; i < PREDICT_HORIZON; i++)
    {
        // for (uint8_t j = 0; j < STATE_NUMBER; j++)
        // {
        //     result += (horizonSimStates[i][j] - futureTargetStates[i][j]) * weights[i][j] *
        //             (horizonSimStates[i][j] - futureTargetStates[i][j]);
        // }
        // result += (horizonSimStates[i][2] - futureTargetStates[i][2]) * addTime *
        //         (horizonSimStates[i][2] - futureTargetStates[i][2]);
        // result += (horizonSimStates[i][4] - futureTargetStates[i][4]) * addTime * 5 *
        //         (horizonSimStates[i][4] - futureTargetStates[i][4]);
        if (pastTime > 2.0 && pastTime < 4)
        {
            for (uint8_t j = 0; j < STATE_NUMBER; j++)
            {
                result += (horizonSimStates[i][j] - futureTargetStates[i][j]) * weights1[i][j] *
                        (horizonSimStates[i][j] - futureTargetStates[i][j]);
            }
        }
        else
        {
            for (uint8_t j = 0; j < STATE_NUMBER; j++)
            {
                result += (horizonSimStates[i][j] - futureTargetStates[i][j]) * weights[i][j] *
                        (horizonSimStates[i][j] - futureTargetStates[i][j]);
            }
        }
    }

    for (uint8_t i = 0; i < PREDICT_HORIZON; i++)
    {
        for (uint8_t j = 0; j < CONTROL_NUMBER; j++)
        {
            result += particleNextLocation[i][j] * 30.0 * particleNextLocation[i][j];
        }
    }

    return result;
}

uint8_t simulate(float initStates[],
                 const float controls[],
                 uint8_t nStep)
{
#define MIN_SPEED   10
#define MAX_SPEED   20

    float x = initStates[0];
    float y = initStates[1];
    float z = initStates[2];
    float p = initStates[3];
    float t = initStates[4];
    float v = initStates[5];

    float a = controls[0];
    float u = controls[1];
    float w = controls[2];

    uint8_t result = 0;
    for (uint8_t i = 0; i < nStep; i++)
    {
        p += u * dt / v;
        t += w * dt / v;
        v += a * dt;
        if (v < MIN_SPEED) { result = 1; }
        else if (v > MAX_SPEED) { result = 2; }

        x += v * dt * cos(p) * cos(t);
        y += v * dt * sin(p) * cos(t);
        z += v * dt * sin(t);
    }

    initStates[0] = x;
    initStates[1] = y;
    initStates[2] = z;
    initStates[3] = p;
    initStates[4] = t;
    initStates[5] = v;

    return result;
}

void updateMovingDirection(const float groupBestLocation[][CONTROL_NUMBER],
                           const float personalBestLocation[][CONTROL_NUMBER],
                           const float particleCurrLocation[][CONTROL_NUMBER],
                           float particleVelocy[][CONTROL_NUMBER],
                           float particleNextLocation[][CONTROL_NUMBER])
{
    for (uint8_t i = 0; i < PREDICT_HORIZON; i++)
    {
        for (uint8_t j = 0; j < CONTROL_NUMBER; j++)
        {
            particleVelocy[i][j] += (
                        0.05 * particleVelocy[i][j] +
                        0.2 * (personalBestLocation[i][j] - particleCurrLocation[i][j]) +
                        0.75 * (groupBestLocation[i][j] - particleCurrLocation[i][j]) );
            particleNextLocation[i][j] = particleCurrLocation[i][j] + particleVelocy[i][j];
        }
    }
}

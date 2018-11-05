#include "main.h"

#include "QuadProg.h"
#include "problem.h"
#include "Array.h"
using namespace QuadProg;

#include <iostream>
using namespace std;

#include <inttypes.h>

double dt = 0.5;           // delta t in model
double freq = 0.5;         // freq of control
extern float pastTime;

#define VARIABLES_NUMBER        ((CONTROL_NUMBER + STATE_NUMBER) * PREDICT_HORIZON)
#define EQCONS_NUMBER           (STATE_NUMBER * PREDICT_HORIZON)
#define NECONS_NUMBER           1

#define ITER_NUMBER             20

float initStates[STATE_NUMBER];
float refStates[PREDICT_HORIZON][STATE_NUMBER];
float obstacle[PREDICT_HORIZON][LOC_STATE_NUMBER];

float stateWeight[][STATE_NUMBER] = {
    {   15.0,    15.0,    15.0,    900.0,        100.0,      10.0    },
    {   13.0,    13.0,    13.0,    900.0,        100.0,      10.0     },
    {   11.0,    11.0,    11.0,    900.0,        100.0,      10.0     },
};

float controlWeight[][CONTROL_NUMBER] = {
    {   100.0,      100.0,      100.0},
    {   100.0,      100.0,      100.0},
    {   100.0,      100.0,      100.0},
};

bool isNan(double d)
{
    if (d == d) { return false; }
    else { return true; }
}

void simulate(float initStates[],
              const float controls[],
              uint8_t nStep);

void SQPCalcControl(const float currSelfStates[],
                    const float futureTargetStates[][STATE_NUMBER],
                    const float obstacleLoc[][LOC_STATE_NUMBER],
                    float controls[],
                    float predictLocation[][LOC_STATE_NUMBER])
{
    Vector<double> c;            // gradient of obj function
    Matrix<double> Q;            // Hessian of obj function

    Matrix<double> CE, CI;       // Jacobian of eq and ineq constraints
    Vector<double> ce0, ci0;     // constant term of eq and ineq constraints

    Vector<double> d;            // optimal solution of QP
    Vector<double> x;
    //        0   1   2   3   4   5   6   7   8     9    10    11      12      13      14  15  16  17  18  19  20  21  22  23  24  25  26
    // x = [ x1, x2, x3, y1, y2, y3, z1, z2, z3, psi1, psi2, psi3, theta1, theta2, theta3, v1, v2, v3, a1, a2, a3, u1, u2, u3, w1, w2, w3 ]
    Vector<double> xx;           // xx = x + d

    c.resize(VARIABLES_NUMBER);
    Q.resize(VARIABLES_NUMBER, VARIABLES_NUMBER);
    CE.resize(VARIABLES_NUMBER, EQCONS_NUMBER);
    CI.resize(VARIABLES_NUMBER, NECONS_NUMBER);
    ce0.resize(EQCONS_NUMBER);
    ci0.resize(NECONS_NUMBER);

    d.resize(VARIABLES_NUMBER);
    x.resize(VARIABLES_NUMBER);
    xx.resize(VARIABLES_NUMBER);

    double err = 10.0;
    double epsilon = 0.001;
    double obj;

    // assign init value of x as currSelfStates[]
    for (uint8_t i = 0; i < PREDICT_HORIZON; i++)
    {
        for (uint8_t j = 0; j < STATE_NUMBER; j++)
        {
            x[PREDICT_HORIZON * j + i] = currSelfStates[j];
        }
    }
    // assign initial and reference states
    memcpy(initStates, currSelfStates, sizeof(float) * STATE_NUMBER);
    memcpy(refStates, futureTargetStates, sizeof(float) * PREDICT_HORIZON * STATE_NUMBER);
    memcpy(obstacle, obstacleLoc, sizeof(float) * PREDICT_HORIZON * LOC_STATE_NUMBER);

    if (pastTime > 3.0 && pastTime < 4.5)
    {
        stateWeight[0][3] = 10000;
        stateWeight[1][3] = 10000;
        stateWeight[2][3] = 10000;
    }
    else if (pastTime > 4.5 && pastTime < 8.0)
    {
        stateWeight[0][3] -= 200;
        stateWeight[1][3] -= 200;
        stateWeight[2][3] -= 200;
    }
    else
    {
        stateWeight[0][3] = 300;
        stateWeight[1][3] = 300;
        stateWeight[2][3] = 300;
    }

    int iter = 0;
    while (err > epsilon && iter < ITER_NUMBER)
    {
        iter++;

        Hessian(x, Q);
        Gradient(x, c);

        GradientCE(x, CE);
        CE0(x, ce0);

        GradientCI(x, CI);
        CI0(x, ci0);

        try
        {
            solve_quadprog(Q, c, CE, ce0, CI, ci0, d);
        }
        catch (...)
        {
            controls[0] = 0.0;
            controls[1] = 0.0;
            controls[2] = 0.0;
            goto onExit;
        }
        // obj = objFcn(x);

        err = 0.0;
        for (uint8_t i = 0; i < VARIABLES_NUMBER; i++)
        {
            x[i] += d[i];
            err += d[i] * d[i];
        }

        err = pow(err, 0.5);
    }

    controls[0] = isNan(x[18]) ? 0.0 : x[18];
    controls[1] = isNan(x[21]) ? 0.0 : x[21];
    controls[2] = isNan(x[24]) ? 0.0 : x[24];

onExit:
    float tempStates[STATE_NUMBER];
    memcpy(tempStates, currSelfStates, sizeof(float) * STATE_NUMBER);
    simulate(tempStates, controls, 1);
    memcpy(predictLocation[0], tempStates, sizeof(float) * STATE_NUMBER);
}

void simulate(float initStates[],
              const float controls[],
              uint8_t nStep)
{
    float x = initStates[0];
    float y = initStates[1];
    float z = initStates[2];
    float p = initStates[3];
    float t = initStates[4];
    float v = initStates[5];

    float a = controls[0];
    float u = controls[1];
    float w = controls[2];

    for (uint8_t i = 0; i < nStep; i++)
    {
        p += u * dt / v;
        t += w * dt / v;
        v += a * dt;

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
}

#include "problem.h"
#include "main.h"

extern double dt;
extern float initStates[STATE_NUMBER];
extern float refStates[][STATE_NUMBER];
extern float obstacle[][LOC_STATE_NUMBER];
extern float stateWeight[][STATE_NUMBER];
extern float controlWeight[][CONTROL_NUMBER];

#define x1      x[0]
#define x2      x[1]
#define x3      x[2]
#define y1      x[3]
#define y2      x[4]
#define y3      x[5]
#define z1      x[6]
#define z2      x[7]
#define z3      x[8]
#define p1      x[9]
#define p2      x[10]
#define p3      x[11]
#define t1      x[12]
#define t2      x[13]
#define t3      x[14]
#define v1      x[15]
#define v2      x[16]
#define v3      x[17]
#define a1      x[18]
#define a2      x[19]
#define a3      x[20]
#define u1      x[21]
#define u2      x[22]
#define u3      x[23]
#define w1      x[24]
#define w2      x[25]
#define w3      x[26]

#define x0      initStates[0]
#define y0      initStates[1]
#define z0      initStates[2]
#define p0      initStates[3]
#define t0      initStates[4]
#define v0      initStates[5]

#define xo1     obstacle[0][0]
#define yo1     obstacle[0][1]
#define zo1     obstacle[0][2]
#define xo2     obstacle[1][0]
#define yo2     obstacle[1][1]
#define zo2     obstacle[1][2]
// Objective fcn
double objFcn(Vector<double> x)
{
    double f = 0.0;

    for (uint8_t i = 0; i < PREDICT_HORIZON; i++)
    {
        for (uint8_t j = 0; j < STATE_NUMBER; j++)
        {
            f += (x[PREDICT_HORIZON * j + i] - refStates[i][j]) * stateWeight[i][j]
                    * (x[PREDICT_HORIZON * j + i] - refStates[i][j]);
        }
        for (uint8_t j = 0; j < CONTROL_NUMBER; j++)
        {
            f += (x[PREDICT_HORIZON * (j + STATE_NUMBER) + i]) * controlWeight[i][j]
                    * (x[PREDICT_HORIZON * (j + STATE_NUMBER) + i]);
        }
    }

    return f;
}

// Gradient of objective fcn
void Gradient(Vector<double> x, Vector<double> &c)
{
    for (uint8_t i = 0; i < PREDICT_HORIZON; i++)
    {
        for (uint8_t j = 0; j < STATE_NUMBER; j++)
        {
            c[PREDICT_HORIZON * j + i] =
                    2.0 * stateWeight[i][j] * (x[PREDICT_HORIZON * j + i] - refStates[i][j]);
        }
        for (uint8_t j = 0; j < CONTROL_NUMBER; j++)
        {
            c[PREDICT_HORIZON * (j + STATE_NUMBER) + i] =
                    2.0 * controlWeight[i][j] * x[PREDICT_HORIZON * (j + STATE_NUMBER) + i];
        }
    }
}

// Hessian matrix of objective fcn
void Hessian(Vector<double> x, Matrix<double> &Q)
{
    for (uint8_t i = 0; i < VARIABLES_NUMBER; i++)
    {
        for (uint8_t j = 0; j < VARIABLES_NUMBER; j++)
        {
            Q[i][j] = 0;
        }
    }

    for (uint8_t i = 0; i < PREDICT_HORIZON; i++)
    {
        for (uint8_t j = 0; j < STATE_NUMBER; j++)
        {
            Q[PREDICT_HORIZON * j + i][PREDICT_HORIZON * j + i]
                    = 2.0 * stateWeight[i][j];
        }
        for (uint8_t j = 0; j < CONTROL_NUMBER; j++)
        {
            Q[PREDICT_HORIZON * (j + STATE_NUMBER) + i][PREDICT_HORIZON * (j + STATE_NUMBER) + i]
                    = 2.0 * controlWeight[i][j];
        }
    }
}

// Gradient of equation constraints
void GradientCE(Vector<double> x, Matrix<double> &CE)
{
    for (uint8_t i = 0; i < VARIABLES_NUMBER; i++)
    {
        for (uint8_t j = 0; j < EQCONS_NUMBER; j++)
        {
            CE[i][j] = 0;
        }
    }
    
    if (v0 < 10) { v0 = 10; }
    if (v1 < 10) { v1 = 10; }
    if (v2 < 10) { v2 = 10; }
    if (v3 < 10) { v3 = 10; }

    CE[0][3] = 1.0;
    CE[0][9] = -1.0;

    CE[1][9] = 1.0;
    CE[1][15] = -1.0;

    CE[2][15] = 1.0;

    CE[3][4] = 1.0;
    CE[3][10] = -1.0;

    CE[4][10] = 1.0;
    CE[4][16] = -1.0;

    CE[5][16] = 1.0;

    CE[6][5] = 1.0;
    CE[6][11] = -1.0;

    CE[7][11] = 1.0;
    CE[7][17] = -1.0;

    CE[8][17] = 1.0;

    CE[9][0] = 1.0;
    CE[9][3] = dt * v1 * cos(t1) * sin(p1);
    CE[9][4] = -dt * v1 * cos(p1) * cos(t1);
    CE[9][6] = -1.0;

    CE[10][6] = 1.0;
    CE[10][9] = dt * v2 * cos(t2) * sin(p2);
    CE[10][10] = -dt * v2 * cos(p2) * cos(t2);
    CE[10][12] = -1.0;

    CE[11][12] = 1.0;
    CE[11][15] = dt * v3 * cos(t3) * sin(p3);
    CE[11][16] = -dt * v3 * cos(p3) * cos(t3);

    CE[12][1] = 1.0;
    CE[12][3] = dt * v1 * cos(p1) * sin(t1);
    CE[12][4] = dt * v1 * sin(p1) * sin(t1);
    CE[12][5] = -dt * v1 * cos(t1);
    CE[12][7] = -1.0;

    CE[13][7] = 1.0;
    CE[13][9] = dt * v2 * cos(p2) * sin(t2);
    CE[13][10] = dt * v2 * sin(p2) * sin(t2);
    CE[13][11] = -dt * v2 * cos(t2);
    CE[13][13] = -1.0;

    CE[14][13] = 1.0;
    CE[14][15] = dt * v3 * cos(p3) * sin(t3);
    CE[14][16] = dt * v3 * sin(p3) * sin(t3);
    CE[14][17] = -dt * v3 * cos(t3);

    CE[15][2] = 1.0;
    CE[15][3] = -dt * cos(p1) * cos(t1);
    CE[15][4] = -dt * cos(t1) * sin(p1);
    CE[15][5] = -dt * sin(t1);
    CE[15][6] = (dt * u2) / (v1 * v1);
    CE[15][7] = (dt * w2) / (v1 * v1);
    CE[15][8] = -1.0;

    CE[16][8] = 1.0;
    CE[16][9] = -dt * cos(p2) * cos(t2);
    CE[16][10] = -dt * cos(t2) * sin(p2);
    CE[16][11] = -dt * sin(t2);
    CE[16][12] = (dt * u3) / (v2 * v2);
    CE[16][13] = (dt * w3) / (v2 * v2);
    CE[16][14] = -1.0;

    CE[17][14] = 1.0;
    CE[17][15] = -dt * cos(p3) * cos(t3);
    CE[17][16] = -dt * cos(t3) * sin(p3);
    CE[17][17] = -dt * sin(t3);

    CE[18][2] = -dt;

    CE[19][8] = -dt;

    CE[20][14] = -dt;

    CE[21][0] = -dt / v0;

    CE[22][6] = -dt / v1;

    CE[23][12] = -dt / v2;

    CE[24][1] = -dt / v0;

    CE[25][7] = -dt / v1;

    CE[26][13] = -dt / v2;
}

// Constant term of equation constraint fcn
void CE0(Vector<double> x, Vector<double> &ce0)
{
    if (v0 < 10) { v0 = 10; }
    if (v1 < 10) { v1 = 10; }
    if (v2 < 10) { v2 = 10; }
    if (v3 < 10) { v3 = 10; }

    ce0[0] = p1 - p0 - (u1 * dt / v0);
    ce0[1] = t1 - t0 - (w1 * dt / v0);
    ce0[2] = v1 - v0 - (a1 * dt);
    ce0[3] = x1 - x0 - (v1 * cos(t1) * cos(p1) * dt);
    ce0[4] = y1 - y0 - (v1 * cos(t1) * sin(p1) * dt);
    ce0[5] = z1 - z0 - (v1 * sin(t1) * dt);

    ce0[6] = p2 - p1 - (u2 * dt / v1);
    ce0[7] = t2 - t1 - (w2 * dt / v1);
    ce0[8] = v2 - v1 - (a2 * dt);
    ce0[9] = x2 - x1 - (v2 * cos(t2) * cos(p2) * dt);
    ce0[10] = y2 - y1 - (v2 * cos(t2) * sin(p2) * dt);
    ce0[11] = z2 - z1 - (v2 * sin(t2) * dt);

    ce0[12] = p3 - p2 - (u3 * dt / v2);
    ce0[13] = t3 - t2 - (w3 * dt / v2);
    ce0[14] = v3 - v2 - (a3 * dt);
    ce0[15] = x3 - x2 - (v3 * cos(t3) * cos(p3) * dt);
    ce0[16] = y3 - y2 - (v3 * cos(t3) * sin(p3) * dt);
    ce0[17] = z3 - z2 - (v3 * sin(t3) * dt);
}

// Gradient of inequation constraint fcn
void GradientCI(Vector<double> x, Matrix<double> &CI)
{
    for (uint8_t i = 0; i < VARIABLES_NUMBER; i++)
    {
        for (uint8_t j = 0; j < NECONS_NUMBER; j++)
        {
            CI[i][j] = 0;
        }
    }

    CI[0][0] = 2 * (x1 - xo1);
    CI[3][0] = 2 * (y1 - yo1);
    CI[6][0] = 2 * (z1 - zo1);
}

// Constant term of inequation constraint fcn
void CI0(Vector<double> x, Vector<double> &ci0)
{
    ci0[0] = (x1 - xo1) * (x1 - xo1) +
            (y1 - yo1) * (y1 - yo1) +
            (z1 - zo1) * (z1 - zo1) - 400.0;
}

// Value of the quadratic fcn
double Quadfun(Matrix<double> Q, Vector<double> c, Vector<double> d)
{
    double sum = 0.0;
    for (uint8_t i = 0; i < Q.nrows(); i++)
    {
        for (uint8_t j = 0; j < Q.ncols(); j++)
        {
            sum += d[i] * Q[i][j] * d[j];
        }
    }

    sum *= 0.5;

    for (uint8_t i = 0; i < c.size(); i++)
    {
        sum += c[i] * d[i];
    }

    return sum;
}

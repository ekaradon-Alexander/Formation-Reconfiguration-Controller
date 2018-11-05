#ifndef _PROBLEM_H_
#define _PROBLEM_H_

#include "Array.h"
#include "main.h"
using namespace QuadProg;

#define VARIABLES_NUMBER        ((CONTROL_NUMBER + STATE_NUMBER) * PREDICT_HORIZON)
#define EQCONS_NUMBER           (STATE_NUMBER * PREDICT_HORIZON)
#define NECONS_NUMBER           1

// Objective fcn
double objFcn(Vector<double> x);

// Gradient of objective fcn
void Gradient(Vector<double> x, Vector<double> &c);

// Hessian matrix of objective fcn
void Hessian(Vector<double> x, Matrix<double> &Q);

// Gradient of equation constraints
void GradientCE(Vector<double> x, Matrix<double> &CE);

// Contant term of equation constraint fcn
void CE0(Vector<double> x, Vector<double> &ce0);

// Gradient of inequation constraint fcn
void GradientCI(Vector<double> x, Matrix<double> &CI);

// Constant term of inequation constraint fcn
void CI0(Vector<double> x, Vector<double> &ci0);

// Value of the quadratic fcn
double Quadfun(Matrix<double> Q, Vector<double> c, Vector<double> d);

#endif // _PROBLEM_H_

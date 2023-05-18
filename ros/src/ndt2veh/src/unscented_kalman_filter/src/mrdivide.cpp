/*
 * File: mrdivide.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 12-May-2018 14:07:49
 */

/* Include Files */
#include <math.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "UnscentedKalmanFilter.h"
#include "mrdivide.h"

/* Function Definitions */

/*
 * Arguments    : const double A[9]
 *                const double B[9]
 *                double y[9]
 * Return Type  : void
 */
void mrdivide(const double A[9], const double B[9], double y[9])
{
  double b_A[9];
  int r1;
  int r2;
  int r3;
  double maxval;
  double a21;
  int rtemp;
  memcpy(&b_A[0], &B[0], 9U * sizeof(double));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabs(B[0]);
  a21 = fabs(B[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (fabs(B[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  b_A[r2] = B[r2] / B[r1];
  b_A[r3] /= b_A[r1];
  b_A[3 + r2] -= b_A[r2] * b_A[3 + r1];
  b_A[3 + r3] -= b_A[r3] * b_A[3 + r1];
  b_A[6 + r2] -= b_A[r2] * b_A[6 + r1];
  b_A[6 + r3] -= b_A[r3] * b_A[6 + r1];
  if (fabs(b_A[3 + r3]) > fabs(b_A[3 + r2])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  b_A[3 + r3] /= b_A[3 + r2];
  b_A[6 + r3] -= b_A[3 + r3] * b_A[6 + r2];
  for (rtemp = 0; rtemp < 3; rtemp++) {
    y[rtemp + 3 * r1] = A[rtemp] / b_A[r1];
    y[rtemp + 3 * r2] = A[3 + rtemp] - y[rtemp + 3 * r1] * b_A[3 + r1];
    y[rtemp + 3 * r3] = A[6 + rtemp] - y[rtemp + 3 * r1] * b_A[6 + r1];
    y[rtemp + 3 * r2] /= b_A[3 + r2];
    y[rtemp + 3 * r3] -= y[rtemp + 3 * r2] * b_A[6 + r2];
    y[rtemp + 3 * r3] /= b_A[6 + r3];
    y[rtemp + 3 * r2] -= y[rtemp + 3 * r3] * b_A[3 + r3];
    y[rtemp + 3 * r1] -= y[rtemp + 3 * r3] * b_A[r3];
    y[rtemp + 3 * r1] -= y[rtemp + 3 * r2] * b_A[r2];
  }
}

/*
 * File trailer for mrdivide.c
 *
 * [EOF]
 */

/*
 * File: sum.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 12-May-2018 14:07:49
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "UnscentedKalmanFilter.h"
#include "sum.h"

/* Function Definitions */

/*
 * Arguments    : const double x[21]
 *                double y[3]
 * Return Type  : void
 */
void sum(const double x[21], double y[3])
{
  int j;
  int k;
  int xoffset;
  for (j = 0; j < 3; j++) {
    y[j] = x[j];
  }

  for (k = 0; k < 6; k++) {
    xoffset = (k + 1) * 3;
    for (j = 0; j < 3; j++) {
      y[j] += x[xoffset + j];
    }
  }
}

/*
 * File trailer for sum.c
 *
 * [EOF]
 */

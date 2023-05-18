/*
 * File: mod.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 12-May-2018 14:07:49
 */

/* Include Files */
#include <math.h>
#include "rt_nonfinite.h"
#include "UnscentedKalmanFilter.h"
#include "mod.h"

/* Function Definitions */

/*
 * Arguments    : double x
 * Return Type  : double
 */
double b_mod(double x)
{
  double r;
  boolean_T rEQ0;
  double q;
  if ((!rtIsInf(x)) && (!rtIsNaN(x))) {
    if (x == 0.0) {
      r = 0.0;
    } else {
      r = fmod(x, 6.2831853071795862);
      rEQ0 = (r == 0.0);
      if (!rEQ0) {
        q = fabs(x / 6.2831853071795862);
        rEQ0 = (fabs(q - floor(q + 0.5)) <= 2.2204460492503131E-16 * q);
      }

      if (rEQ0) {
        r = 0.0;
      } else {
        if (x < 0.0) {
          r += 6.2831853071795862;
        }
      }
    }
  } else {
    r = rtNaN;
  }

  return r;
}

/*
 * File trailer for mod.c
 *
 * [EOF]
 */

/*
 * File: UnscentedKalmanFilter.h
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 12-May-2018 14:07:49
 */

#ifndef UNSCENTEDKALMANFILTER_H
#define UNSCENTEDKALMANFILTER_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "UnscentedKalmanFilter_types.h"

/* Function Declarations */
extern void UnscentedKalmanFilter(unsigned char updateMode, double sampleTime,
  const double q[3], const double r[3], const double sensorSignal[5], const
  double X_[3], const double P_[9], double X[3], double P[9]);

#endif

/*
 * File trailer for UnscentedKalmanFilter.h
 *
 * [EOF]
 */

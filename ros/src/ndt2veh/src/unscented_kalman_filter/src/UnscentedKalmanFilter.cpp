/*
 * File: UnscentedKalmanFilter.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 12-May-2018 16:56:57
 */

/* Include Files */
#include <math.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "UnscentedKalmanFilter.h"
#include "mrdivide.h"
#include "mod.h"
#include "sum.h"
#include "diag.h"

/* Function Definitions */

/*
 * Input assign
 * Arguments    : unsigned char updateMode
 *                double sampleTime
 *                const double q[3]
 *                const double r[3]
 *                const double sensorSignal[5]
 *                const double X_[3]
 *                const double P_[9]
 *                double X[3]
 *                double P[9]
 * Return Type  : void
 */
void UnscentedKalmanFilter(unsigned char updateMode, double sampleTime, const
  double q[3], const double r[3], const double sensorSignal[5], const double X_
  [3], const double P_[9], double X[3], double P[9])
{
  double speed;
  int i0;
  double Wc[7];
  static const double Wm[7] = { 0.0, 0.16666666666666666, 0.16666666666666666,
    0.16666666666666666, 0.16666666666666666, 0.16666666666666666,
    0.16666666666666666 };

  double X_bar[9];
  int info;
  int colj;
  int j;
  boolean_T exitg1;
  int jj;
  int jmax;
  double ajj;
  int ix;
  int iy;
  int k;
  int r2;
  double X_sig[21];
  double A[9];
  double X_tt[21];
  double X_t[21];
  double Y;
  double b_X_t[3];
  static const double dv0[7] = { 0.0, 0.16666666666666666, 0.16666666666666666,
    0.16666666666666666, 0.16666666666666666, 0.16666666666666666,
    0.16666666666666666 };

  double c_X_t[3];
  double Pxy[3];
  double Y_tt[7];
  double Y_t[14];
  double b_Y_t[21];
  double d_X_t[9];
  double R[4];
  double c_Y_t[7];
  double d_Y_t[3];
  double b_Y[2];
  double b_Y_tt[14];
  double K[9];
  double b_Pxy[6];
  double e_Y_t[2];
  double f_Y_t[2];
  double g_Y_t[4];
  double b_K[6];
  double e_X_t[6];
  speed = sensorSignal[3];

  /*  UKF Parameters */
  /*  State numbers */
  /*  default, tunable */
  /*  default, tunable */
  /*  default, tunable */
  /*  scaling factor */
  /*  Weights for means */
  for (i0 = 0; i0 < 7; i0++) {
    Wc[i0] = Wm[i0];
  }

  Wc[0] = 2.0;

  /*  Weights for covariance */
  diag(q, P);

  /* --Sigma points generation */
  memcpy(&X_bar[0], &P_[0], 9U * sizeof(double));
  info = 0;
  colj = 0;
  j = 1;
  exitg1 = false;
  while ((!exitg1) && (j < 4)) {
    jj = (colj + j) - 1;
    ajj = 0.0;
    if (!(j - 1 < 1)) {
      ix = colj;
      iy = colj;
      for (k = 1; k < j; k++) {
        ajj += X_bar[ix] * X_bar[iy];
        ix++;
        iy++;
      }
    }

    ajj = X_bar[jj] - ajj;
    if (ajj > 0.0) {
      ajj = sqrt(ajj);
      X_bar[jj] = ajj;
      if (j < 3) {
        if (j - 1 != 0) {
          iy = jj + 3;
          i0 = (colj + 3 * (2 - j)) + 4;
          for (jmax = colj + 4; jmax <= i0; jmax += 3) {
            ix = colj;
            Y = 0.0;
            k = (jmax + j) - 2;
            for (r2 = jmax; r2 <= k; r2++) {
              Y += X_bar[r2 - 1] * X_bar[ix];
              ix++;
            }

            X_bar[iy] += -Y;
            iy += 3;
          }
        }

        ajj = 1.0 / ajj;
        i0 = (jj + 3 * (2 - j)) + 4;
        for (k = jj + 3; k + 1 <= i0; k += 3) {
          X_bar[k] *= ajj;
        }

        colj += 3;
      }

      j++;
    } else {
      X_bar[jj] = ajj;
      info = j;
      exitg1 = true;
    }
  }

  if (info == 0) {
    jmax = 3;
  } else {
    jmax = info - 1;
  }

  for (j = 0; j < jmax; j++) {
    for (r2 = j + 1; r2 < jmax; r2++) {
      X_bar[r2 + 3 * j] = 0.0;
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (k = 0; k < 3; k++) {
      A[k + 3 * i0] = 1.7320508075688772 * X_bar[i0 + 3 * k];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    X_sig[i0] = X_[i0];
    for (k = 0; k < 3; k++) {
      X_bar[k + 3 * i0] = X_[k];
      X_sig[k + 3 * (i0 + 1)] = X_bar[k + 3 * i0] + A[k + 3 * i0];
      X_sig[k + 3 * (i0 + 4)] = X_bar[k + 3 * i0] - A[k + 3 * i0];
    }
  }

  /* --Time Update */
  for (r2 = 0; r2 < 7; r2++) {
    /*  Edit state equation in here */
    X_t[3 * r2] = X_sig[3 * r2] + sampleTime * speed * cos(X_sig[2 + 3 * r2]);
    X_t[1 + 3 * r2] = X_sig[1 + 3 * r2] + sampleTime * speed * sin(X_sig[2 + 3 *
      r2]);
    X_t[2 + 3 * r2] = X_sig[2 + 3 * r2] + sampleTime * sensorSignal[4];
    for (i0 = 0; i0 < 3; i0++) {
      X_tt[i0 + 3 * r2] = X_t[i0 + 3 * r2] * dv0[r2];
    }
  }

  sum(X_tt, X);

  /*  First Update */
  for (r2 = 0; r2 < 7; r2++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_X_t[i0] = X_t[i0 + 3 * r2] - X[i0];
      c_X_t[i0] = X_t[i0 + 3 * r2] - X[i0];
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (k = 0; k < 3; k++) {
        d_X_t[i0 + 3 * k] = b_X_t[i0] * c_X_t[k];
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (k = 0; k < 3; k++) {
        P[k + 3 * i0] += Wc[r2] * d_X_t[k + 3 * i0];
      }
    }
  }

  /* --Sensor Measurements */
  if (updateMode == 0) {
    diag(r, X_bar);
    for (r2 = 0; r2 < 7; r2++) {
      /*  Edit measurement model in here */
      b_Y_t[3 * r2] = X_sig[3 * r2];
      b_Y_t[1 + 3 * r2] = X_sig[1 + 3 * r2];
      b_Y_t[2 + 3 * r2] = X_sig[2 + 3 * r2];
      for (i0 = 0; i0 < 3; i0++) {
        X_tt[i0 + 3 * r2] = b_Y_t[i0 + 3 * r2] * dv0[r2];
      }
    }

    sum(X_tt, Pxy);

    /* --Measurement Update */
    for (r2 = 0; r2 < 7; r2++) {
      for (i0 = 0; i0 < 3; i0++) {
        d_Y_t[i0] = b_Y_t[i0 + 3 * r2] - Pxy[i0];
        c_X_t[i0] = b_Y_t[i0 + 3 * r2] - Pxy[i0];
      }

      for (i0 = 0; i0 < 3; i0++) {
        for (k = 0; k < 3; k++) {
          d_X_t[i0 + 3 * k] = d_Y_t[i0] * c_X_t[k];
        }
      }

      for (i0 = 0; i0 < 3; i0++) {
        for (k = 0; k < 3; k++) {
          X_bar[k + 3 * i0] += Wc[r2] * d_X_t[k + 3 * i0];
        }
      }
    }

    memset(&A[0], 0, 9U * sizeof(double));
    for (r2 = 0; r2 < 7; r2++) {
      for (i0 = 0; i0 < 3; i0++) {
        b_X_t[i0] = X_t[i0 + 3 * r2] - X[i0];
        d_Y_t[i0] = b_Y_t[i0 + 3 * r2] - Pxy[i0];
      }

      for (i0 = 0; i0 < 3; i0++) {
        for (k = 0; k < 3; k++) {
          d_X_t[i0 + 3 * k] = b_X_t[i0] * d_Y_t[k];
        }
      }

      for (i0 = 0; i0 < 3; i0++) {
        for (k = 0; k < 3; k++) {
          A[k + 3 * i0] += Wc[r2] * d_X_t[k + 3 * i0];
        }
      }
    }

    /*  heading flexibility across 0 & 2pi */
    ajj = b_mod(sensorSignal[2] - Pxy[2]);
    if (ajj > 3.1415926535897931) {
      ajj -= 6.2831853071795862;
    }

    if (ajj < -3.1415926535897931) {
      ajj += 6.2831853071795862;
    }

    mrdivide(A, X_bar, K);
    c_X_t[0] = sensorSignal[0] - Pxy[0];
    c_X_t[1] = sensorSignal[1] - Pxy[1];
    c_X_t[2] = ajj;

    /*  Final Update */
    for (i0 = 0; i0 < 3; i0++) {
      ajj = 0.0;
      for (k = 0; k < 3; k++) {
        ajj += K[i0 + 3 * k] * c_X_t[k];
        d_X_t[i0 + 3 * k] = 0.0;
        for (jmax = 0; jmax < 3; jmax++) {
          d_X_t[i0 + 3 * k] += K[i0 + 3 * jmax] * X_bar[jmax + 3 * k];
        }
      }

      for (k = 0; k < 3; k++) {
        Y = 0.0;
        for (jmax = 0; jmax < 3; jmax++) {
          Y += d_X_t[i0 + 3 * jmax] * K[k + 3 * jmax];
        }

        P[i0 + 3 * k] -= Y;
      }

      X[i0] += ajj;
    }
  } else if (updateMode == 1) {
    for (r2 = 0; r2 < 7; r2++) {
      /*  Edit measurement model in here */
      Y_t[r2 << 1] = X_sig[3 * r2];
      Y_t[1 + (r2 << 1)] = X_sig[1 + 3 * r2];
    }

    for (i0 = 0; i0 < 4; i0++) {
      R[i0] = 0.0;
    }

    for (j = 0; j < 2; j++) {
      R[j + (j << 1)] = r[j];
    }

    for (r2 = 0; r2 < 7; r2++) {
      for (i0 = 0; i0 < 2; i0++) {
        b_Y_tt[i0 + (r2 << 1)] = Y_t[i0 + (r2 << 1)] * dv0[r2];
      }
    }

    for (j = 0; j < 2; j++) {
      b_Y[j] = b_Y_tt[j];
    }

    for (k = 0; k < 6; k++) {
      jmax = (k + 1) << 1;
      for (j = 0; j < 2; j++) {
        b_Y[j] += b_Y_tt[jmax + j];
      }
    }

    /* --Measurement Update */
    for (r2 = 0; r2 < 7; r2++) {
      for (i0 = 0; i0 < 2; i0++) {
        e_Y_t[i0] = Y_t[i0 + (r2 << 1)] - b_Y[i0];
        f_Y_t[i0] = Y_t[i0 + (r2 << 1)] - b_Y[i0];
      }

      for (i0 = 0; i0 < 2; i0++) {
        for (k = 0; k < 2; k++) {
          g_Y_t[i0 + (k << 1)] = e_Y_t[i0] * f_Y_t[k];
        }
      }

      for (i0 = 0; i0 < 2; i0++) {
        for (k = 0; k < 2; k++) {
          R[k + (i0 << 1)] += Wc[r2] * g_Y_t[k + (i0 << 1)];
        }
      }
    }

    for (i0 = 0; i0 < 6; i0++) {
      b_Pxy[i0] = 0.0;
    }

    for (r2 = 0; r2 < 7; r2++) {
      for (i0 = 0; i0 < 3; i0++) {
        b_X_t[i0] = X_t[i0 + 3 * r2] - X[i0];
      }

      for (i0 = 0; i0 < 2; i0++) {
        e_Y_t[i0] = Y_t[i0 + (r2 << 1)] - b_Y[i0];
      }

      for (i0 = 0; i0 < 3; i0++) {
        for (k = 0; k < 2; k++) {
          e_X_t[i0 + 3 * k] = b_X_t[i0] * e_Y_t[k];
        }
      }

      for (i0 = 0; i0 < 2; i0++) {
        for (k = 0; k < 3; k++) {
          b_Pxy[k + 3 * i0] += Wc[r2] * e_X_t[k + 3 * i0];
        }
      }
    }

    if (fabs(R[1]) > fabs(R[0])) {
      jmax = 1;
      r2 = 0;
    } else {
      jmax = 0;
      r2 = 1;
    }

    ajj = R[r2] / R[jmax];
    Y = R[2 + r2] - ajj * R[2 + jmax];
    for (k = 0; k < 3; k++) {
      b_K[k + 3 * jmax] = b_Pxy[k] / R[jmax];
      b_K[k + 3 * r2] = (b_Pxy[3 + k] - b_K[k + 3 * jmax] * R[2 + jmax]) / Y;
      b_K[k + 3 * jmax] -= b_K[k + 3 * r2] * ajj;
    }

    e_Y_t[0] = sensorSignal[0];
    e_Y_t[1] = sensorSignal[1];
    for (i0 = 0; i0 < 2; i0++) {
      f_Y_t[i0] = e_Y_t[i0] - b_Y[i0];
    }

    /*  Final Update */
    for (i0 = 0; i0 < 3; i0++) {
      ajj = 0.0;
      for (k = 0; k < 2; k++) {
        ajj += b_K[i0 + 3 * k] * f_Y_t[k];
        e_X_t[i0 + 3 * k] = 0.0;
        for (jmax = 0; jmax < 2; jmax++) {
          e_X_t[i0 + 3 * k] += b_K[i0 + 3 * jmax] * R[jmax + (k << 1)];
        }
      }

      for (k = 0; k < 3; k++) {
        Y = 0.0;
        for (jmax = 0; jmax < 2; jmax++) {
          Y += e_X_t[i0 + 3 * jmax] * b_K[k + 3 * jmax];
        }

        P[i0 + 3 * k] -= Y;
      }

      X[i0] += ajj;
    }
  } else {
    if (updateMode == 2) {
      for (r2 = 0; r2 < 7; r2++) {
        /*  Edit measurement model in here */
        Y_tt[r2] = X_sig[3 * r2] * dv0[r2];
        c_Y_t[r2] = X_sig[3 * r2];
      }

      Y = Y_tt[0];
      for (k = 0; k < 6; k++) {
        Y += Y_tt[k + 1];
      }

      /* --Measurement Update */
      speed = r[2];
      for (r2 = 0; r2 < 7; r2++) {
        speed += Wc[r2] * ((c_Y_t[r2] - Y) * (c_Y_t[r2] - Y));
      }

      for (r2 = 0; r2 < 3; r2++) {
        Pxy[r2] = 0.0;
      }

      for (r2 = 0; r2 < 7; r2++) {
        ajj = c_Y_t[r2] - Y;
        for (i0 = 0; i0 < 3; i0++) {
          Pxy[i0] += Wc[r2] * ((X_t[i0 + 3 * r2] - X[i0]) * ajj);
        }
      }

      /*  heading flexibility across 0 & 2pi */
      ajj = b_mod(sensorSignal[2] - Y);
      if (ajj > 3.1415926535897931) {
        ajj -= 6.2831853071795862;
      }

      if (ajj < -3.1415926535897931) {
        ajj += 6.2831853071795862;
      }

      for (i0 = 0; i0 < 3; i0++) {
        Y = Pxy[i0] / speed;
        Pxy[i0] = Y;
        X[i0] += Y * ajj;
      }

      /*  Final Update */
      for (i0 = 0; i0 < 3; i0++) {
        for (k = 0; k < 3; k++) {
          P[i0 + 3 * k] -= Pxy[i0] * speed * Pxy[k];
        }
      }
    }
  }
}

/*
 * File trailer for UnscentedKalmanFilter.c
 *
 * [EOF]
 */

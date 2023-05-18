/*
 * Copyright (c) 2014, Nagoya University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Autoware nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef GSYMMETRIC_EIGEN_
#define GSYMMETRIC_EIGEN_

#include <cuda.h>
#include <cuda_runtime.h>
#include <math.h>
#include <ndt_matcher/ndt_gpu/matrix_device.h>

namespace gpu
{

class SymmetricEigensolver3x3
{
public:
  CUDAH SymmetricEigensolver3x3();
  SymmetricEigensolver3x3(int offset);
  CUDAH SymmetricEigensolver3x3(const SymmetricEigensolver3x3& other);
  void SetInputMatrices(double *inputMatrices);
  void SetEigenvectors(double *eigenvectors);
  void SetEigenvalues(double *eigenvalues);
  double *GetBuffer() const;
  /* Normalize input matrices by dividing each matrix
   * to the element that has the largest absolute value
   * in each matrix */
  CUDAH void NormalizeInput(int tid);
  /* Compute eigenvalues */
  CUDAH void ComputeEigenvalues(int tid);
  /* First step to compute the eigenvector 0
   * Because computing the eigenvector 0 using
   * only one kernel is too expensive (which causes
   * the "too many resources requested for launch" error,
   * I have to divide them into two distinct kernels. */
  CUDAH void ComputeEigenvector00(int tid);
  /* Second step to compute the eigenvector 0 */
  CUDAH void ComputeEigenvector01(int tid);
  /* First step to compute the eigenvector 1 */
  CUDAH void ComputeEigenvector10(int tid);
  /* Second step to compute the eigenvector 1 */
  CUDAH void ComputeEigenvector11(int tid);
  /* Compute the final eigenvector by crossing
   * eigenvector 0 and eigenvector 1 */
  CUDAH void ComputeEigenvector2(int tid);
  /* Final step to compute eigenvalues */
  CUDAH void UpdateEigenvalues(int tid);
  /* Free memory */
  void MemFree();

private:
  CUDAH void ComputeOrthogonalComplement(MatrixDevice w, MatrixDevice u, MatrixDevice v);
  /* Operators */
  CUDAH void Multiply(MatrixDevice u, double mult, MatrixDevice output);
  CUDAH void Subtract(MatrixDevice u, MatrixDevice v, MatrixDevice output);
  CUDAH void Divide(MatrixDevice u, double div, MatrixDevice output);
  CUDAH double Dot(MatrixDevice u, MatrixDevice v);
  CUDAH void Cross(MatrixDevice in0, MatrixDevice in1, MatrixDevice out);

  int mOffset;
  /* Buffers for intermediate calculation */
  double *mBuffer;
  int *mI02;
  double *mMaxAbsElement;
  double *mNorm;
  double *mEigenvectors;
  double *mEigenvalues;
  double *mInputMatrices;

  bool mIsCopied;
};

CUDAH SymmetricEigensolver3x3::SymmetricEigensolver3x3()
{
  mBuffer = NULL;
  mEigenvectors = NULL;
  mEigenvalues = NULL;
  mInputMatrices = NULL;
  mMaxAbsElement = NULL;
  mNorm = NULL;
  mI02 = NULL;
  mOffset = 0;
  mIsCopied = false;
}

CUDAH SymmetricEigensolver3x3::SymmetricEigensolver3x3(const SymmetricEigensolver3x3& other)
{
  mBuffer = other.mBuffer;
  mOffset = other.mOffset;
  mEigenvectors = other.mEigenvectors;
  mEigenvalues = other.mEigenvalues;
  mInputMatrices = other.mInputMatrices;

  mMaxAbsElement = other.mMaxAbsElement;
  mNorm = other.mNorm;
  mI02 = other.mI02;
  mIsCopied = true;
}

CUDAH void SymmetricEigensolver3x3::NormalizeInput(int tid)
{
  MatrixDevice input(3, 3, mOffset, mInputMatrices + tid);
  double a00 = input(0, 0);
  double a01 = input(0, 1);
  double a02 = input(0, 2);
  double a11 = input(1, 1);
  double a12 = input(1, 2);
  double a22 = input(2, 2);
  double max0 = (fabs(a00) > fabs(a01)) ? fabs(a00) : fabs(a01);
  double max1 = (fabs(a02) > fabs(a11)) ? fabs(a02) : fabs(a11);
  double max2 = (fabs(a12) > fabs(a22)) ? fabs(a12) : fabs(a22);

  double maxAbsElement = (max0 > max1) ? max0 : max1;
  maxAbsElement = (maxAbsElement > max2) ? maxAbsElement : max2;

  if (maxAbsElement == 0.0)
  {
    MatrixDevice evec(3, 3, mOffset, mEigenvectors + tid);
    evec(0, 0) = 1.0;
    evec(1, 1) = 1.0;
    evec(2, 2) = 1.0;
    mNorm[tid] = 0.0;
    return;
  }

  double invMaxAbsElement = 1.0 / maxAbsElement;
  a00 *= invMaxAbsElement;
  a01 *= invMaxAbsElement;
  a02 *= invMaxAbsElement;
  a11 *= invMaxAbsElement;
  a12 *= invMaxAbsElement;
  a22 *= invMaxAbsElement;

  input(0, 0) = a00;
  input(0, 1) = a01;
  input(0, 2) = a02;
  input(1, 1) = a11;
  input(1, 2) = a12;
  input(2, 2) = a22;
  input(1, 0) = a01;
  input(2, 0) = a02;
  input(2, 1) = a12;

  mNorm[tid] = a01 * a01 + a02 * a02 + a12 * a12;
  mMaxAbsElement[tid] = maxAbsElement;
}

CUDAH void SymmetricEigensolver3x3::ComputeEigenvalues(int tid)
{
  MatrixDevice input(3, 3, mOffset, mInputMatrices + tid);
  MatrixDevice eval(3, 1, mOffset, mEigenvalues + tid);

  double a00 = input(0, 0);
  double a01 = input(0, 1);
  double a02 = input(0, 2);
  double a11 = input(1, 1);
  double a12 = input(1, 2);
  double a22 = input(2, 2);

  double norm = mNorm[tid];

  if (norm > 0.0)
  {
    double traceDiv3 = (a00 + a11 + a22) / 3.0;
    double b00 = a00 - traceDiv3;
    double b11 = a11 - traceDiv3;
    double b22 = a22 - traceDiv3;
    double denom = sqrt((b00 * b00 + b11 * b11 + b22 * b22 + norm * 2.0) / 6.0);
    double c00 = b11 * b22 - a12 * a12;
    double c01 = a01 * b22 - a12 * a02;
    double c02 = a01 * a12 - b11 * a02;
    double det = (b00 * c00 - a01 * c01 + a02 * c02) / (denom * denom * denom);
    double halfDet = det * 0.5;

    halfDet = (halfDet > -1.0) ? halfDet : -1.0;
    halfDet = (halfDet < 1.0) ? halfDet : 1.0;

    double angle = acos(halfDet) / 3.0;
    double beta2 = cos(angle) * 2.0;
    double beta0 = cos(angle + M_PI * 2.0 / 3.0) * 2.0;
    double beta1 = -(beta0 + beta2);

    eval(0) = traceDiv3 + denom * beta0;
    eval(1) = traceDiv3 + denom * beta1;
    eval(2) = traceDiv3 + denom * beta2;

    mI02[tid] = (halfDet >= 0) ? 2 : 0;
    mI02[tid + mOffset] = (halfDet >= 0) ? 0 : 2;
  }
  else
  {
    eval(0) = a00;
    eval(1) = a11;
    eval(2) = a22;
  }
}

CUDAH void SymmetricEigensolver3x3::UpdateEigenvalues(int tid)
{
  double maxAbsElement = mMaxAbsElement[tid];
  MatrixDevice eval(3, 1, mOffset, mEigenvalues + tid);

  eval(0) *= maxAbsElement;
  eval(1) *= maxAbsElement;
  eval(2) *= maxAbsElement;
}

CUDAH void SymmetricEigensolver3x3::ComputeEigenvector00(int tid)
{
  if (mNorm[tid] > 0.0)
  {
    MatrixDevice input(3, 3, mOffset, mInputMatrices + tid);
    MatrixDevice rowMat(3, 3, mOffset, mBuffer + tid);
    double eval0 = mEigenvalues[tid + mI02[tid] * mOffset];

    input.copy(rowMat);
    rowMat(0, 0) -= eval0;
    rowMat(1, 1) -= eval0;
    rowMat(2, 2) -= eval0;

    //row0 is r0xr1, row1 is r0xr2, row2 is r1xr2
    MatrixDevice rxr(3, 3, mOffset, mBuffer + 3 * 3 * mOffset + tid);
    Cross(rowMat.row(0), rowMat.row(1), rxr.row(0));
    Cross(rowMat.row(0), rowMat.row(2), rxr.row(1));
    Cross(rowMat.row(1), rowMat.row(2), rxr.row(2));
  }
  else
  {
    mEigenvectors[tid] = 1.0;
  }
}

CUDAH void SymmetricEigensolver3x3::ComputeEigenvector01(int tid)
{
  if (mNorm[tid] > 0.0)
  {
    MatrixDevice evec0(3, 1, mOffset * 3, mEigenvectors + tid + mI02[tid] * mOffset);
    //row0 is r0xr1, row1 is r0xr2, row2 is r1xr2
    MatrixDevice rxr(3, 3, mOffset, mBuffer + 3 * 3 * mOffset + tid);
    double d0 = rxr(0, 0) * rxr(0, 0) + rxr(0, 1) * rxr(0, 1) * rxr(0, 2) * rxr(0, 2);
    double d1 = rxr(1, 0) * rxr(1, 0) + rxr(1, 1) * rxr(1, 1) * rxr(1, 2) * rxr(1, 2);
    double d2 = rxr(2, 0) * rxr(2, 0) + rxr(2, 1) * rxr(2, 1) * rxr(2, 2) * rxr(2, 2);

    double dmax = (d0 > d1) ? d0 : d1;
    int imax = (d0 > d1) ? 0 : 1;

    dmax = (d2 > dmax) ? d2 : dmax;
    imax = (d2 > dmax) ? 2 : imax;

    Divide(rxr.row(imax), sqrt(dmax), evec0);
  }
}

CUDAH void SymmetricEigensolver3x3::ComputeEigenvector10(int tid)
{
  if (mNorm[tid] > 0.0)
  {
    MatrixDevice input(3, 3, mOffset, mInputMatrices + tid);
    MatrixDevice evec0(3, 1, mOffset * 3, mEigenvectors + tid + mI02[tid] * mOffset);
    double eval1 = mEigenvalues[tid + mOffset];

    MatrixDevice u(3, 1, mOffset, mBuffer + tid);
    MatrixDevice v(3, 1, mOffset, mBuffer + 3 * mOffset + tid);

    ComputeOrthogonalComplement(evec0, u, v);

    MatrixDevice au(3, 1, mOffset, mBuffer + 6 * mOffset + tid);
    MatrixDevice av(3, 1, mOffset, mBuffer + 9 * mOffset + tid);

    double t0, t1, t2;
    t0 = u(0);
    t1 = u(1);
    t2 = u(2);

    au(0) = (input(0, 0) - eval1) * t0 + input(0, 1) * t1 + input(0, 2) * t2;
    au(1) = input(0, 1) * t0 + (input(1, 1) - eval1) * t1 + input(1, 2) * t2;
    au(2) = input(0, 2) * t0 + input(1, 2) * t1 + (input(2, 2) - eval1) * t2;

    t0 = v(0);
    t1 = v(1);
    t2 = v(2);

    av(0) = (input(0, 0) - eval1) * t0 + input(0, 1) * t1 + input(0, 2) * t2;
    av(1) = input(0, 1) * t0 + (input(1, 1) - eval1) * t1 + input(1, 2) * t2;
    av(2) = input(0, 2) * t0 + input(1, 2) * t1 + (input(2, 2) - eval1) * t2;
  }
  else
  {
    mEigenvectors[tid + mOffset * 4] = 1.0;
  }
}

CUDAH void SymmetricEigensolver3x3::ComputeEigenvector11(int tid)
{
  if (mNorm[tid] > 0.0)
  {
    MatrixDevice evec1(3, 1, mOffset * 3, mEigenvectors + tid + mOffset);
    MatrixDevice u(3, 1, mOffset, mBuffer + tid);
    MatrixDevice v(3, 1, mOffset, mBuffer + 3 * mOffset + tid);
    MatrixDevice au(3, 1, mOffset, mBuffer + 6 * mOffset + tid);
    MatrixDevice av(3, 1, mOffset, mBuffer + 9 * mOffset + tid);

    double m00 = u(0) * au(0) + u(1) * au(1) + u(2) * au(2);
    double m01 = u(0) * av(0) + u(1) * av(1) + u(2) * av(2);
    double m11 = v(0) * av(0) + v(1) * av(1) + v(2) * av(2);

    double absM00 = fabs(m00);
    double absM01 = fabs(m01);
    double absM11 = fabs(m11);

    if (absM00 > 0 || absM01 > 0 || absM11 > 0)
    {
      double uMult = (absM00 >= absM11) ? m01 : m11;
      double vMult = (absM00 >= absM11) ? m00 : m01;

      bool res = fabs(uMult) >= fabs(vMult);
      double *large = (res) ? &uMult : &vMult;
      double *small = (res) ? &vMult : &uMult;

      *small /= (*large);
      *large = 1.0 / sqrt(1.0 + (*small) * (*small));
      *small *= (*large);

      Multiply(u, uMult, u);
      Multiply(v, vMult, v);
      Subtract(u, v, evec1);
    }
    else
    {
      u.copy(evec1);
    }
  }
}

CUDAH void SymmetricEigensolver3x3::Multiply(MatrixDevice u,
  double mult, MatrixDevice output)
{
  output(0) = u(0) * mult;
  output(1) = u(1) * mult;
  output(2) = u(2) * mult;
}

CUDAH void SymmetricEigensolver3x3::Subtract(MatrixDevice u,
  MatrixDevice v, MatrixDevice output)
{
  output(0) = u(0) - v(0);
  output(1) = u(1) - v(1);
  output(2) = u(2) - v(2);
}

CUDAH void SymmetricEigensolver3x3::Divide(MatrixDevice u,
  double div, MatrixDevice output)
{
  output(0) = u(0) / div;
  output(1) = u(1) / div;
  output(2) = u(2) / div;
}

CUDAH double SymmetricEigensolver3x3::Dot(MatrixDevice u, MatrixDevice v)
{
  return (u(0) * v(0) + u(1) * v(1) + u(2) * v(2));
}

CUDAH void SymmetricEigensolver3x3::Cross(MatrixDevice u,
  MatrixDevice v, MatrixDevice out)
{
  out(0) = u(1) * v(2) - u(2) * v(1);
  out(1) = u(2) * v(0) - u(0) * v(2);
  out(2) = u(0) * v(1) - u(1) * v(0);
}

CUDAH void SymmetricEigensolver3x3::ComputeOrthogonalComplement(MatrixDevice w,
  MatrixDevice u, MatrixDevice v)
{
  bool c = (fabs(w(0)) > fabs(w(1)));
  double invLength = (c) ?
    (1.0 / sqrt(w(0) * w(0) + w(2) * w(2))) : (1.0 / sqrt(w(1) * w(1) + w(2) * w(2)));

  u(0) = (c) ? -w(2) * invLength : 0.0;
  u(1) = (c) ? 0.0 : w(2) * invLength;
  u(2) = (c) ? w(0) * invLength : -w(1) * invLength;
  Cross(w, u, v);
}

CUDAH void SymmetricEigensolver3x3::ComputeEigenvector2(int tid)
{
  if (mNorm[tid] > 0.0)
  {
    MatrixDevice evec0(3, 1, mOffset * 3,
      mEigenvectors + tid + mI02[tid] * mOffset);
    MatrixDevice evec1(3, 1, mOffset * 3,
      mEigenvectors + tid + mOffset);
    MatrixDevice evec2(3, 1, mOffset * 3,
      mEigenvectors + tid + mI02[tid + mOffset] * mOffset);

    Cross(evec0, evec1, evec2);
  }
  else
  {
    mEigenvectors[tid + mOffset * 8] = 1.0;
  }
}

} // namespace gpu

#endif // GSYMMETRIC_EIGEN_

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

#ifndef GMATRIX_H_
#define GMATRIX_H_

#include <cuda.h>
#include <cuda_runtime.h>
#include <float.h>
#include <memory>
#include <ndt_matcher/ndt_gpu/common.h>
#include <ndt_matcher/ndt_gpu/debug.h>
#include <ndt_matcher/ndt_gpu/memory.h>

namespace gpu
{

class Matrix
{
public:
  CUDAH Matrix();
  CUDAH Matrix(int rows, int cols, int offset, double *buffer);
  CUDAH int rows() const;
  CUDAH int cols() const;
  CUDAH int offset() const;
  CUDAH double *buffer() const;
  CUDAH void SetRows(int rows);
  CUDAH void SetCols(int cols);
  CUDAH void SetOffset(int offset);
  CUDAH void SetBuffer(double *buffer);
  CUDAH void SetCellVal(int row, int col, double val);
  CUDAH void copy(Matrix &output);

  CUDAH void resize(int rows, int cols);

  CUDAH double *CellAddr(int row, int col);
  CUDAH double *CellAddr(int index);

  CUDAH void operator=(const Matrix input);
  CUDAH double& operator()(int row, int col);
  CUDAH void set(int row, int col, double val);
  CUDAH double& operator()(int index);
  CUDAH double at(int row, int col) const;
  CUDAH bool operator*=(double val);
  CUDAH bool operator/=(double val);
  CUDAH bool transpose(Matrix &output);

  CUDAH bool inverse(Matrix &output);
  CUDAH Matrix col(int index);
  CUDAH Matrix row(int index);

protected:
  double *mBuffer;
  int mRows, mCols, mOffset;
};

CUDAH Matrix::Matrix()
{
  mBuffer = NULL;
  mRows = mCols = mOffset = 0;
}

CUDAH Matrix::Matrix(int rows, int cols, int offset, double *buffer)
{
  mRows = rows;
  mCols = cols;
  mOffset = offset;
  mBuffer = buffer;
}

CUDAH int Matrix::rows() const
{
  return mRows;
}

CUDAH int Matrix::cols() const
{
  return mCols;
}

CUDAH int Matrix::offset() const
{
  return mOffset;
}

CUDAH double *Matrix::buffer() const
{
  return mBuffer;
}

CUDAH void Matrix::SetRows(int rows)
{
  mRows = rows;
}

CUDAH void Matrix::SetCols(int cols)
{
  mCols = cols;
}

CUDAH void Matrix::SetOffset(int offset)
{
  mOffset = offset;
}

CUDAH void Matrix::SetBuffer(double *buffer)
{
  mBuffer = buffer;
}

CUDAH void Matrix::SetCellVal(int row, int col, double val)
{
  mBuffer[(row * mCols + col) * mOffset] = val;
}

CUDAH void Matrix::copy(Matrix &output)
{
  for (int i = 0; i < mRows; i++)
  {
    for (int j = 0; j < mCols; j++)
    {
      output(i, j) = mBuffer[(i * mCols + j) * mOffset];
    }
  }
}

CUDAH void Matrix::resize(int rows, int cols)
{
  mRows = rows;
  mCols = cols;
}

CUDAH double *Matrix::CellAddr(int row, int col)
{
  if (row >= mRows || col >= mCols || row < 0 || col < 0)
    return NULL;

  return mBuffer + (row * mCols + col) * mOffset;
}

CUDAH double *Matrix::CellAddr(int index)
{
  if (mRows == 1 && index >= 0 && index < mCols)
  {
    return mBuffer + index * mOffset;
  }
  else if (mCols == 1 && index >= 0 && index < mRows)
  {
    return mBuffer + index * mOffset;
  }

  return NULL;
}

CUDAH void Matrix::operator=(const Matrix input)
{
  mRows = input.mRows;
  mCols = input.mCols;
  mOffset = input.mOffset;
  mBuffer = input.mBuffer;
}

CUDAH double& Matrix::operator()(int row, int col)
{
  return mBuffer[(row * mCols + col) * mOffset];
}

CUDAH void Matrix::set(int row, int col, double val)
{
  mBuffer[(row * mCols + col) * mOffset] = val;
}

CUDAH double& Matrix::operator()(int index)
{
  return mBuffer[index * mOffset];
}

CUDAH double Matrix::at(int row, int col) const
{
  return mBuffer[(row * mCols + col) * mOffset];
}

CUDAH bool Matrix::operator*=(double val)
{
  for (int i = 0; i < mRows; i++)
  {
    for (int j = 0; j < mCols; j++)
    {
      mBuffer[(i * mCols + j) * mOffset] *= val;
    }
  }

  return true;
}

CUDAH bool Matrix::operator/=(double val)
{
  if (val == 0)
    return false;

  for (int i = 0; i < mRows * mCols; i++)
  {
    mBuffer[i * mOffset] /= val;
  }

  return true;
}

CUDAH bool Matrix::transpose(Matrix &output)
{
  if (mRows != output.mCols || mCols != output.mRows)
    return false;

  for (int i = 0; i < mRows; i++)
  {
    for (int j = 0; j < mCols; j++)
    {
      output(j, i) = mBuffer[(i * mCols + j) * mOffset];
    }
  }

  return true;
}

CUDAH bool Matrix::inverse(Matrix &output)
{
  if (mRows != mCols || mRows == 0 || mCols == 0)
    return false;

  if (mRows == 1)
  {
    if (mBuffer[0] != 0)
      output(0, 0) = 1 / mBuffer[0];
    else
      return false;
  }

  if (mRows == 2)
  {
    double det = at(0, 0) * at(1, 1) - at(0, 1) * at(1, 0);

    if (det != 0)
    {
      output(0, 0) = at(1, 1) / det;
      output(0, 1) = - at(0, 1) / det;

      output(1, 0) = - at(1, 0) / det;
      output(1, 1) = at(0, 0) / det;
    }
    else
    {
      return false;
    }
  }

  if (mRows == 3)
  {
    double det = at(0, 0) * at(1, 1) * at(2, 2) + at(0, 1) * at(1, 2) * at(2, 0) +
      at(1, 0) * at (2, 1) * at(0, 2) - at(0, 2) * at(1, 1) * at(2, 0) -
      at(0, 1) * at(1, 0) * at(2, 2) - at(0, 0) * at(1, 2) * at(2, 1);
    double idet = 1.0 / det;

    if (det != 0)
    {
      output(0, 0) = (at(1, 1) * at(2, 2) - at(1, 2) * at(2, 1)) * idet;
      output(0, 1) = - (at(0, 1) * at(2, 2) - at(0, 2) * at(2, 1)) * idet;
      output(0, 2) = (at(0, 1) * at(1, 2) - at(0, 2) * at(1, 1)) * idet;

      output(1, 0) = - (at(1, 0) * at(2, 2) - at(1, 2) * at(2, 0)) * idet;
      output(1, 1) = (at(0, 0) * at(2, 2) - at(0, 2) * at(2, 0)) * idet;
      output(1, 2) = - (at(0, 0) * at(1, 2) - at(0, 2) * at(1, 0)) * idet;

      output(2, 0) = (at(1, 0) * at(2, 1) - at(1, 1) * at(2, 0)) * idet;
      output(2, 1) = - (at(0, 0) * at(2, 1) - at(0, 1) * at(2, 0)) * idet;
      output(2, 2) = (at(0, 0) * at(1, 1) - at(0, 1) * at(1, 0)) * idet;
    }
    else
    {
      return false;
    }
  }

  return true;
}

CUDAH Matrix Matrix::col(int index)
{
  return Matrix(mRows, 1, mOffset * mCols, mBuffer + index * mOffset);
}

CUDAH Matrix Matrix::row(int index)
{
  return Matrix(1, mCols, mOffset, mBuffer + index * mCols * mOffset);
}

} // namespace gpu

#endif // GMATRIX_H_


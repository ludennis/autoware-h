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

#ifndef MATRIX_DEVICE_H_
#define MATRIX_DEVICE_H_

#include <ndt_matcher/ndt_gpu/matrix.h>

namespace gpu
{

class MatrixDevice : public Matrix
{
public:
  CUDAH MatrixDevice();
  MatrixDevice(int rows, int cols);
  CUDAH MatrixDevice(int rows, int cols, int offset, double *buffer);
  CUDAH bool IsEmpty();
  CUDAH MatrixDevice col(int index);
  CUDAH MatrixDevice row(int index);
  CUDAH void SetBuffer(double *buffer);
  void MemAlloc();
  void MemFree();

private:
  bool mFree;
};

CUDAH MatrixDevice::MatrixDevice()
{
  mRows = mCols = mOffset = 0;
  mBuffer = NULL;
  mFree = true;
}

CUDAH MatrixDevice::MatrixDevice(int rows, int cols, int offset, double *buffer)
{
  mRows = rows;
  mCols = cols;
  mOffset = offset;
  mBuffer = buffer;
  mFree = false;
}

CUDAH bool MatrixDevice::IsEmpty()
{
  return (mRows == 0 || mCols == 0 || mBuffer == NULL);
}

CUDAH MatrixDevice MatrixDevice::col(int index)
{
  return MatrixDevice(mRows, 1, mOffset * mCols, mBuffer + index * mOffset);
}

CUDAH MatrixDevice MatrixDevice::row(int index)
{
  return MatrixDevice(1, mCols, mOffset, mBuffer + index * mCols * mOffset);
}

CUDAH void MatrixDevice::SetBuffer(double *buffer)
{
  mBuffer = buffer;
}

class SquareMatrixDevice : public MatrixDevice
{
public:
  SquareMatrixDevice(int size);
};

} //namespace gpu

#endif // MATRIX_DEVICE_H_

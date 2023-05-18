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

#include <ndt_matcher/ndt_gpu/matrix_host.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

namespace gpu
{

MatrixHost::MatrixHost()
{
  mFree = false;
}

MatrixHost::MatrixHost(int rows, int cols)
{
  mRows = rows;
  mCols = cols;
  mOffset = 1;

  mBuffer = (double*)malloc(sizeof(double) * mRows * mCols * mOffset);
  memset(mBuffer, 0, sizeof(double) * mRows * mCols * mOffset);
  mFree = true;
}

MatrixHost::MatrixHost(int rows, int cols, int offset, double *buffer)
{
  mRows = rows;
  mCols = cols;
  mOffset = offset;
  mBuffer = buffer;
  mFree = false;
}

MatrixHost::MatrixHost(const MatrixHost& other)
{
  mRows = other.mRows;
  mCols = other.mCols;
  mOffset = other.mOffset;
  mFree = other.mFree;

  if (mFree)
  {
    mBuffer = (double*)malloc(sizeof(double) * mRows * mCols * mOffset);
    memcpy(mBuffer, other.mBuffer, sizeof(double) * mRows * mCols * mOffset);
  }
  else
  {
    mBuffer = other.mBuffer;
  }
}

extern "C" __global__ void CopyMatrixDevice(MatrixDevice input,
  MatrixDevice output)
{
  int row = threadIdx.x;
  int col = threadIdx.y;
  int rowsNum = input.rows();
  int colsNum = input.cols();

  if (row < rowsNum && col < colsNum)
    output(row, col) = input(row, col);
}

bool MatrixHost::MoveToGpu(MatrixDevice output)
{
  if (mRows != output.rows() || mCols != output.cols())
    return false;

  if (mOffset == output.offset())
  {
    checkCudaErrors(cudaMemcpy(output.buffer(), mBuffer,
      sizeof(double) * mRows * mCols * mOffset, cudaMemcpyHostToDevice));
    return true;
  }
  else
  {
    std::shared_ptr<double> tmp =
      AllocateCudaMemory<double>(mRows * mCols * mOffset);
    checkCudaErrors(cudaMemcpy(tmp.get(), mBuffer,
      sizeof(double) * mRows * mCols * mOffset, cudaMemcpyHostToDevice));
    MatrixDevice tmpOutput(mRows, mCols, mOffset, tmp.get());

    dim3 blockX(mRows, mCols, 1);
    dim3 gridX(1, 1, 1);
    CopyMatrixDevice<<<gridX, blockX>>>(tmpOutput, output);
    checkCudaErrors(cudaDeviceSynchronize());

    return true;
  }
}

bool MatrixHost::MoveToHost(MatrixDevice input)
{
  if (mRows != input.rows() || mCols != input.cols())
    return false;

  if (mOffset == input.offset())
  {
    checkCudaErrors(cudaMemcpy(mBuffer, input.buffer(),
      sizeof(double) * mRows * mCols * mOffset, cudaMemcpyDeviceToHost));
    return true;
  }
  else
  {
    std::shared_ptr<double> tmp =
      AllocateCudaMemory<double>(mRows * mCols * mOffset);
    MatrixDevice tmpOutput(mRows, mCols, mOffset, tmp.get());

    dim3 blockX(mRows, mCols, 1);
    dim3 gridX(1, 1, 1);
    CopyMatrixDevice<<<gridX, blockX>>>(input, tmpOutput);
    checkCudaErrors(cudaDeviceSynchronize());
    checkCudaErrors(cudaMemcpy(mBuffer, tmp.get(),
      sizeof(double) * mRows * mCols * mOffset, cudaMemcpyDeviceToHost));

    return true;
  }
}

MatrixHost &MatrixHost::operator=(const MatrixHost &other)
{
  mRows = other.mRows;
  mCols = other.mCols;
  mOffset = other.mOffset;
  mFree = other.mFree;

  if (mFree)
  {
    mBuffer = (double*)malloc(sizeof(double) * mRows * mCols * mOffset);
    memcpy(mBuffer, other.mBuffer, sizeof(double) * mRows * mCols * mOffset);
  }
  else
  {
    mBuffer = other.mBuffer;
  }

  return *this;
}

void MatrixHost::debug()
{
  for (int i = 0; i < mRows; i++)
  {
    for (int j = 0; j < mCols; j++)
    {
      std::cout << mBuffer[(i * mCols + j) * mOffset] << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

MatrixHost::~MatrixHost()
{
  if (mFree)
    free(mBuffer);
}


SquareMatrixHost::SquareMatrixHost(int size)
  : MatrixHost(size, size)
{}

} // namespace gpu

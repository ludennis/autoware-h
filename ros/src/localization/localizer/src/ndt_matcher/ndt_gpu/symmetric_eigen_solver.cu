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

#include <ndt_matcher/ndt_gpu/debug.h>
#include <ndt_matcher/ndt_gpu/symmetric_eigen_solver.h>

namespace gpu
{

SymmetricEigensolver3x3::SymmetricEigensolver3x3(int offset)
{
  mOffset = offset;

  checkCudaErrors(cudaMalloc(&mBuffer, sizeof(double) * 18 * mOffset));
  checkCudaErrors(cudaMalloc(&mMaxAbsElement, sizeof(double) * mOffset));
  checkCudaErrors(cudaMalloc(&mNorm, sizeof(double) * mOffset));
  checkCudaErrors(cudaMalloc(&mI02, sizeof(int) * 2 * mOffset));

  mEigenvectors = NULL;
  mEigenvalues = NULL;
  mInputMatrices = NULL;

  mIsCopied = false;
}

void SymmetricEigensolver3x3::SetInputMatrices(double *inputMatrices)
{
  mInputMatrices = inputMatrices;
}

void SymmetricEigensolver3x3::SetEigenvectors(double *eigenvectors)
{
  mEigenvectors = eigenvectors;
}

void SymmetricEigensolver3x3::SetEigenvalues(double *eigenvalues)
{
  mEigenvalues = eigenvalues;
}

double* SymmetricEigensolver3x3::GetBuffer() const
{
  return mBuffer;
}

void SymmetricEigensolver3x3::MemFree()
{
  if (!mIsCopied)
  {
    if (mBuffer != NULL)
    {
      checkCudaErrors(cudaFree(mBuffer));
      mBuffer = NULL;
    }

    if (mMaxAbsElement != NULL)
    {
      checkCudaErrors(cudaFree(mMaxAbsElement));
      mMaxAbsElement = NULL;
    }

    if (mNorm != NULL)
    {
      checkCudaErrors(cudaFree(mNorm));
      mNorm = NULL;
    }

    if (mI02 != NULL)
    {
      checkCudaErrors(cudaFree(mI02));
      mI02 = NULL;
    }
  }
}

} // namespace gpu

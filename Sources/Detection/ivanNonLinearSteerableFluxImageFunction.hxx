/*=========================================================================

Image-based Vascular Analysis Toolkit (IVAN)

Copyright (c) 2012, Iván Macía Oliver
Vicomtech Foundation, San Sebastián - Donostia (Spain)
University of the Basque Country, San Sebastián - Donostia (Spain)

All rights reserved

See LICENSE file for license details

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
IMPLIED WARRANTIES SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
SUCH DAMAGE.

==========================================================================*/
// File: ivanNonLinearSteerableFluxImageFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/07/06

#ifndef __ivanNonLinearSteerableFluxImageFunction_hxx
#define __ivanNonLinearSteerableFluxImageFunction_hxx

#include "ivanNonLinearSteerableFluxImageFunction.h"

#include <vnl/vnl_math.h>

namespace ivan
{
  
template <class TInputImage, class TOutput>
NonLinearSteerableFluxImageFunction<TInputImage,TOutput>
::NonLinearSteerableFluxImageFunction()
{
  m_RadiusFactor = 1.0; // default for this filter

}


template <class TInputImage, class TOutput>
NonLinearSteerableFluxImageFunction<TInputImage,TOutput>
::~NonLinearSteerableFluxImageFunction()
{
  
}


/** Print self method */
template <class TInputImage, class TOutput>
void
NonLinearSteerableFluxImageFunction<TInputImage,TOutput>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
    
}


template <class TInputImage, class TOutput>
typename NonLinearSteerableFluxImageFunction<TInputImage,TOutput>::OutputType
NonLinearSteerableFluxImageFunction<TInputImage,TOutput>
::EvaluateVesselnessAtContinuousIndex( const HessianTensorType & hessian,
  const ContinuousIndexType & cindex ) const
{
  typedef typename HessianTensorType::EigenVectorsMatrixType EigenVectorsMatrixType;
  typedef typename HessianTensorType::EigenValuesArrayType   EigenValuesArrayType;
    
  EigenVectorsMatrixType eigenVectors;
  EigenValuesArrayType   eigenValues;
  
  // We need the arrays in the form of an itk::Vector
  VectorType  firstEigenVector, secondEigenVector, thirdEigenVector;
  
  // Calculate eigenvalues and eigenvectors of Hessian matrix at the current location
  hessian.ComputeEigenAnalysis( eigenValues, eigenVectors );
  
  // Check that the hessian matrix is not a zero matrix. This avoids problems with NaN eigenvalues and eigenvectors
  double trace = 0.0;
  bool zeroTrace = true;
  unsigned int diag = 0; // diagonal position
  
  for( unsigned int i=0; i<TInputImage::GetImageDimension(); ++i, 
    diag += TInputImage::GetImageDimension()-i )
  {
    trace += hessian[diag];
    
    if( zeroTrace )
    {
      if( fabs( hessian[diag] ) > 1e-3 )
        zeroTrace = false;
    }
  }
  
  if( zeroTrace )
  {
    if( fabs( trace ) > 1e-3 )
      zeroTrace = false;
  }    
  
  if( zeroTrace )
    return itk::NumericTraits<OutputType>::Zero;
    
  // Recover eigenvectors from the matrix (first two rows)
  for( unsigned int i=0; i < TInputImage::GetImageDimension(); ++i )
  {
    firstEigenVector[i]  = eigenVectors( 0, i );
    secondEigenVector[i] = eigenVectors( 1, i );
    thirdEigenVector[i]  = eigenVectors( 2, i );
  }
  
  
  // Now comes the vesselness calculation
  
  PointType  currentPoint, currentPointShifted;
  OutputType firstFluxComponents[2], secondFluxComponents[2];
  OutputType firstFlux, secondFlux;
  
  typedef typename GradientFunctionType::OutputType VectorType;
  VectorType gradientVector;
  
  // Get the physical coordinates of the current index
  this->GetInputImage()->TransformContinuousIndexToPhysicalPoint( cindex, currentPoint );
    
  // Compute fluxes. We use as shift s = m_Sigma * m_RadiusFactor 
  
  for( unsigned int i=0; i<TInputImage::ImageDimension; ++i )
    currentPointShifted[i] = currentPoint[i] + this->m_RadiusFactor * this->m_Sigma * firstEigenVector[i];
  
  if( this->m_GradientFunction->IsInsideBuffer( currentPointShifted ) )
  {
    firstFluxComponents[0] = 0.0;
    gradientVector = this->m_GradientFunction->Evaluate( currentPointShifted );
    
    // The flux is the dot product of the surface vector and the gradient vector. The surface
    // vector is in this case one of the eigenvectors but in the opposite direction
    for( unsigned int i=0; i<TInputImage::ImageDimension; ++i )
      firstFluxComponents[0] -= gradientVector[i] * firstEigenVector[i]; // use minus for inward direction
  }
  else
    firstFluxComponents[0] = 0.0;
  
  if( firstFluxComponents[0] < 0.0 ) // only take the positive part of the flux
    firstFluxComponents[0] = 0.0;
  
  // Now shift in the other direction
  
  for( unsigned int i=0; i<TInputImage::ImageDimension; ++i )
    currentPointShifted[i] = currentPoint[i] - this->m_RadiusFactor * this->m_Sigma * firstEigenVector[i];
  
  if( this->m_GradientFunction->IsInsideBuffer( currentPointShifted ) )
  {
    firstFluxComponents[1] = 0.0;
    gradientVector = this->m_GradientFunction->Evaluate( currentPointShifted );
    
    for( unsigned int i=0; i<TInputImage::ImageDimension; ++i )
      firstFluxComponents[1] += gradientVector[i] * firstEigenVector[i];
  }
  else
    firstFluxComponents[1] = 0.0;
  
  if( firstFluxComponents[1] < 0.0 ) // only take the positive part of the flux
    firstFluxComponents[1] = 0.0;
  
  firstFlux = vnl_math_min( firstFluxComponents[0], firstFluxComponents[1] );
  
  // Now for the second flux
  
  for( unsigned int i=0; i<TInputImage::ImageDimension; ++i )
    currentPointShifted[i] = currentPoint[i] + this->m_RadiusFactor * this->m_Sigma * secondEigenVector[i];
  
  if( this->m_GradientFunction->IsInsideBuffer( currentPointShifted ) )
  {
    secondFluxComponents[0] = 0.0;
    gradientVector = this->m_GradientFunction->Evaluate( currentPointShifted );
    
    for( unsigned int i=0; i<TInputImage::ImageDimension; ++i )
      secondFluxComponents[0] -= gradientVector[i] * secondEigenVector[i]; // use minus for inward direction
  }
  else
    secondFluxComponents[0] = 0.0;
  
  if( secondFluxComponents[0] < 0.0 ) // only take the positive part of the flux
    secondFluxComponents[0] = 0.0;
  
  // Now shift in the other direction
  
  for( unsigned int i=0; i<TInputImage::ImageDimension; ++i )
    currentPointShifted[i] = currentPoint[i] - this->m_RadiusFactor * this->m_Sigma * secondEigenVector[i];
  
  if( this->m_GradientFunction->IsInsideBuffer( currentPointShifted ) )
  {
    secondFluxComponents[1] = 0.0;
    gradientVector = this->m_GradientFunction->Evaluate( currentPointShifted );
    
    for( unsigned int i=0; i<TInputImage::ImageDimension; ++i )
      secondFluxComponents[1] += gradientVector[i] * secondEigenVector[i];
  }
  else
    secondFluxComponents[1] = 0.0;
  
  if( secondFluxComponents[1] < 0.0 ) // only take the positive part of the flux
    secondFluxComponents[1] = 0.0;
  
  secondFlux = vnl_math_min( secondFluxComponents[0], secondFluxComponents[1] );
  
  
  // Now combine both values by taking again the minimum and obtain the final value
  
  return vnl_math_min( firstFlux, secondFlux );
}
  
} // end namespace ivan

#endif

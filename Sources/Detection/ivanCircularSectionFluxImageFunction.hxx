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
// File: ivanCircularSectionFluxImageFunction.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/07/06

#ifndef __ivanCircularSectionFluxImageFunction_hxx
#define __ivanCircularSectionFluxImageFunction_hxx

#include "ivanCircularSectionFluxImageFunction.h"

#include "itkNumericTraits.h"

#include <vnl/vnl_math.h>

namespace ivan
{
  
template <class TInputImage, class TOutput>
CircularSectionFluxImageFunction<TInputImage,TOutput>
::CircularSectionFluxImageFunction() :
  m_AdaptativeSampling( false ),
  m_UseNonLinearFlux( true ),
  m_NonLinearFluxFunction( AverageFluxFunction ),
  m_RadialResolution( 12 )  
{
  this->m_RadiusFactor = 1.0; // default for this filter
  this->ComputeIntervals();
}


template <class TInputImage, class TOutput>
CircularSectionFluxImageFunction<TInputImage,TOutput>
::~CircularSectionFluxImageFunction()
{
  
}


/** Print self method */
template <class TInputImage, class TOutput>
void
CircularSectionFluxImageFunction<TInputImage,TOutput>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  os << indent << "AdaptativeSampling: " << m_AdaptativeSampling << std::endl;
  os << indent << "UseNonLinearFlux: " << m_UseNonLinearFlux << std::endl;
  os << indent << "NonLinearFluxFunction: " << m_NonLinearFluxFunction << std::endl;
  os << indent << "RadialResolution: " << m_RadialResolution << std::endl;
}


template <class TInputImage, class TOutput>
void
CircularSectionFluxImageFunction<TInputImage,TOutput>
::SetAdaptativeSampling( bool adaptative )
{
  if( this->GetAdaptativeSampling() == adaptative )
    return;
  
  this->SetAdaptativeSampling( adaptative );  
  this->ComputeIntervals();
  this->Modified(); 
}


template <class TInputImage, class TOutput>
void
CircularSectionFluxImageFunction<TInputImage,TOutput>
::SetUseNonLinearFlux( bool nonLinear )
{
  if( this->GetAdaptativeSampling() == nonLinear )
    return;
    
  this->m_UseNonLinearFlux = nonLinear;
  
  if( this->m_RadialResolution % 2 != 0 ) // odd
  {
    itkWarningMacro( "RadialResolution is added 1 in order to be even since UseNonLinearFlux is ON." );
    ++this->m_RadialResolution;
    this->ComputeIntervals();
  }
  
  this->Modified(); 
}


template <class TInputImage, class TOutput>
void
CircularSectionFluxImageFunction<TInputImage,TOutput>
::SetRadialResolution( const unsigned int resolution )
{
  if( this->GetRadialResolution() == resolution && !this->GetAdaptativeSampling() )
    return;
  else if( this->GetAdaptativeSampling() )
  {
    this->SetAdaptativeSampling( false );
  }
  
  this->m_RadialResolution = resolution;
  
  if( this->m_RadialResolution % 2 != 0 ) // odd
  {
    itkWarningMacro( "RadialResolution is added 1 in order to be even since UseNonLinearFlux is ON." );
    ++this->m_RadialResolution;
  }
  
  this->ComputeIntervals();
  this->Modified();
}


template <class TInputImage, class TOutput>
void
CircularSectionFluxImageFunction<TInputImage,TOutput>
::SetSigma( double sigma )
{ 
  if( this->m_Sigma == sigma )
    return;
    
  this->m_Sigma = sigma;

  Superclass::SetSigma( sigma );
    
  if( this->GetAdaptativeSampling() )
  {
    this->ComputeIntervals();
    this->Modified(); 
  }
}


template <class TInputImage, class TOutput>
void
CircularSectionFluxImageFunction<TInputImage,TOutput>
::ComputeIntervals()
{
  if( this->GetAdaptativeSampling() )
  {
    this->m_RadialResolution = static_cast<unsigned int>
      ( vnl_math_rnd( 2.0 * vnl_math::pi * this->m_Sigma + 1.0 ) );
  }
  
  // Set the size of the arrays for storing radial boundariness values depending on the number of samples
  m_SinArray.SetSize( this->m_RadialResolution );
  m_CosArray.SetSize( this->m_RadialResolution );
    
  // Precalculate sin and cos values as these calculations are costly
  
  double angle = 0.0;
  double angleInc = vnl_math::pi * 2.0 / static_cast<double>( this->m_RadialResolution ); 
  
  for ( unsigned int i=0; i < this->m_RadialResolution; ++i )
  {
    m_SinArray[i] = sin( angle );
    m_CosArray[i] = cos( angle );
    angle += angleInc;
  }  
}


template <class TInputImage, class TOutput>
typename CircularSectionFluxImageFunction<TInputImage,TOutput>::OutputType
CircularSectionFluxImageFunction<TInputImage,TOutput>
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
  
  unsigned int        calculatedSamples = 0; // samples calculated for each circle
  PointType           currentPoint, currentCirclePoint;
  OutputType          finalFlux = 0.0;
  itk::Array<double>  radialFlux;
  const double        radius = this->m_RadiusFactor * this->m_Sigma;
    
  radialFlux.SetSize( this->m_RadialResolution );
  
  typedef typename GradientFunctionType::OutputType VectorType;
  VectorType gradientVector, inwardVector;
  
  // Get the physical coordinates of the current index
  this->GetInputImage()->TransformContinuousIndexToPhysicalPoint( cindex, currentPoint );
  
  // In this case we evaluate at the center only once
  //gradientVector = this->m_GradientFunction->Evaluate( currentPoint );
  
  // Calculate flux values in a circle defined by the two eigenvectors
  for( unsigned int i=0; i<this->m_RadialResolution; ++i )
  {
    radialFlux[i] = 0.0;
    
    for( unsigned int k=0; k<TInputImage::ImageDimension; ++k )
    {
      currentCirclePoint[k] = currentPoint[k] + radius * 
        ( this->m_CosArray[i] * firstEigenVector[k] + this->m_SinArray[i] * secondEigenVector[k] );
      inwardVector[k] = -this->m_CosArray[i] * firstEigenVector[k] - this->m_SinArray[i] * secondEigenVector[k];
    }
    
    if( this->m_GradientFunction->IsInsideBuffer( currentCirclePoint ) )
    {
      gradientVector = this->m_GradientFunction->Evaluate( currentCirclePoint );
      
      for( unsigned int k=0; k<TInputImage::ImageDimension; ++k )
        radialFlux[i] += gradientVector[k] * inwardVector[k];
    }       
  }
  
  // Now compute flux depending on linearity
  
  if( this->m_UseNonLinearFlux )
  {
    for( unsigned int i=0; i<this->m_RadialResolution; ++i )
      finalFlux += radialFlux[i];
      
    finalFlux /= (double)this->m_RadialResolution;
  }
  else
  {
    if( this->m_NonLinearFluxFunction == MinimumFluxFunction )
    {
      for( unsigned int i=0; i<this->m_RadialResolution/2; ++i )
        finalFlux += vnl_math_min( radialFlux[i], radialFlux[(this->m_RadialResolution/2)+i] );
    }
    else // average
    {
      for( unsigned int i=0; i<this->m_RadialResolution/2; ++i )
        finalFlux += ( radialFlux[i] + radialFlux[(this->m_RadialResolution/2)+i] ) / 2.0;      
    }
          
    finalFlux /= (double)(this->m_RadialResolution/2);
  }
      
  return finalFlux;
}

} // end namespace ivan

#endif

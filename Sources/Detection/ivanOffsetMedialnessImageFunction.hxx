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
// File: ivanOffsetMedialnessImageFunction.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: 
// Date: 2010/07/06

#ifndef __ivanOffsetMedialnessImageFunction_hxx
#define __ivanOffsetMedialnessImageFunction_hxx

#include "ivanOffsetMedialnessImageFunction.h"
#include "ivanGlobals.h"

#include "vnl/vnl_vector_fixed.h"


namespace ivan
{
  
template <class TInputImage, class TOutput, class TCoordRep>
OffsetMedialnessImageFunction<TInputImage,TOutput,TCoordRep>
::OffsetMedialnessImageFunction() :
  m_GradientSigma( 1.0 ),
  m_SymmetryCoefficient( 1.0 ),
  m_UseCentralBoundariness( false ),
  m_AdaptativeSampling( false ),
  m_AutoComputeSectionNormal( true ),
  m_RadialResolution( 12 ),
  //m_Threshold( 0.0 ),
  m_GradientImageFunctionType( OffsetMedialnessImageFunction::GradientNormalProjectionFunctionHyperIntense )
{
  this->m_SectionNormal.Fill( 0.0 );
  
  this->m_Radius = this->m_Sigma / 1.7320508; // sqrt(3.0)

  this->m_GradientFunction = GradientFunctionType::New();
  this->m_GradientFunction->NormalizeAcrossScaleOn();
  this->m_GradientFunction->UseImageSpacingOn();
  this->m_GradientFunction->SetInterpolationMode( GradientFunctionType::LinearInterpolation );
  this->m_GradientFunction->SetSigma( this->m_GradientSigma );
  
  this->m_GradientMagnitudeFunction = GradientMagnitudeFunctionType::New();
  this->m_GradientMagnitudeFunction->NormalizeAcrossScaleOn();
  this->m_GradientMagnitudeFunction->UseImageSpacingOn();
  this->m_GradientMagnitudeFunction->SetInterpolationMode( GradientMagnitudeFunctionType::LinearInterpolation );
  this->m_GradientMagnitudeFunction->SetSigma( this->m_GradientSigma );
  
  this->ComputeIntervals();
}


template <class TInputImage, class TOutput, class TCoordRep>
OffsetMedialnessImageFunction<TInputImage,TOutput,TCoordRep>
::~OffsetMedialnessImageFunction()
{
  
}


/** Print self method */
template <class TInputImage, class TOutput, class TCoordRep>
void
OffsetMedialnessImageFunction<TInputImage,TOutput,TCoordRep>
::PrintSelf( std::ostream& os, itk::Indent indent) const
{
  Superclass::PrintSelf( os, indent );
  
  os << indent << "GradientSigma: " << this->m_GradientSigma << std::endl;
  os << indent << "SymmetryCoefficient: " << this->m_SymmetryCoefficient << std::endl;
  os << indent << "Radius: " << this->m_Radius << std::endl;
  os << indent << "UseCentralBoundariness: " << this->m_UseCentralBoundariness << std::endl;
  os << indent << "AdaptativeSampling: " << this->m_AdaptativeSampling << std::endl;
  os << indent << "AutoComputeSectionNormal: " << this->m_AutoComputeSectionNormal << std::endl;
  os << indent << "SectionNormal: " << this->m_SectionNormal << std::endl;
  os << indent << "RadialResolution: " << this->m_RadialResolution << std::endl;
  os << indent << "CosArray: " << this->m_CosArray << std::endl;
  os << indent << "SinArray: " << this->m_SinArray << std::endl;
  //os << indent << "Threshold: " << this->m_Threshold << std::endl;
  
  os << indent << "GradientMagnitudeFunction: " << this->m_GradientMagnitudeFunction.GetPointer() << std::endl;
  this->m_GradientMagnitudeFunction->Print( os, indent.GetNextIndent() );
  
  os << indent << "GradientFunction: " << this->m_GradientFunction.GetPointer() << std::endl;
  this->m_GradientFunction->Print( os, indent.GetNextIndent() );
}


template <class TInputImage, class TOutput, class TCoordRep>
void
OffsetMedialnessImageFunction<TInputImage,TOutput,TCoordRep>
::SetAdaptativeSampling( bool adaptative )
{
  if( this->GetAdaptativeSampling() == adaptative )
    return;
  
  this->m_AdaptativeSampling = adaptative;  
  this->ComputeIntervals();
  this->Modified(); 
}


template <class TInputImage, class TOutput, class TCoordRep>
void
OffsetMedialnessImageFunction<TInputImage,TOutput,TCoordRep>
::SetRadialResolution( const unsigned int resolution )
{
  if( this->GetRadialResolution() == resolution && !this->GetAdaptativeSampling() )
    return;
  else if( this->GetAdaptativeSampling() )
  {
    this->SetAdaptativeSampling( false );
  }
  
  this->m_RadialResolution = resolution;
  
  this->ComputeIntervals();
  this->Modified();
}


template <class TInputImage, class TOutput, class TCoordRep>
void
OffsetMedialnessImageFunction<TInputImage,TOutput,TCoordRep>
::SetGradientSigma( double gradientSigma )
{ 
  if( this->m_GradientSigma == gradientSigma )
    return;
    
  this->m_GradientSigma = gradientSigma;
    
  this->m_GradientFunction->SetSigma( gradientSigma );
  this->m_GradientMagnitudeFunction->SetSigma( gradientSigma );
}


template <class TInputImage, class TOutput, class TCoordRep>
void
OffsetMedialnessImageFunction<TInputImage,TOutput,TCoordRep>
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


/** Set the input image */
template <class TInputImage, class TOutput, class TCoordRep>
void
OffsetMedialnessImageFunction<TInputImage,TOutput,TCoordRep>
::SetInputImage( const InputImageType * ptr )
{
  Superclass::SetInputImage( ptr );
  
  this->m_GradientFunction->SetInputImage( ptr );
  this->m_GradientMagnitudeFunction->SetInputImage( ptr );
}


template <class TInputImage, class TOutput, class TCoordRep>
void
OffsetMedialnessImageFunction<TInputImage,TOutput,TCoordRep>
::Initialize()
{
  Superclass::Initialize();
  
  this->m_GradientFunction->Initialize();
  this->m_GradientMagnitudeFunction->Initialize();
}


template <class TInputImage, class TOutput, class TCoordRep>
void
OffsetMedialnessImageFunction<TInputImage,TOutput,TCoordRep>
::ComputeIntervals()
{
  if( this->GetAdaptativeSampling() )
  {
    this->m_RadialResolution = static_cast<unsigned int>
      ( vnl_math_rnd( 2.0 * vnl_math::pi * this->m_Sigma + 1.0 ) );
  }
  
  // Set the size of the arrays for storing radial boundariness values depending on the number of samples
  this->m_SinArray.SetSize( this->m_RadialResolution );
  this->m_CosArray.SetSize( this->m_RadialResolution );
    
  // Precalculate sin and cos values as these calculations are costly
  
  double angle = 0.0;
  double angleInc = vnl_math::pi * 2.0 / static_cast<double>( this->m_RadialResolution ); 
  
  for ( unsigned int i=0; i < this->m_RadialResolution; ++i )
  {
    this->m_SinArray[i] = sin( angle );
    this->m_CosArray[i] = cos( angle );
    angle += angleInc;
  }  
}


template <class TInputImage, class TOutput, class TCoordRep>
typename OffsetMedialnessImageFunction<TInputImage,TOutput,TCoordRep>::OutputType
OffsetMedialnessImageFunction<TInputImage,TOutput,TCoordRep>
::EvaluateVesselnessAtContinuousIndex( const HessianTensorType & hessian,
  const ContinuousIndexType & cindex ) const
{
  // The section plane will be formed by the first and second base vectors, which are normal to the normal vector
  
  typedef vnl_vector_fixed<double,3>  VnlVectorType;  
  VnlVectorType  firstBaseVector, secondBaseVector;
  
  if( this->m_AutoComputeSectionNormal )
  {
    typedef typename HessianTensorType::EigenVectorsMatrixType  EigenVectorsMatrixType;
    typedef typename HessianTensorType::EigenValuesArrayType    EigenValuesArrayType;
        
    EigenVectorsMatrixType eigenVectors;
    EigenValuesArrayType   eigenValues;
    
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
      firstBaseVector[i]       = eigenVectors( 0, i );
      if( firstBaseVector[i] == -0.0 )
        firstBaseVector[i] = 0.0;
      
      secondBaseVector[i]      = eigenVectors( 1, i );
      if( secondBaseVector[i] == -0.0 )
        secondBaseVector[i] = 0.0;
      
      this->m_SectionNormal[i] = eigenVectors( 2, i );
      if( this->m_SectionNormal[i] == -0.0 )
        this->m_SectionNormal[i] = 0.0;
    }
  
    // Do not process those pixels whose first two eigenvalues are not negative
    /*    
    if( m_FilterByEigenValues )
    {
      if( eigenValues[0] >= 0.0f || eigenValues[1] >= 0.0f )
      { 
      	if( maskPtr.IsNotNull() )
          ++maskIt;
        continue;
      }
    }*/
  }
  else // the section normal is provided
  {
    // Verify that the section was provided
    assert( this->m_SectionNormal[0] + this->m_SectionNormal[1] + this->m_SectionNormal[2] != 0.0 );
    
    VnlVectorType normalVector;
    normalVector.copy_in( this->m_SectionNormal.GetDataPointer() );
    
    // Calculate an orthonormal base of vectors in the plane of the section given the normal vector
    ComputePlaneBasisVectorsFromNormal( normalVector, firstBaseVector, secondBaseVector );
  }
       
  ////////////////////////////
  // Medialness calculation //
  
  unsigned int calculatedSamples = 0; // samples calculated for each circle
  OutputType medialness; // calculated medialness at current point and scale
  double weightExponent; // exponent of weighting factors for medialness calculations
  double averageRadialMedialness; // measure of average medialness without weighting
  double centralBoundariness; // boundariness at the current point
  
  PointType currentPoint, currentCirclePoint;
  //IndexType currentCircleIndex;
  
  // Arrays used for medialness calculation
  
  itk::Array<double>   radialMedialness; // stores all values of gradient in the circle with given radius
  itk::Array<double>   radialMedialnessWeights; // stores all weighting factors in the circle with given radius

  radialMedialness.SetSize( this->m_RadialResolution );
  radialMedialnessWeights.SetSize( this->m_RadialResolution );
  
  // Initialize the arrays
  radialMedialness.Fill( 0.0f );
  radialMedialnessWeights.Fill( 0.0f );

  // Get the physical coordinates of the current index
  this->GetInputImage()->TransformContinuousIndexToPhysicalPoint( cindex, currentPoint );
  
  typename InputImageType::SpacingType spacing = this->GetInputImage()->GetSpacing();

  // Calculate points in a circle defined by the two eigenvectors with r = m_Radius
  for( unsigned int i=0; i<this->m_RadialResolution; ++i )
  {
    // Evaluate and store the value of the gradient in a circle 
    
    for( int k=0; k < TInputImage::GetImageDimension(); ++k )
      currentCirclePoint[k] = currentPoint[k] + this->m_Radius * 
        ( this->m_CosArray[i] * firstBaseVector[k] + this->m_SinArray[i] * secondBaseVector[k] );
    
    if( this->m_GradientMagnitudeFunction->IsInsideBuffer( currentCirclePoint ) ) // the check may be made with any filter
    {
      if( this->m_GradientImageFunctionType == GradientNormalProjectionFunctionHyperIntense ||
          this->m_GradientImageFunctionType == GradientNormalProjectionFunctionHypoIntense )
      {
        // Project the gradient into the radial direction, that is, compute the dot product of both vectors
               
        GradientFunctionType::OutputType gradient = this->m_GradientFunction->Evaluate( currentCirclePoint );
               
        radialMedialness[i] = 0.0; 
        
        // For hyperintese vessels use minus sign, since we are going from higher to lower values 
        double gradientSign = ( ( this->m_GradientImageFunctionType == GradientNormalProjectionFunctionHyperIntense ) ? -1.0 : 1.0 );
        
        for( int k=0; k < TInputImage::GetImageDimension(); ++k )
          radialMedialness[i] += gradientSign * gradient[k] * ( this->m_CosArray[i] * firstBaseVector[k] + this->m_SinArray[i] * secondBaseVector[k] );
      }
      else // GradientMagnitudeFunction
      {
        radialMedialness[i] = this->m_GradientMagnitudeFunction->Evaluate( currentCirclePoint );        
      }
        
      ++calculatedSamples;
    }
    else
      radialMedialness[i] = 0.0;    
  }
  
  medialness = 0.0;
  
  if( calculatedSamples > 0 )
  {
    if( m_SymmetryCoefficient == 1.0 ) // speed-up calculations for the simplest (default) case
    {
      for ( unsigned int i=0; i<this->m_RadialResolution; ++i )
        medialness += radialMedialness[i];  
    }    
    else
    {
      averageRadialMedialness = radialMedialness.one_norm() 
        / static_cast<double>( calculatedSamples );
        
      for ( unsigned int i=0; i<this->m_RadialResolution; ++i )
      {
        if( !radialMedialness[i] )
          continue;
        
        // Calculate b coefficients
        weightExponent = ( 1.0 - radialMedialness[i] / averageRadialMedialness ) 
          / m_SymmetryCoefficient;
        radialMedialnessWeights[i] = vcl_exp( -0.5 * weightExponent * weightExponent );
        medialness += radialMedialnessWeights[i] * radialMedialness[i];
      }    
    }

    medialness /= static_cast<double>( calculatedSamples );
  }
          
  // Now calculate final medialness using an adaptative threshold that depends on the central boundariness
  // (value of gradient at the central point). Vessel centers are supposed to have a low value of boundariness
  // and thus this is penalized. A threshold is introduced here for noise etc.
  
  if( this->m_UseCentralBoundariness )
  {
    centralBoundariness = this->m_GradientMagnitudeFunction->Evaluate( currentPoint );
	  
	  medialness /= centralBoundariness;

    //if( medialness - centralBoundariness < this->m_Threshold ) // this was the original implementation but I don't like it
      //medialness = 0.0f;
  }
  //else if( medialness < this->m_Threshold )
    //medialness = 0.0f;
    
  //return medialness - centralBoundariness;
  //return centralBoundariness;
  return medialness;
}
  
} // end namespace ivan

#endif

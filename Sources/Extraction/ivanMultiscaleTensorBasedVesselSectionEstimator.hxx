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
// File: ivanMultiscaleTensorBasedVesselSectionEstimator.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2010/02/03


#ifndef __ivanMultiscaleTensorBasedVesselSectionEstimator_hxx
#define __ivanMultiscaleTensorBasedVesselSectionEstimator_hxx

#include "ivanMultiscaleTensorBasedVesselSectionEstimator.h"

#ifdef _DEBUG
  #include "itkLinearInterpolateImageFunction.h"
#endif

#include <vcl_cassert.h>

#define SCALE_TOLERANCE 1e-2


namespace ivan
{

template <class TImage, class TScaledImageFunction, class TCenterline, class TMetricsCalculator>
MultiscaleTensorBasedVesselSectionEstimator<TImage,TScaledImageFunction,TCenterline,TMetricsCalculator>
::MultiscaleTensorBasedVesselSectionEstimator()
{
  
}


template <class TImage, class TScaledImageFunction, class TCenterline, class TMetricsCalculator>
MultiscaleTensorBasedVesselSectionEstimator<TImage,TScaledImageFunction,TCenterline, TMetricsCalculator>
::~MultiscaleTensorBasedVesselSectionEstimator()
{

}


template <class TImage, class TScaledImageFunction, class TCenterline, class TMetricsCalculator>
void
MultiscaleTensorBasedVesselSectionEstimator<TImage,TScaledImageFunction,TCenterline, TMetricsCalculator>
::Compute()
{
#ifdef _DEBUG
  typedef itk::LinearInterpolateImageFunction<TImage>  InterpolatorType;
  InterpolatorType::Pointer interpolator = InterpolatorType::New();
  interpolator->SetInputImage( m_Image );  
#endif

  assert( this->m_Image.IsNotNull() );
  
  SectionPointer                      currentSection;
  PointType                           centerPoint;
  typename SectionType::VectorType    sectionNormal;
  StructureTensorType                 tensor;
    
  typedef typename StructureTensorType::EigenVectorsMatrixType   EigenVectorsMatrixType;
  typedef typename StructureTensorType::EigenValuesArrayType     EigenValuesArrayType;
  typedef EigenValuesArrayType                                   EigenVectorArrayType;
  
  EigenVectorsMatrixType eigenVectors;
  EigenValuesArrayType   eigenValues;
  
  double dotProduct;
  double currentScale = 0.0;
  
  // If the section range is not specified take all the sections
  if( this->m_SectionRange[0] == 0 && this->m_SectionRange[1] == 0 )
    this->m_SectionRange[1] = this->GetCenterline()->size() - 1;
  
  for( unsigned int i = this->m_SectionRange[0]; i <= m_SectionRange[1]; ++i )
  {
    std::cout << "Calculating section " << i+1 << " out of " << 
      ( this->m_SectionRange[1] - this->m_SectionRange[0] + 1 ) << " ... " << std::endl;
    
    currentSection = this->GetCenterline()->at(i);
    
    for( unsigned int dim = 0; dim < ImageType::GetImageDimension(); ++dim )
      centerPoint[dim] = currentSection->GetCenter()[dim];

#ifdef _DEBUG
    if( interpolator->IsInsideBuffer( centerPoint ) )
      std::cout << "CenterValue: " << interpolator->Evaluate( centerPoint ) << std::endl;
    else
      std::cout << "CenterValue: out of bounds" << std::endl;
#endif

    // Get scale for hessian calculation and then calculate/interpolate hessian
    currentScale = this->GetScaleAt( i, centerPoint );
    this->GetInterpolatedTensor( centerPoint, currentScale, tensor );
    
    // Calculate eigenvalues and eigenvectors of Hessian matrix at the current location
    tensor.ComputeEigenAnalysis( eigenValues, eigenVectors );
    
    dotProduct = 0.0;
        
    // WARNING: CHECK THAT THESE ARE THE TWO MOST NEGATIVE EIGENVALUED-EIGENVECTORS
    // THE NORMAL CORRESPONDS TO THE THIRD (LARGEST POSITIVE EIGENVALUED-EIGENVECTOR)
    for( unsigned int dim = 0; dim < ImageType::GetImageDimension(); ++dim )
    {
      sectionNormal[dim] = eigenVectors( 2, dim ); // third eigenvector
      if( i )
        dotProduct += this->GetCenterline()->at(i-1)->GetNormal()[dim] * sectionNormal[dim];
    }
    
    // If this is not the first normal, avoid changing sign in direction from the previous normal
    if( dotProduct < 0.0 )
    {
      for( unsigned int dim = 0; dim < ImageType::GetImageDimension(); ++dim )
        sectionNormal[dim] = -sectionNormal[dim]; 
    }
      
    currentSection->SetNormal( sectionNormal );
    currentSection->SetScale( currentScale );
    // !!! WARNING: THIS MAY BE ONLY AN APPROXIMATION IN SOME CASES WHEN THE SCALE CHOSEN
    // IS PROPORTIONAL TO THE RADIUS ASSUMED WITH A GAUSSIAN CIRCULAR CROSS-SECTION (SEE Krissian et al. 2000)
    currentSection->SetRadius( currentScale * 1.732 ); // might be an approximation (Krissian)

#ifdef _DEBUG
    std::cout << "Done." << std::endl;
#endif 

    this->m_CurrentScale = currentScale;
  }
}


template <class TImage, class TScaledImageFunction, class TCenterline, class TMetricsCalculator>
void 
MultiscaleTensorBasedVesselSectionEstimator<TImage,TScaledImageFunction,TCenterline, TMetricsCalculator>
::GetInterpolatedTensor( const PointType & center, double scale, StructureTensorType & tensor )
{
  assert( this->m_ScaledImageFunctionContainer.size() && this->m_Scales.size() == this->m_ScaledImageFunctionContainer.size() );
  
  if( m_ScaledImageFunctionContainer.size() == 1 )
  {
    // We cannot interpolate
    tensor = this->m_ScaledImageFunctionContainer[0]->Evaluate( center );
    return;
  }
  else if( scale <= this->m_Scales[0] )
  {
    tensor = this->m_ScaledImageFunctionContainer[0]->Evaluate( center );
    return;
  }
  else if( scale >= this->m_Scales[ this->m_ScaledImageFunctionContainer.size()-1 ] )
  {
    tensor = this->m_ScaledImageFunctionContainer[ this->m_ScaledImageFunctionContainer.size()-1 ]->Evaluate( center );
    return;
  }
  else
  {
    unsigned int idx = 0;
    
    while( m_Scales[idx+1] < scale )
      ++idx;
      
    // Try not to interpolate if sigma is within some tolerance of our discrete set of scales
    
    if( vnl_math_abs( scale - this->m_Scales[idx] ) <= SCALE_TOLERANCE )
    {
      tensor = this->m_ScaledImageFunctionContainer[idx]->Evaluate( center );
      return;
    }
    else if( vnl_math_abs( scale - this->m_Scales[idx+1] ) <= SCALE_TOLERANCE )
    {
      tensor = this->m_ScaledImageFunctionContainer[idx+1]->Evaluate( center );
      return;     
    }
    
    // Now idx has the position of the lower bound
    // Values are generated either linearly or logarithmically, but between two generated values the
    // interpolation is linear, even if the values are generated logarithmically, because the values
    // themselves are not logarithms
        
    StructureTensorType lowerTensor = this->m_ScaledImageFunctionContainer[idx]->Evaluate( center );
    StructureTensorType upperTensor = this->m_ScaledImageFunctionContainer[idx+1]->Evaluate( center );
    
    double linearFactor = ( scale - this->m_Scales[idx] ) / ( this->m_Scales[idx+1] - this->m_Scales[idx] );
    
    for( unsigned int i=0; i < tensor.GetNumberOfComponents(); ++i )
      tensor[i] = lowerTensor[i] +  linearFactor * ( upperTensor[i] - lowerTensor[i] );    
  }  
}


template <class TImage, class TScaledImageFunction, class TCenterline, class TMetricsCalculator>
void 
MultiscaleTensorBasedVesselSectionEstimator<TImage,TScaledImageFunction,TCenterline,TMetricsCalculator>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
 
}

} // end namespace ivan

#endif // __ivanMultiscaleTensorBasedVesselSectionEstimator_hxx

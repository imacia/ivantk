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
// File: ivanFixedScaleHessianBasedVesselSectionEstimator.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: generic node of an acyclic graph structure
// Date: 2010/06/04


#ifndef __ivanFixedScaleHessianBasedVesselSectionEstimator_hxx
#define __ivanFixedScaleHessianBasedVesselSectionEstimator_hxx

#include "ivanFixedScaleHessianBasedVesselSectionEstimator.h"


namespace ivan
{

template <class TImage, class TCenterline, class TMetricsCalculator>
FixedScaleHessianBasedVesselSectionEstimator<TImage,TCenterline,TMetricsCalculator>
::FixedScaleHessianBasedVesselSectionEstimator() :
  m_Scale( 1.0 )
{
  m_HessianFunction = HessianFunctionType::New();
  this->RecomputeKernel();
  
  this->m_CurrentScale = this->m_Scale; // scale is fixed
}


template <class TImage, class TCenterline, class TMetricsCalculator>
FixedScaleHessianBasedVesselSectionEstimator<TImage,TCenterline, TMetricsCalculator>
::~FixedScaleHessianBasedVesselSectionEstimator()
{

}

template <class TImage, class TCenterline, class TMetricsCalculator>
void
FixedScaleHessianBasedVesselSectionEstimator<TImage,TCenterline, TMetricsCalculator>
::SetScale( double scale )
{
  if( this->m_Scale != scale )
  {
    this->m_Scale = scale;
    this->RecomputeKernel();
  } 
}


template <class TImage, class TCenterline, class TMetricsCalculator>
void
FixedScaleHessianBasedVesselSectionEstimator<TImage,TCenterline, TMetricsCalculator> 
::SetImage( ImageType *image )
{
  if( this->m_Image.GetPointer() != image )
  {
    this->m_Image = image;
    this->RecomputeKernel();
  } 
}


template <class TImage, class TCenterline, class TMetricsCalculator>
void
FixedScaleHessianBasedVesselSectionEstimator<TImage,TCenterline, TMetricsCalculator>
::Compute()
{
  assert( this->m_Image.IsNotNull() );
  
  typename SectionType::Pointer             currentSection;
  typename SectionType::VectorType          sectionNormal;
  typename HessianFunctionType::PointType   centerPoint;
  typename HessianFunctionType::OutputType  hessian;
    
  typedef typename HessianTensorType::EigenVectorsMatrixType EigenVectorsMatrixType;
  typedef typename HessianTensorType::EigenValuesArrayType   EigenValuesArrayType;
    
  EigenVectorsMatrixType eigenVectors;
  EigenValuesArrayType   eigenValues;
  
  double dotProduct;
  
  // If the section range is not specified take all the sections
  if( this->m_SectionRange[0] == 0 && this->m_SectionRange[1] == 0 )
    this->m_SectionRange[1] = this->GetCenterline()->size() - 1;
  
  for( unsigned int i = this->m_SectionRange[0]; i <= m_SectionRange[1]; ++i )
  {
    currentSection = this->GetCenterline()->at(i);
    
    for( unsigned int dim = 0; dim < ImageType::GetImageDimension(); ++dim )
      centerPoint[dim] = currentSection->GetCenter()[dim];
    
    if( m_HessianFunction->IsInsideBuffer( centerPoint ) )
    {
      hessian = m_HessianFunction->Evaluate( centerPoint );
    }
    else
    {
      itkWarningMacro( "Point out of bounds. Returning zero normal and radius" );
      sectionNormal.Fill(0.0);
      currentSection->SetNormal( sectionNormal );
      currentSection->SetRadius( 0.0 );

      return;
    }
    
    // Calculate eigenvalues and eigenvectors of Hessian matrix at the current location
    hessian.ComputeEigenAnalysis( eigenValues, eigenVectors );
    
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
    currentSection->SetRadius( this->m_Scale / sqrt( 1.732 ) ); // see Krissian et al. for Gaussian tubes
  }  
}


template <class TImage, class TCenterline, class TMetricsCalculator>
void
FixedScaleHessianBasedVesselSectionEstimator<TImage,TCenterline, TMetricsCalculator>
::RecomputeKernel()
{
  m_HessianFunction->SetSigma( this->m_Scale );
  m_HessianFunction->SetInputImage( this->m_Image );
  m_HessianFunction->SetNormalizeAcrossScale( true );
  m_HessianFunction->SetUseImageSpacing( true );
  m_HessianFunction->Initialize();  
}


template <class TImage, class TCenterline, class TMetricsCalculator>
void 
FixedScaleHessianBasedVesselSectionEstimator<TImage,TCenterline,TMetricsCalculator>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  os << indent << "Scale: " << m_Scale << std::endl;
  os << indent << "Image: " << m_Image.GetPointer() << std::endl;
}

} // end namespace ivan

#endif // __ivanFixedScaleHessianBasedVesselSectionEstimator_hxx

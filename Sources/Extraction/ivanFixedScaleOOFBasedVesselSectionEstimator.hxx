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
// File: ivanFixedScaleOOFBasedVesselSectionEstimator.hxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: generic node of an acyclic graph structure
// Date: 2010/06/04


#ifndef __ivanFixedScaleOOFBasedVesselSectionEstimator_hxx
#define __ivanFixedScaleOOFBasedVesselSectionEstimator_hxx

#include "ivanFixedScaleOOFBasedVesselSectionEstimator.h"


namespace ivan
{

template <class TImage, class TVectorField, class TCenterline, class TMetricsCalculator>
FixedScaleOOFBasedVesselSectionEstimator<TImage,TVectorField,TCenterline,TMetricsCalculator>
::FixedScaleOOFBasedVesselSectionEstimator() :
  m_GradientSigma( 1.0 ),
  m_Radius( 3.0 )
{
  this->m_OOFFunction = OOFFunctionType::New();
  this->m_CurrentScale = this->m_Radius; // scale is fixed
}


template <class TImage, class TVectorField, class TCenterline, class TMetricsCalculator>
FixedScaleOOFBasedVesselSectionEstimator<TImage,TVectorField,TCenterline,TMetricsCalculator>
::~FixedScaleOOFBasedVesselSectionEstimator()
{

}


template <class TImage, class TVectorField, class TCenterline, class TMetricsCalculator>
void
FixedScaleOOFBasedVesselSectionEstimator<TImage,TVectorField,TCenterline,TMetricsCalculator> 
::SetImage( ImageType *image )
{
  if( this->m_Image.GetPointer() != image )
  {
    this->m_Image = image;
  } 
}


template <class TImage, class TVectorField, class TCenterline, class TMetricsCalculator>
void
FixedScaleOOFBasedVesselSectionEstimator<TImage,TVectorField,TCenterline,TMetricsCalculator>
::Compute()
{
  assert( this->m_Image.IsNotNull() );
  
  typename SectionType::Pointer             currentSection;
  typename SectionType::VectorType          sectionNormal;
  typename SectionType::PointType           sectionCenter;
    
  FluxMatrixType           fluxMatrix;
  EigenVectorsMatrixType   eigenVectors;
  EigenValuesArrayType     eigenValues;
  
  // Calculate eigenvalues and eigenvectors of flux matrix at the current location
  //fluxMatrix.ComputeEigenAnalysis( eigenValues, eigenVectors );
    
  double dotProduct;
  
  // If the section range is not specified take all the sections
  if( this->m_SectionRange[0] == 0 && this->m_SectionRange[1] == 0 )
    this->m_SectionRange[1] = this->GetCenterline()->size() - 1;
  
  for( unsigned int i = this->m_SectionRange[0]; i <= this->m_SectionRange[1]; ++i )
  {
    currentSection = this->GetCenterline()->at(i);
    
    for( unsigned int dim = 0; dim < ImageType::GetImageDimension(); ++dim )
      sectionCenter[dim] = currentSection->GetCenter()[dim];
      
    fluxMatrix = this->m_OOFFunction->EvaluateFluxMatrix( sectionCenter );
    
    // Calculate eigenvalues and eigenvectors of the flux matrix at the current location
    fluxMatrix.ComputeEigenAnalysis( eigenValues, eigenVectors );
    
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
  }  
}


template <class TImage, class TVectorField, class TCenterline, class TMetricsCalculator>
void
FixedScaleOOFBasedVesselSectionEstimator<TImage,TVectorField,TCenterline,TMetricsCalculator>
::Initialize()
{
  typename OOFFunctionType::VectorFieldPointer vectorField = this->m_OOFFunction->GetVectorField();
  
  vectorField->SetInputImage( this->m_Image );
  vectorField->SetSigma( this->m_GradientSigma );
  vectorField->NormalizeAcrossScaleOn();
  vectorField->UseImageSpacingOn();
  vectorField->Initialize();
  
  this->m_OOFFunction->SetRadius( this->m_Radius );
  this->m_OOFFunction->SetInputImage( this->m_Image );
  this->m_OOFFunction->Initialize(); // this computes the sphere grid
}


template <class TImage, class TVectorField, class TCenterline, class TMetricsCalculator>
void 
FixedScaleOOFBasedVesselSectionEstimator<TImage,TVectorField,TCenterline,TMetricsCalculator>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  os << indent << "Radius: " << this->m_Radius << std::endl;
  os << indent << "GradientSigma: " << this->m_GradientSigma << std::endl;
  os << indent << "OOFFunction: " << this->m_OOFFunction.GetPointer() << std::endl;
  this->m_OOFFunction->Print( os, indent.GetNextIndent() );
}

} // end namespace ivan

#endif // __ivanFixedScaleOOFBasedVesselSectionEstimator_hxx

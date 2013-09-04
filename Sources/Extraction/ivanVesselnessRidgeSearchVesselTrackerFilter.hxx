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
// File: ivanVesselnessRidgeSearchVesselTrackerFilter.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2012/02/12


#ifndef __ivanVesselnessRidgeSearchVesselTrackerFilter_hxx
#define __ivanVesselnessRidgeSearchVesselTrackerFilter_hxx

#include "ivanVesselnessRidgeSearchVesselTrackerFilter.h"
#include "ivanGlobals.h"
#include "ivanVesselBranchNode.h"


namespace ivan
{
  
template <class TInputImage, class TOutputVessel, class TVesselnessFunction>
VesselnessRidgeSearchVesselTrackerFilter<TInputImage,TOutputVessel,TVesselnessFunction>
::VesselnessRidgeSearchVesselTrackerFilter() :
  m_MaximumSearchDistance( 5.0 ),
  m_RadialResolution( 5 ),
  m_AngularResolution( 8 ),
  m_AdaptativeSampling( false )
{
  this->ComputeIntervals();
}


template <class TInputImage, class TOutputVessel, class TVesselnessFunction>
void
VesselnessRidgeSearchVesselTrackerFilter<TInputImage,TOutputVessel,TVesselnessFunction>
::SetAdaptativeSampling( bool adaptative )
{
  if( this->GetAdaptativeSampling() == adaptative )
    return;
  
  this->SetAdaptativeSampling( adaptative );  
  this->ComputeIntervals();
  this->Modified(); 
}


template <class TInputImage, class TOutputVessel, class TVesselnessFunction>
void
VesselnessRidgeSearchVesselTrackerFilter<TInputImage,TOutputVessel,TVesselnessFunction>
::SetAngularResolution( const unsigned int resolution )
{
  if( this->GetAngularResolution() == resolution && !this->GetAdaptativeSampling() )
    return;
  else if( this->GetAdaptativeSampling() )
  {
    this->SetAdaptativeSampling( false );
  }
  
  this->m_AngularResolution = resolution;
  
  this->ComputeIntervals();
  this->Modified();
}


template <class TInputImage, class TOutputVessel, class TVesselnessFunction>
void
VesselnessRidgeSearchVesselTrackerFilter<TInputImage,TOutputVessel,TVesselnessFunction>
::ComputeIntervals()
{
  if( this->GetAdaptativeSampling() )
  {
    typename CenterlineType::Pointer  centerline = this->GetCurrentBranch()->GetCenterline();
    typename SectionType::Pointer     section = centerline->at( this->m_BranchPointIndex );
      
    this->m_AngularResolution = static_cast<unsigned int>
      ( vnl_math_rnd( 2.0 * vnl_math::pi * section->GetRadius() + 1.0 ) );
  }
  
  // Set the size of the arrays for storing radial boundariness values depending on the number of samples
  this->m_SinArray.SetSize( this->m_AngularResolution );
  this->m_CosArray.SetSize( this->m_AngularResolution );
    
  // Precalculate sin and cos values as these calculations are costly
  
  double angle = 0.0;
  double angleInc = vnl_math::pi * 2.0 / static_cast<double>( this->m_AngularResolution ); 
  
  for ( unsigned int i=0; i < this->m_AngularResolution; ++i )
  {
    this->m_SinArray[i] = sin( angle );
    this->m_CosArray[i] = cos( angle );
    angle += angleInc;
  }  
}


template <class TInputImage, class TOutputVessel, class TVesselnessFunction>
void 
VesselnessRidgeSearchVesselTrackerFilter<TInputImage,TOutputVessel,TVesselnessFunction>
::Search()
{
  typename CenterlineType::Pointer  centerline = this->GetCurrentBranch()->GetCenterline();
  typename SectionType::Pointer     section = centerline->at( this->m_BranchPointIndex );
    
    
  // Perform a radial sampling at the local point and plane in order to search for the maximum value of
  // vesselness. This is similar to a directed optimization procedure.
  
  double samplingDistance = this->m_MaximumSearchDistance / (double)this->m_RadialResolution;
  
  double currentMax, currentValue;
  //itk::Array<double>   radialValues;
  
  //radialValues.SetSize( this->m_RadialResolution );
  
  
  // Initialize vesselness function taking into account current scale as calculated for the section.
  // Do this only if the scale changed from the previous point and it is not the first point
  
  InputImageType *inputImage = static_cast<InputImageType*>( this->ProcessObject::GetInput(0) );

  if( !this->m_BranchPointIndex )
  {
    this->m_VesselnessFunctionInitializer->Initialize( this->m_VesselnessFunction.GetPointer(), 
      inputImage, section->GetScale() );
  }
  else if( centerline->at( this->m_BranchPointIndex-1 ) != centerline->at( this->m_BranchPointIndex ) )
  {
    this->m_VesselnessFunctionInitializer->Initialize( this->m_VesselnessFunction.GetPointer(), 
      inputImage, section->GetScale() );    
  }
  
  if( this->m_VesselnessFunction->IsInsideBuffer( section->GetCenter() ) )
    currentMax = this->m_VesselnessFunction->Evaluate( section->GetCenter() );
  else
    currentMax = 0.0;
  
  typedef vnl_vector_fixed<double,3>  VnlVectorType;  
  VnlVectorType  firstBaseVector, secondBaseVector, normalVector;
  
  normalVector.copy_in( section->GetNormal().GetDataPointer() );
  
  // Calculate an orthonormal base of vectors in the plane of the section given the normal vector
  ComputePlaneBasisVectorsFromNormal( normalVector, firstBaseVector, secondBaseVector );
  
  typename SectionType::PointType currentCirclePoint, currentMaxPoint;
  currentMaxPoint = section->GetCenter();

  bool reestimate = false;
  
  // Calculate points in a circle defined by the two base vectors
  for( unsigned int i=0; i<this->m_RadialResolution; ++i )
  {
    //radialValues.Fill(0.0);
       
    for( unsigned int j=0; j<this->m_AngularResolution; ++j )
    {
      // Evaluate and store the value of the gradient in a circle 
      
      for( unsigned int k=0; k < TInputImage::GetImageDimension(); ++k )
        currentCirclePoint[k] = section->GetCenter()[k] + ( (i+1) * samplingDistance ) * 
          ( this->m_CosArray[j] * firstBaseVector[k] + this->m_SinArray[j] * secondBaseVector[k] );
          
      if( this->m_VesselnessFunction->IsInsideBuffer( currentCirclePoint ) )
        currentValue = this->m_VesselnessFunction->Evaluate( currentCirclePoint );
      else
        currentValue = 0.0;
        
      if( currentValue > currentMax )
      {
        reestimate = true;
        currentMax = currentValue;
        currentMaxPoint = currentCirclePoint;
      }
    }
  }

  if( reestimate )
  {  
    //typename CenterlineType::PointType newCenter = currentMaxPoint;
    section->SetCenter( currentMaxPoint );
    
    // Recompute section at the new center
    this->m_SectionEstimator->Compute();
  }
}


template <class TInputImage, class TOutputVessel, class TVesselnessFunction>
void 
VesselnessRidgeSearchVesselTrackerFilter<TInputImage,TOutputVessel,TVesselnessFunction>
::PrintSelf(std::ostream& os, itk::Indent indent) const
{
  Superclass::PrintSelf(os, indent);
    
  os << indent << "MaximumSearchDistance: " << this->m_MaximumSearchDistance << std::endl;
  os << indent << "RadialResolution  : " << this->m_RadialResolution << std::endl;
  os << indent << "AngularResolution : " << this->m_AngularResolution << std::endl;
  os << indent << "AdaptativeSampling : " << this-> m_AdaptativeSampling << std::endl;
}

} // end namespace ivan

#endif

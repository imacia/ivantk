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
// File: ivanVesselTrackerFilter.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2009/02/06


#ifndef __ivanVesselTrackerFilter_hxx
#define __ivanVesselTrackerFilter_hxx

#include "ivanVesselTrackerFilter.h"

namespace ivan
{
  
/**
 *
 */
template <class TInputImage, class TOutputVessel>
VesselTrackerFilter<TInputImage,TOutputVessel>
::VesselTrackerFilter() :
  m_EnableSearchStage( true ),
  m_InvertDirection( false ),
  m_InitialStepSize( 1.0 ),
  m_BranchPointIndex(0)
{
  
}


/**
 *
 */
template <class TInputImage, class TOutputVessel>
void
VesselTrackerFilter<TInputImage,TOutputVessel>
::SetSectionEstimator( SectionEstimatorType * sectionEstimator )
{ 
  this->m_SectionEstimator = sectionEstimator;
  this->Modified();
}


/**
 *
 */
template <class TInputImage, class TOutputVessel>
void
VesselTrackerFilter<TInputImage,TOutputVessel>
::Initialize()
{
  // Make sure we have at least a BranchNode already created
  
  OutputVesselPointer               vesselGraph = this->GetOutput(0);
  typename CenterlineType::Pointer  centerline;
  
  // Create the first VesselBranchNode if necessary
  if( !vesselGraph->GetRootNode() )
  {
    this->m_CurrentBranch = BranchNodeType::New();
    vesselGraph->SetRootNode( this->m_CurrentBranch );
  }
  else
  {
    this->m_CurrentBranch = dynamic_cast<BranchNodeType*>( vesselGraph->GetRootNode().GetPointer() );
    if( this->m_CurrentBranch.IsNull() )
      itkExceptionMacro( "Root node of vessel graph is not a branch node." );         
  }
  
  // Assign the current centerline to the section estimator
  this->m_SectionEstimator->SetCenterline( this->m_CurrentBranch->GetCenterline() );
  
  this->m_BranchPointIndex = 0;
}


/**
 *
 */
template <class TInputImage, class TOutputVessel>
void
VesselTrackerFilter<TInputImage,TOutputVessel>
::GenerateData()
{
  //ImageConstPointer  inputImage = this->GetInput();
  //OutputVesselPointer outputPtr = this->GetOutput(0);
  
  this->Initialize();
  
  while( !this->Finished() )
  {
    // Create a new section
    typename SectionType::Pointer newSection = SectionType::New();
    newSection->SetCenter( this->m_CurrentPoint );
    m_CurrentBranch->GetCenterline()->push_back( newSection );
        
    this->Turn();
    
    if( this->GetEnableSearchStage() )
      this->Search();
    
    this->Measure();
    this->Step();
          
#ifdef _DEBUG
    // BREAK FOR DEBUG PURPOSES SO WE KNOW WHAT IS HAPPENING
    if( this->m_BranchPointIndex == 140 )
      break;
#endif
  }
}


/**
 *
 */
template <class TInputImage, class TOutputVessel>
void
VesselTrackerFilter<TInputImage,TOutputVessel>
::Turn()
{
  assert( this->m_SectionEstimator.IsNotNull() );
  
  // This computes the section normal which estimates the current vessel direction
  this->m_SectionEstimator->SetSection( this->m_BranchPointIndex );
  this->m_SectionEstimator->Compute();  
}


/**
 *
 */
template <class TInputImage, class TOutputVessel>
void
VesselTrackerFilter<TInputImage,TOutputVessel>
::Step()
{
  assert( m_SectionEstimator.IsNotNull() );
  
  SectionType *section = this->m_CurrentBranch->GetCenterline()->at( this->m_BranchPointIndex );
  
  double stepSize = this->m_InitialStepSize;
  
  if( this->m_InvertDirection )
    stepSize = -stepSize;  
  
  // By default use regular step size
  for( unsigned int dim = 0; dim < InputImageType::GetImageDimension(); ++dim )
  {
    this->m_PreviousPoint[dim] = this->m_CurrentPoint[dim];
    this->m_CurrentPoint[dim]  = this->m_PreviousPoint[dim] + stepSize * section->GetNormal()[dim];
  }
  
  ++this->m_BranchPointIndex;
}


/**
 *
 */
template <class TInputImage, class TOutputVessel>
bool
VesselTrackerFilter<TInputImage,TOutputVessel>
::Finished()
{
  assert( this->m_EndCondition.IsNotNull() );
  
  return this->m_EndCondition->Finished();
}


/**
 *
 */
template <class TInputImage, class TOutputVessel>
void 
VesselTrackerFilter<TInputImage,TOutputVessel>
::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf( os, indent );
  
  os << indent << "EnableSearchStage: " << this->m_EnableSearchStage << std::endl;
  os << indent << "InitialStepSize: " << this->m_InitialStepSize << std::endl;
  os << indent << "PreviousPoint: " << this->m_PreviousPoint << std::endl;
  os << indent << "CurrentPoint: " << this->m_CurrentPoint << std::endl;
  os << indent << "BranchPointIndex: " << this->m_BranchPointIndex << std::endl;
}

} // end namespace ivan

#endif

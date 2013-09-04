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
// File: ivanOptimizedVesselnessBasedSearchVesselTrackerFilter.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2009/02/06


#ifndef __ivanOptimizedVesselnessBasedSearchVesselTrackerFilter_hxx
#define __ivanOptimizedVesselnessBasedSearchVesselTrackerFilter_hxx

#include "ivanOptimizedVesselnessBasedSearchVesselTrackerFilter.h"
#include "ivanVesselBranchNode.h"


namespace ivan
{
  
/**
 *
 */
template <class TInputImage, class TOutputVessel, class TVesselnessFunction, class TCostFunction>
OptimizedVesselnessBasedSearchVesselTrackerFilter<TInputImage,TOutputVessel,TVesselnessFunction,TCostFunction>
::OptimizedVesselnessBasedSearchVesselTrackerFilter()
{
  this->m_CostFunction = CostFunctionType::New();
}


/**
 *
 */
template <class TInputImage, class TOutputVessel, class TVesselnessFunction, class TCostFunction>
void 
OptimizedVesselnessBasedSearchVesselTrackerFilter<TInputImage,TOutputVessel,TVesselnessFunction,TCostFunction>
::Search()
{
  // !!! THE COMMENTED CODE SEARCHES FOR THE FIRST BRANCH BUT THIS MAY NOT BE CORRECT
  /*typename OutputVesselPointer vesselGraph = this->GetOutput(0);
  
  typedef  VesselBranchNode<CenterlineType>  BranchNodeType;
  typename BranchNodeType::Pointer  branchNode = 
    dynamic_cast<BranchNodeType*>( vesselGraph->GetRootNode().GetPointer() );
  
  if( branchNode.IsNull() )
    itkExceptionMacro( "Root node of vessel graph is not a branch node." );
  
  typename CenterlineType::Pointer  centerline = branchNode->GetCenterline();
  typename SectionType::Pointer     section = centerline->at( this->m_BranchIndex );*/ 
  
  // Pass the current estimated center and normal to the cost function so we can search for a
  // better center point in the plane based on vesselness
  
  typename CenterlineType::Pointer  centerline = this->GetCurrentBranch()->GetCenterline();
  typename SectionType::Pointer     section = centerline->at( this->m_BranchPointIndex );
  
  
  this->m_CostFunction->SetPlaneNormal( section->GetNormal() );
  this->m_CostFunction->SetPlaneCenter( section->GetCenter() );   
  
  // Declare the initial and last position as parameters for the optimizer
  itk::Optimizer::ParametersType initialPosition = itk::Optimizer::ParametersType
    ( this->m_CostFunction->GetNumberOfParameters() );
  itk::Optimizer::ParametersType finalParameters;
  
  // Set the initial position for the optimizer. The initial position is zero for alpha and beta, 
  // which corresponds exactly with the position of the current center point
  for( unsigned int i=0; i < this->m_CostFunction->GetNumberOfParameters(); ++i )
    initialPosition[i] = 0.0;
    
  this->m_Optimizer->SetInitialPosition( initialPosition );
  this->m_Optimizer->StartOptimization(); 
	finalParameters = this->m_Optimizer->GetCurrentPosition();
	
	// Set the new center point for the section      
	typename SectionType::CenterPointType centerPoint;
	
	for( unsigned int i=0; i<InputImageDimension; ++i )
	{
	  // Update current point (not previous!)
	  this->m_CurrentPoint[i] = this->m_CurrentPoint[i] + 
	    finalParameters[0] * this->m_CostFunction->GetPlaneFirstBaseVector()[i] +
	    finalParameters[1] * this->m_CostFunction->GetPlaneSecondBaseVector()[i];
	}
	
	section->SetCenter( this->m_CurrentPoint );
}


/**
 *
 */
template <class TInputImage, class TOutputVessel, class TVesselnessFunction, class TCostFunction>
void 
OptimizedVesselnessBasedSearchVesselTrackerFilter<TInputImage,TOutputVessel,TVesselnessFunction,TCostFunction>
::PrintSelf(std::ostream& os, itk::Indent indent) const
{
  Superclass::PrintSelf(os, indent);
  
}

} // end namespace ivan

#endif

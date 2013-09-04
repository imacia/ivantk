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
// File     : itkRadiusEstimationVesselNetworkFilter.hxx
// Author   : Iv�n Mac�a (imacia@vicomtech.org)
// Date     : January 2007
// Revision : 1 
// VICOMTech, Spain
// http://www.vicomtech.org


#ifndef __ivanRadiusEstimationVesselNetworkFilter_hxx
#define __ivanRadiusEstimationVesselNetworkFilter_hxx

#include "itkRadiusEstimationVesselNetworkFilter.h"
#include "itkImageRegionConstIterator.h"
#include "itkNormalVariateGenerator.h"
#include "itkOnePlusOneEvolutionaryOptimizer.h"
#include "itkImageToCircleFitCostFunction.h"
#include "itkLinearInterpolateImageFunction.h"

namespace ivan
{
  
/**
 *
 */
template <class TInputNetwork, class TOutputNetwork, class TSourceImage>
RadiusEstimationVesselNetworkFilter<TInputNetwork,TOutputNetwork,TSourceImage>
::RadiusEstimationVesselNetworkFilter()
{
  // Modify superclass default values, can be overridden by subclasses
  this->SetNumberOfRequiredInputs(2);
}


/**
 *
 */
template <class TInputNetwork, class TOutputNetwork, class TSourceImage>
void
RadiusEstimationVesselNetworkFilter<TInputNetwork,TOutputNetwork,TSourceImage>
::SetSourceImage( const SourceImageType * sourceImage )
{
  this->SetNthInput(1, const_cast<SourceImageType*>( sourceImage ));
}


/**
 *
 */
template <class TInputNetwork, class TOutputNetwork, class TSourceImage>
const typename RadiusEstimationVesselNetworkFilter<TInputNetwork,TOutputNetwork,TSourceImage>::SourceImageType * 
RadiusEstimationVesselNetworkFilter<TInputNetwork,TOutputNetwork,TSourceImage>
::GetSourceImage() const
{
  return (static_cast<const TSourceImage*>(this->ProcessObject::GetInput(1)));
}


/**
 *
 */
template <class TInputNetwork, class TOutputNetwork, class TSourceImage>
void
RadiusEstimationVesselNetworkFilter<TInputNetwork,TOutputNetwork,TSourceImage>
::GenerateData()
{
  // Get the input an output pointers
  typename TInputNetwork::ConstPointer inputPtr  = this->GetInput();
  typename TOutputNetwork::Pointer     outputPtr = this->GetOutput(0);
  typename TSourceImage::Pointer       sourcePtr = dynamic_cast<TSourceImage*>
    ( this->ProcessObject::GetInput(1) );
    
  std::cout << "Estimating optimized radius and section..." << std::endl;
  
  const unsigned int ImageDimension = TSourceImage::GetImageDimension(); 
  
  // Get access to the vessel branches
  typename TInputNetwork::VesselBranchIterator                branchesIt;
    
  // Create a gradient magnitude image function for gradient estimation
  
  typedef typename TInputNetwork::ScalePixelType   InputScalePixelType;
  typedef typename TOutputNetwork::ScalePixelType  OutputScalePixelType;
   
  // Variables for optimization of radius and section
  
  typedef itk::Statistics::NormalVariateGenerator  GeneratorType;
  GeneratorType::Pointer generator = GeneratorType::New();
  
  typedef itk::OnePlusOneEvolutionaryOptimizer   OptimizerType;
  OptimizerType::Pointer     optimizer = OptimizerType::New();
  optimizer->MaximizeOn();
  optimizer->SetNormalVariateGenerator( generator );
  optimizer->SetEpsilon( 1.0 );
  optimizer->SetMaximumIteration( 1000 );

  typedef itk::ImageToCircleFitCostFunction<SourceImageType>    CostFunctionType;

  typedef itk::LinearInterpolateImageFunction
		<TSourceImage,typename CostFunctionType::CoordinateRepresentationType>  InterpolatorType;
  typename InterpolatorType::Pointer interpolator = InterpolatorType::New();

  typename CostFunctionType::Pointer  costFunction = CostFunctionType::New();
  costFunction->SetInterpolator( interpolator );
  costFunction->SetImage( sourcePtr );
  costFunction->SetSymmetryCoefficient(0.5);
  
  optimizer->SetCostFunction( costFunction );
  
  OptimizerType::ParametersType initialPosition = OptimizerType::ParametersType
    ( costFunction->GetNumberOfParameters() );
  OptimizerType::ParametersType finalParameters;

	// Previous normal and new estimated normal of section plane
	typedef typename TOutputNetwork::VesselBranchType OutputVesselBranchType;
  typename OutputVesselBranchType::NormalVectorType  normal;
  
  // Variables for optimization results    
  double finalRadius, bestValue;
  unsigned int numberOfIterations;
  
  typename SourceImageType::PointType     point; // current point
  typename SourceImageType::IndexType     index; // current index
  	
  // ImageRegion related variables
  typename SourceImageType::RegionType             vesselPointRegion;
  typename SourceImageType::RegionType::SizeType   vesselPointSize;
  typename SourceImageType::RegionType::IndexType  vesselPointStartIndex;
  typename SourceImageType::RegionType::SizeType::SizeValueType   vesselPointBaseSize;
  
  // Iterators for vessel branches and vessel points 
  typename TInputNetwork::VesselBranchConstIterator  branchIt;
  typename TInputNetwork::VesselBranchType
  	::VesselPointConstIterator     inPointIt;
  typename TInputNetwork::VesselBranchType
  	::VesselPointIterator          outPointIt;
  		
  // Clear output first as we will insert the points again
  outputPtr->Clear();
  
  // Set the output spacing too
  outputPtr->SetSpacing( inputPtr->GetSpacing() );
    		  		
  // Iterate through the branches
  for( branchIt = inputPtr->Begin(); branchIt != inputPtr->End(); ++branchIt )
  {
  	// First make a copy of the point itself and insert it in the output
	  typename TInputNetwork::VesselBranchPointer outBranch = (*branchIt)->MakeCopy();
	  	
	  // Iterate through the points in the branches
	  for( inPointIt = (*branchIt)->Begin(), outPointIt = outBranch->Begin();
	    inPointIt != (*branchIt)->End(); ++inPointIt, ++outPointIt )
	  {
	  	index = (*inPointIt)->GetIndex();
	    
	    for( unsigned int k=0; k<SourceImageType::GetImageDimension(); ++k )
	      point[k] = sourcePtr->GetSpacing()[k] * index[k]; // voxel center 
	    
	    // Estimate a base size for the region. This avoids that the optimizer
      // tries to estimate a radius that is too large limiting the search space   
      vesselPointBaseSize = static_cast<typename SourceImageType::SizeType::SizeValueType>
        ( ceil( 1.732050807 * (*inPointIt)->GetScale() ) + 1.0 ) + 4;
    
      if( vesselPointBaseSize % 2 == 0 )
        vesselPointBaseSize += 1;
	              
	    // Set the region and central point for optimization
	    
	    // Update the sizes
	    vesselPointSize[0] = vesselPointSize[1] = vesselPointSize[2] = vesselPointBaseSize;
	    
	    // Make sure the size is not negative           
	    for( unsigned int k=0; k<SourceImageType::GetImageDimension(); ++k )
	    {
	      vesselPointStartIndex[k] = index[k];
	      vesselPointStartIndex[k] -= ( vesselPointSize[k] - 1 ) / 2;
	      if( vesselPointStartIndex[k] < 0 )
	      {
	        vesselPointSize[k] -= vesselPointStartIndex[k];
	        vesselPointStartIndex[k] = 0;
	      }	      
	    }
	    
	    vesselPointRegion.SetSize( vesselPointSize );
	    vesselPointRegion.SetIndex( vesselPointStartIndex );
	    
	    /////////////////////////////////////////////////
	    // Optimization for estimating radius and section
	    
	    generator->Initialize(12345);
	    optimizer->Initialize( 10 );
	    
	    costFunction->SetRegion( vesselPointRegion );
	    costFunction->SetCenter( point );
	    costFunction->Initialize();
	    
	    normal = (*inPointIt)->GetNormal();
	    normal.Normalize();
	    
	    // Initialize the optimizer with sqrt(3)*sigma as radius and the third eigenvector as the estimated normal
	    initialPosition[0] = 1.732050807 * (*inPointIt)->GetScale();
	    optimizer->SetInitialPosition( initialPosition );
	    	    
	    try 
	    { 
	      optimizer->StartOptimization(); 
	    } 
	    catch (itk::ExceptionObject& excpt)
	    {
	      std::cerr << "EXCEPTION CAUGHT!" << excpt.GetDescription() << std::endl;
	      return;
	    }  
	          
	    finalParameters = optimizer->GetCurrentPosition();
	    finalRadius = finalParameters[0];
	    numberOfIterations = optimizer->GetCurrentIteration();
	    bestValue = optimizer->GetValue();
	          
	    std::cout << "Point " << index << std::endl;
	    std::cout << " Radius     = " << finalRadius  << std::endl;
	    std::cout << " Iterations = " << numberOfIterations << std::endl;
	    std::cout << " Best value = " << bestValue          << std::endl;
	    std::cout << std::endl;
	    
	    (*outPointIt)->SetRadius( finalRadius );
	    (*outPointIt)->SetScale( finalRadius / 1.732050807 );
	  } // end for pointIt
	  
	  // Push the branch into the output VesselNetwork
	  outputPtr->PushBack( outBranch );
    
  } // end for branchIt
}


/**
 *
 */
template <class TInputNetwork, class TOutputNetwork, class TSourceImage>
void 
RadiusEstimationVesselNetworkFilter<TInputNetwork,TOutputNetwork,TSourceImage>
::PrintSelf(std::ostream& os, itk::Indent indent) const
{
  Superclass::PrintSelf(os, indent);

}


} // end namespace ivan

#endif

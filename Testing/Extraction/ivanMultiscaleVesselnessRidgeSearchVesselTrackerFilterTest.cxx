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
// File: ivanMultiscaleRidgeSearchVesselTrackerFilterTest.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: tests a vessel tracker filter, using a multi-scale approach with a search based
//   on a given vesselness function. The components used are:
//   - Section Estimator: multiscale, adaptive, Hessian-based
//   - End Condition: max iterations
//   - Vesselness Function: offset medialness function (multi-scale)
//   - Search Stage: find ridges of vesselness in the estimated plane to reestimate
// Date: 2012/02/12

#include "ivanVesselGraph.h"
#include "ivanCircularVesselSection.h"
#include "ivanVesselnessRidgeSearchVesselTrackerFilter.h"
#include "ivanMaxIterationsVesselTrackerEndCondition.h"
#include "ivanOffsetMedialnessImageFunction.h"
#include "ivanOffsetMedialnessImageFunctionInitializer.h"
#include "ivanMultiscaleAdaptiveHessianBasedVesselSectionEstimator.h"

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIterator.h"
#include "itkLinearInterpolateImageFunction.h"


#include <fstream>


int main( int argc, const char *argv[] )
{
  if( argc < 10 )
  {
    std::cerr << "Usage: " << argv[0] << "inputImage outputTextFile numberOfScales minScale maxScale logarithmicScales=0"
      " seedCoord_x seedCoord_y seedCoord_z [seedCoordIsPhysical=0] [invertDirection=0] [maxIterations]" << std::endl;
    return EXIT_FAILURE;
  }

  typedef short           PixelType;
  typedef float           RealPixelType;
  typedef unsigned char   MaskPixelType;
  const unsigned int Dimension = 3;

  typedef itk::Image<PixelType,Dimension>       ImageType;
  typedef itk::Image<MaskPixelType,Dimension>   MaskImageType;
  typedef itk::Image<RealPixelType,Dimension>   RealImageType;

  typedef itk::SymmetricSecondRankTensor< RealPixelType, Dimension > HessianPixelType;
  typedef itk::Image< HessianPixelType, Dimension >                  HessianImageType;
  
  typedef itk::Point<double,Dimension>          PointType;
  typedef ivan::CircularVesselSection
    <Dimension>                                 VesselSectionType;
  typedef ivan::VesselCenterline
    <unsigned int, VesselSectionType>           CenterlineType;

  typedef ivan::VesselGraph<CenterlineType>               VesselGraphType;
  typedef ivan::VesselBranchNode<CenterlineType>          BranchNodeType;
    
  typedef ivan::MaxIterationsVesselTrackerEndCondition    EndConditionType;

  typedef itk::ImageFileReader<ImageType>  ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName( argv[1] );

  try
  {
    reader->Update();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "EXCEPTION CAUGHT!!! " << excpt.GetDescription();
    return EXIT_FAILURE;
  }

  // Create a mask image with the non-zero values

  MaskImageType::Pointer mask = MaskImageType::New();
  mask->SetSpacing( reader->GetOutput()->GetSpacing() );
  mask->SetOrigin(  reader->GetOutput()->GetOrigin() );
  mask->SetDirection( reader->GetOutput()->GetDirection() );
  mask->SetRegions( reader->GetOutput()->GetRequestedRegion() );
  mask->Allocate();
  mask->FillBuffer(0);

  typedef itk::ImageRegionConstIterator<ImageType>  ConstIteratorType;
  typedef itk::ImageRegionIterator<MaskImageType>   MaskIteratorType;

  ConstIteratorType it ( reader->GetOutput(), reader->GetOutput()->GetRequestedRegion() );
  MaskIteratorType  oit( mask, reader->GetOutput()->GetRequestedRegion() );

  it.GoToBegin();
  oit.GoToBegin();

  while( !it.IsAtEnd() )
  { 
    if( it.Get() > 0 )
      oit.Set( itk::NumericTraits<MaskPixelType>::max() );

    ++it;
    ++oit;
  }
  
  
  // Create the vesselness filter
  
  typedef ivan::OffsetMedialnessImageFunction<ImageType,double>    MetricFunctionType;
  

  // Declare the type of vessel tracker

  typedef ivan::VesselnessRidgeSearchVesselTrackerFilter
    <ImageType, VesselGraphType, MetricFunctionType>               VesselTrackerType;
  
  VesselTrackerType::Pointer vesselTracker = VesselTrackerType::New();
  vesselTracker->SetInput( reader->GetOutput() );


  // Provide an initializer that will be used both for the multi-scale section estimator and
  // for the vesselness used in the search stage. At the search stage, this will be used
  // for setting the appropiate scale and reinitializing the vesselness if necessary

  typedef ivan::OffsetMedialnessImageFunctionInitializer
    <ImageType>                                                    VesselnessFunctionInitializerType;
    
  VesselnessFunctionInitializerType::Pointer initializer = VesselnessFunctionInitializerType::New();
	initializer->SetScaleSelectionMethod( VesselnessFunctionInitializerType::UseScaleAsRadius );
  initializer->SetHessianSigma( 1.0 );

  vesselTracker->SetVesselnessFunctionInitializer( initializer );
  
  
  // Set the section estimator

  typedef ivan::MultiscaleAdaptiveHessianBasedVesselSectionEstimator
    <ImageType,MetricFunctionType,CenterlineType>                  SectionEstimatorType;

  SectionEstimatorType::Pointer sectionEstimator = SectionEstimatorType::New();
  sectionEstimator->SetImage( reader->GetOutput() );
  
  // The metric we used may be initialized as most scale-space functions. This way we don't need to
  // access the container with the metric functions for each scale and initialize them one by one. 
  // Here we simply reuse the same initializer used for the search stage.
    
  sectionEstimator->SetMetricFunctionInitializer( initializer );
  
  if( (bool)atoi( argv[6] ) )
    sectionEstimator->SetScaleStepMethod( SectionEstimatorType::LogarithmicScaleSteps );
  else
    sectionEstimator->SetScaleStepMethod( SectionEstimatorType::EquispacedScaleSteps );
  
  unsigned int numScales = atof( argv[3] );
  double       minScale = atof( argv[4] );
  double       maxScale = atof( argv[5] );

  sectionEstimator->SetNumberOfScales( numScales );
  sectionEstimator->SetMinimumScale( minScale );
  sectionEstimator->SetMaximumScale( maxScale );
  sectionEstimator->Initialize();
  
  vesselTracker->SetSectionEstimator( sectionEstimator );

  
  // Set the end condition
  
  EndConditionType::Pointer endCondition = EndConditionType::New();

  bool invertDirection = false;

  if( argc > 11 )
    invertDirection = (bool)atoi( argv[11] );

  vesselTracker->SetInvertDirection( invertDirection );

  unsigned int maxIterations = 20;
 
  if( argc > 12 )
    maxIterations = atoi( argv[12] );

  endCondition->SetMaxIterations( maxIterations );
  vesselTracker->SetEndCondition( endCondition.GetPointer() );

  
  // Set the initial position
  
  bool seedCoordIsPhysical = false;
    
  if( argc > 10 )
    seedCoordIsPhysical = (bool)atoi( argv[10] );

  ImageType::IndexType index;
  ImageType::PointType point;
  
  ImageType::Pointer image = reader->GetOutput();
  
  if( !seedCoordIsPhysical )
  {
    index[0] = atoi( argv[7] );
    index[1] = atoi( argv[8] );
    index[2] = atoi( argv[9] );

    image->TransformIndexToPhysicalPoint( index, point );    
  }
  else
  {
    point[0] = atof( argv[7] );
    point[1] = atof( argv[8] );
    point[2] = atof( argv[9] );

    image->TransformPhysicalPointToIndex( point, index );
  }

  std::cout << "Start value: " << image->GetPixel( index ) << std::endl;

  vesselTracker->SetStartingPoint( point );
  
  
  // Set the search parameters
  
  vesselTracker->SetRadialResolution(5);
  vesselTracker->SetAngularResolution(8);
  vesselTracker->SetMaximumSearchDistance( minScale );
  //vesselTracker->SetMaximumSearchDistance( maxScale * 0.333 );

  try
  {
    vesselTracker->Update();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "EXCEPTION CAUGHT!!! " << excpt.GetDescription();
    return EXIT_FAILURE;
  }

  VesselGraphType::Pointer vesselGraph = vesselTracker->GetOutput();
   
  // Computed number of sections
  BranchNodeType::Pointer  vesselBranch = static_cast<BranchNodeType*>
    ( vesselGraph->GetRootNode().GetPointer() );

  std::cout << "Number of computed sections: " << vesselBranch->GetCenterline()->Size() << std::endl;
  
  std::ofstream fileout;
  fileout.open( argv[2] );
  fileout << "Index Center(x y z) Normal (x y z) Scale" << std::endl;

  CenterlineType::Pointer centerline = vesselBranch->GetCenterline();
  CenterlineType::SectionType *currentSection;

  for( unsigned int i=0; i<centerline->Size(); ++i )
  {
    currentSection  = centerline->at(i);
    
    fileout << currentSection->GetCenter()[0] << " "
            << currentSection->GetCenter()[1] << " "
            << currentSection->GetCenter()[2] << " "
            << currentSection->GetNormal()[0] << " "
            << currentSection->GetNormal()[1] << " "
            << currentSection->GetNormal()[2] << " "
            << currentSection->GetScale()
            << std::endl;    
  }

  fileout.close();

  return EXIT_SUCCESS; 
}

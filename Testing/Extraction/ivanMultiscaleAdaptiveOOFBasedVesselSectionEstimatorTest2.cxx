/*=========================================================================

Image-based Vascular Analysis Toolkit (IVAN)

Copyright (c) 2012, Ivan Macia Oliver
Vicomtech Foundation, San Sebastian - Donostia (Spain)
University of the Basque Country, San Sebastian - Donostia (Spain)

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
// File: ivanMultiscaleAdaptiveOOFBasedVesselSectionEstimatorTest2.cxx
// Author: Ivan Macia (imacia@vicomtech.org)
// Description: tests the MultiscaleAdaptiveOOFBasedVesselSectionEstimator class with a tracker filter.
//   From an initial seed, a tracking procedure is started. At each point, a vesselness metric
//   is estimated for a range of discrete scales. The scale which gives a maximum is the one 
//   chosen for the section estimator.
//   This example uses as metric the OptimallyOrientedFluxVesselnessImageFunction itself
// Date: 2012/02/08

#include "ivanVesselGraph.h"
#include "ivanCircularVesselSection.h"
#include "ivanVesselTrackerFilter.h"
#include "ivanMaxIterationsVesselTrackerEndCondition.h"
#include "ivanDiscreteGradientGaussianImageFunction.h"
#include "ivanOptimallyOrientedFluxVesselnessImageFunction.h"
#include "ivanOptimallyOrientedFluxVesselnessImageFunctionInitializer.h"
#include "ivanMultiscaleAdaptiveOOFBasedVesselSectionEstimator.h"

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIterator.h"
#include "itkLinearInterpolateImageFunction.h"


#include <fstream>


int main( int argc, const char *argv[] )
{
  if( argc < 5 )
  {
    std::cerr << "Usage: " << argv[0] << "InputImage OutputTextFile NumberOfScales MinScale MaxScale ScaleStepMethod=0"
      " SeedCoord_x SeedCoord_y SeedCoord_z [SeedCoordIsPhysical=0] [InvertDirection=0] [MaxIterations] [PowerFactor (for scaleStepMethod = 2)]" << std::endl;
    return EXIT_FAILURE;
  }

  typedef short           PixelType;
  typedef float           RealPixelType;
  typedef unsigned char   MaskPixelType;
  const unsigned int Dimension = 3;

  typedef itk::Image<PixelType,Dimension>       ImageType;
  typedef itk::Image<MaskPixelType,Dimension>   MaskImageType;
  typedef itk::Image<RealPixelType,Dimension>   RealImageType;

  typedef itk::Point<double,Dimension>          PointType;
  typedef ivan::CircularVesselSection
    <Dimension>                                 VesselSectionType;
  typedef ivan::VesselCenterline
    <unsigned int, VesselSectionType>           CenterlineType;

  typedef ivan::VesselGraph<CenterlineType>               VesselGraphType;
  typedef ivan::VesselBranchNode<CenterlineType>          BranchNodeType;
    
  typedef ivan::VesselTrackerFilter
    <ImageType, VesselGraphType>                          VesselTrackerType;
    
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
  
  
  VesselTrackerType::Pointer vesselTracker = VesselTrackerType::New();
  vesselTracker->SetInput( reader->GetOutput() );

  
  // Set the section estimator
  
  typedef ivan::DiscreteGradientGaussianImageFunction<ImageType>   VectorFieldType;
  typedef ivan::GeometricMeanTwoNegativeEigenvalueFunctor<>        EigenvalueFunctorType;
  typedef ivan::OptimallyOrientedFluxVesselnessImageFunction
    <ImageType,VectorFieldType,EigenvalueFunctorType,double>       MetricFunctionType;
  typedef ivan::OptimallyOrientedFluxVesselnessImageFunctionInitializer
    <MetricFunctionType,ImageType>                                 MetricFunctionInitializerType;
    
  typedef ivan::MultiscaleAdaptiveOOFBasedVesselSectionEstimator
    <ImageType,MetricFunctionType,VectorFieldType,CenterlineType>  SectionEstimatorType;

  SectionEstimatorType::Pointer sectionEstimator = SectionEstimatorType::New();
  sectionEstimator->SetImage( reader->GetOutput() );
  
  /** The metric we used is initialized using a subclass of MetricFunctionInitializerBase. 
    * This way we don't need to access the container with the metric functions for each scale 
    * and initialize them one by one. */
  MetricFunctionInitializerType::Pointer initializer = MetricFunctionInitializerType::New();
  sectionEstimator->SetMetricFunctionInitializer( initializer );
    int scaleStepMethod = atoi( argv[6] );

  if( scaleStepMethod == 0 )
    sectionEstimator->SetScaleStepMethod( SectionEstimatorType::EquispacedScaleSteps );
  else if( scaleStepMethod == 1 )
    sectionEstimator->SetScaleStepMethod( SectionEstimatorType::LogarithmicScaleSteps );
  else if( scaleStepMethod == 2 )
  {
    sectionEstimator->SetScaleStepMethod( SectionEstimatorType::PowerFactorScaleSteps );
    if( argc > 13 )
      sectionEstimator->SetPowerFactor( atof( argv[13] ) );
  }
  else
  {
    std::cerr << "Incorrect scale step method" << std::endl;
    return EXIT_FAILURE;
  }
  
  sectionEstimator->SetNumberOfScales( atof( argv[3] ) );
  sectionEstimator->SetMinimumScale( atof( argv[4] ) );
  sectionEstimator->SetMaximumScale( atof( argv[5] ) );
  sectionEstimator->Initialize();
  
  vesselTracker->SetSectionEstimator( sectionEstimator );

  
  // Set the end condition
  
  EndConditionType::Pointer endCondition = EndConditionType::New();

  bool invertDirection = false;

  if( argc > 11 )
    invertDirection = (bool)atoi( argv[11] );

  vesselTracker->SetInvertDirection( invertDirection );

  unsigned int maxIterations = 40;
 
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

  vesselTracker->SetStartingPoint( point );

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

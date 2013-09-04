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
// File: ivanVesselnessBasedVesselTrackerFilterTest.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: test the OptimizedVesselnessBasedSearchVesselTrackerFilter class. This tracks a 
//   vessel along its centerline from a starting point and optimizes the section center and normal
//   after each step of the tracking algorithm.
// Date: 2010/06/06

#include "ivanVesselGraph.h"
#include "ivanCircularVesselSection.h"
#include "ivanOptimizedVesselnessBasedSearchVesselTrackerFilter.h"
#include "ivanVesselnessBasedVesselTrackerEndCondition.h"
#include "ivanFixedScaleHessianBasedVesselSectionEstimator.h"
#include "ivanMultiscaleMedialnessImageFilter.h"

#include "itkMultiScaleHessianBasedMeasureImageFilter.h"
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkNormalVariateGenerator.h"
#include "itkOnePlusOneEvolutionaryOptimizer.h"
#include "itkHessianToObjectnessMeasureImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkImageRegionIterator.h"
#include "itkLinearInterpolateImageFunction.h"

#include <fstream>


int main( int argc, const char *argv[] )
{
  if( argc < 5 )
  {
    std::cerr << "Usage: " << argv[0] << "inputImage seedCoord_x seedCoord_y seedCoord_z "
      "[inputVesselNessImage]" << std::endl;
    return EXIT_FAILURE;
  }

  typedef short           PixelType;
  typedef float           RealPixelType;
  typedef unsigned char   MaskPixelType;
  const unsigned int Dimension = 3;
  typedef itk::Image<PixelType,Dimension>       ImageType;
  typedef itk::Image<MaskPixelType,Dimension>   MaskImageType;
  typedef itk::Image<RealPixelType,Dimension>   RealImageType;
  typedef RealImageType                         VesselnessImageType;

  typedef itk::SymmetricSecondRankTensor< RealPixelType, Dimension > HessianPixelType;
  typedef itk::Image< HessianPixelType, Dimension >                  HessianImageType;
  
  typedef itk::Point<double,Dimension>          PointType;
  typedef ivan::CircularVesselSection
    <Dimension>                                 VesselSectionType;
  typedef ivan::VesselCenterline
    <unsigned int, VesselSectionType>           CenterlineType;

  typedef ivan::VesselGraph<CenterlineType>            VesselGraphType;
  typedef ivan::VesselBranchNode<CenterlineType>       BranchNodeType;

  // Interpolator used to get values from vesselness image
  typedef itk::LinearInterpolateImageFunction<RealImageType>  VesselnessInterpolatorType;

  // Cost function type
  typedef ivan::Image3DPlaneFunctionToCostFunctionAdaptor<VesselnessImageType>  VesselnessCostFunctionType;
    
  typedef ivan::OptimizedVesselnessBasedSearchVesselTrackerFilter
    <ImageType, VesselGraphType, VesselnessInterpolatorType, 
    VesselnessCostFunctionType>                               VesselTrackerType;
    
  typedef ivan::VesselnessBasedVesselTrackerEndCondition
    <VesselnessInterpolatorType>                              EndConditionType;

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

  // Calculate a vesselness value for the whole input image
  // (ideally this should be computed on-the-fly

  /*typedef itk::MultiscaleMedialnessImageFilter
    <ImageType,RealImageType,double,MaskImageType> MedialnessFilterType;

  MedialnessFilterType::Pointer medialnessFilter = MedialnessFilterType::New();
  medialnessFilter->SetInput( reader->GetOutput() );
  medialnessFilter->SetMaskImage( mask );
  medialnessFilter->SetScales( 3, 2.0, 5.0 );
  medialnessFilter->SetDoNotComputeZeroPixels( true );
  medialnessFilter->SetNormalizeAcrossScale( true );
  medialnessFilter->SetSymmetryCoefficient( 0.5 );
  
  try
  {
    medialnessFilter->Update();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "EXCEPTION CAUGHT!!! " << excpt.GetDescription();
    return EXIT_FAILURE;
  }*/

  RealImageType::Pointer vesselnessImage;

  // If a vesselness image is provided do not calculate one
  typedef itk::ImageFileReader<RealImageType>    RealReaderType;

  // Declare the type of enhancement filter
  typedef itk::HessianToObjectnessMeasureImageFilter< HessianImageType, RealImageType > 
    ObjectnessFilterType;

  // Declare the type of multiscale enhancement filter
  typedef itk::MultiScaleHessianBasedMeasureImageFilter< ImageType, HessianImageType, 
    RealImageType > MultiScaleEnhancementFilterType;

  if( argc > 5 )
  {
    RealReaderType::Pointer vesselnessReader = RealReaderType::New();
    vesselnessReader->SetFileName( argv[5] );

    try
    {
      vesselnessReader->Update();
    }
    catch( itk::ExceptionObject & excpt )
    {
      std::cerr << "EXCEPTION CAUGHT!!! " << excpt.GetDescription();
      return EXIT_FAILURE;
    }
    
    vesselnessImage = vesselnessReader->GetOutput();

  }
  else
  {   
    ObjectnessFilterType::Pointer objectnessFilter = ObjectnessFilterType::New();
    objectnessFilter->SetScaleObjectnessMeasure(false);
    objectnessFilter->SetBrightObject(true);
    objectnessFilter->SetAlpha(0.5);
    objectnessFilter->SetBeta(0.5);
    objectnessFilter->SetGamma(5.0);

    MultiScaleEnhancementFilterType::Pointer multiScaleEnhancementFilter = MultiScaleEnhancementFilterType::New();
    multiScaleEnhancementFilter->SetInput( reader->GetOutput() );
    multiScaleEnhancementFilter->SetHessianToMeasureFilter( objectnessFilter );
    multiScaleEnhancementFilter->SetSigmaStepMethodToLogarithmic();
    multiScaleEnhancementFilter->SetNumberOfSigmaSteps(3);
    multiScaleEnhancementFilter->SetSigmaMinimum( 1.0 );
    multiScaleEnhancementFilter->SetSigmaMaximum( 5.0 );
     
    // Now run the multiscale filter
    try
    {
      multiScaleEnhancementFilter->Update();
    }
    catch (itk::ExceptionObject &e)
    {
      std::cerr << e << std::endl;
    }

    typedef itk::RescaleIntensityImageFilter<RealImageType,RealImageType>  RescalerType;
    RescalerType::Pointer rescaler = RescalerType::New();
    rescaler->SetInput( multiScaleEnhancementFilter->GetOutput() );
    rescaler->SetOutputMinimum(0.0);
    rescaler->SetOutputMaximum(255.0);

    try
    {
      rescaler->Update();
    }
    catch (itk::ExceptionObject &e)
    {
      std::cerr << e << std::endl;
    }

    vesselnessImage = rescaler->GetOutput();
    vesselnessImage->DisconnectPipeline(); // we will free memory here when smartptr go out of scope
  
    typedef itk::ImageFileWriter<RealImageType>  VesselnessWriterType;
    VesselnessWriterType::Pointer writer = VesselnessWriterType::New();
    writer->SetInput( vesselnessImage );
    writer->SetFileName( "vesselness.mhd" );

    try
    {
      writer->Update();
    }
    catch( itk::ExceptionObject & excpt )
    {
      std::cerr << "EXCEPTION CAUGHT!!! " << excpt.GetDescription();
      return EXIT_FAILURE;
    }
  }
  
  VesselTrackerType::Pointer vesselTracker = VesselTrackerType::New();
  vesselTracker->SetInput( reader->GetOutput() );

  // Set the section estimator

  typedef ivan::FixedScaleHessianBasedVesselSectionEstimator
    <ImageType,CenterlineType>  SectionEstimatorType;

  SectionEstimatorType::Pointer sectionEstimator = SectionEstimatorType::New();
  sectionEstimator->SetImage( reader->GetOutput() );
  sectionEstimator->SetScale( 3.0 );
  vesselTracker->SetSectionEstimator( sectionEstimator );

  // Set the vesselness function to be optimized. We want to set as vesselness function
  // an image with values. In order to provide a continuous image function we need to
  // use an interpolator.

  VesselnessInterpolatorType::Pointer vesselnessInterpolator = 
    VesselnessInterpolatorType::New();
  vesselnessInterpolator->SetInputImage( vesselnessImage );
  vesselTracker->SetVesselnessImageFunction( vesselnessInterpolator );

  // Set the end condition
  
  EndConditionType::Pointer endCondition = EndConditionType::New();
  endCondition->SetVesselnessFunction( vesselnessInterpolator );
  endCondition->GetValueFunctor()->SetMinValue( 100.0 );
  
  vesselTracker->SetEndCondition( endCondition.GetPointer() );

  // Create optimizer
  
  typedef itk::Statistics::NormalVariateGenerator  GeneratorType;
  GeneratorType::Pointer generator = GeneratorType::New();

  typedef itk::OnePlusOneEvolutionaryOptimizer    OptimizerType;
  OptimizerType::Pointer optimizer = OptimizerType::New();
  optimizer->MaximizeOn();
  optimizer->SetNormalVariateGenerator( generator );
  optimizer->SetEpsilon( 1.0 );
  optimizer->SetMaximumIteration( 100 );

  vesselTracker->SetOptimizer( optimizer );

  // Set the initial position

  ImageType::IndexType index;
  index[0] = atoi( argv[2] );
  index[1] = atoi( argv[3] );
  index[2] = atoi( argv[4] );

  ImageType::Pointer image = reader->GetOutput();

  VesselTrackerType::ImagePointType startingPoint;
  startingPoint[0] = image->GetOrigin()[0] + image->GetSpacing()[0] * index[0];
  startingPoint[1] = image->GetOrigin()[1] + ( image->GetLargestPossibleRegion().GetSize()[1]
    - 1 - index[1] ) * image->GetSpacing()[1];
  startingPoint[2] = image->GetOrigin()[2] + ( image->GetLargestPossibleRegion().GetSize()[2]
    - 1 - index[2] ) * image->GetSpacing()[2];

  vesselTracker->SetStartingPoint( startingPoint );

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
  fileout.open( "VesselGraph.txt" );

  vesselGraph->Print( fileout );

  fileout.close();

  fileout.open( "VesselBranch.txt" );
  fileout << "Index Center(x y z) Normal (x y z)" << std::endl;

  CenterlineType::Pointer centerline = vesselBranch->GetCenterline();
  
  for( unsigned int i=0; i<centerline->Size(); ++i )
  {
    fileout << i << " " << centerline->at(i)->GetCenter() << " " <<
      centerline->at(i)->GetNormal() << std::endl;
  }

  return EXIT_SUCCESS; 
}

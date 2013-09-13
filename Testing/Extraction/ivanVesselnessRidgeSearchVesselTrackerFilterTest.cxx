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
// File: ivanVesselnessRidgeSearchVesselTrackerFilterTest.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: test the VesselnessRidgeSearchVesselTrackerFilter class. This tracks a 
//   vessel along its centerline from a starting point. At each step, the initial new 
//   section center is reestimated, by taking the local maxima of vesselness in 
//   the first estimated section plane, and then reestimating the section itself on the
//   new center.
//   This version uses a vesselness image if provided, and otherwise uses 
//   itk::MultiScaleHessianBasedMeasureImageFilter.
// Date: 2012/02/12

#include "ivanVesselGraph.h"
#include "ivanCircularVesselSection.h"
#include "ivanVesselnessRidgeSearchVesselTrackerFilter.h"
#include "ivanMaxIterationsVesselTrackerEndCondition.h"
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
  if( argc < 6 )
  {
    std::cerr << "Usage: " << argv[0] << "inputImage outputTextFile scale seedCoord_x seedCoord_y seedCoord_z "
      "[seedCoordIsPhysical=0] [invertDirection=0] [maxIterations=100] [inputVesselnessImage]" << std::endl;
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

  typedef ivan::VesselnessRidgeSearchVesselTrackerFilter
    <ImageType, VesselGraphType, VesselnessInterpolatorType>  VesselTrackerType;
    
  typedef ivan::MaxIterationsVesselTrackerEndCondition        EndConditionType;
  
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

  double scale = atof( argv[3] );

  RealImageType::Pointer vesselnessImage;

  // If a vesselness image is provided do not calculate one
  typedef itk::ImageFileReader<RealImageType>    RealReaderType;

  // Declare the type of enhancement filter
  typedef itk::HessianToObjectnessMeasureImageFilter< HessianImageType, RealImageType > 
    ObjectnessFilterType;
  
  // Declare the type of multiscale enhancement filter
  typedef itk::MultiScaleHessianBasedMeasureImageFilter< ImageType, HessianImageType, 
    RealImageType > MultiScaleEnhancementFilterType;

  if( argc > 10 )
  {
    RealReaderType::Pointer vesselnessReader = RealReaderType::New();
    vesselnessReader->SetFileName( argv[10] );

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
    multiScaleEnhancementFilter->SetNumberOfSigmaSteps(1);
    multiScaleEnhancementFilter->SetSigmaMinimum( scale );
    multiScaleEnhancementFilter->SetSigmaMaximum( scale );
     
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
  sectionEstimator->SetScale( scale );
  vesselTracker->SetSectionEstimator( sectionEstimator );

  // Set the vesselness function to be optimized. We want to set as vesselness function
  // an image with values. In order to provide a continuous image function we need to
  // use an interpolator.

  VesselnessInterpolatorType::Pointer vesselnessInterpolator = 
    VesselnessInterpolatorType::New();
  vesselnessInterpolator->SetInputImage( vesselnessImage );
  vesselTracker->SetVesselnessImageFunction( vesselnessInterpolator );

  
  // Invert direction if necessary
  
  bool invertDirection = false;

  if( argc > 8 )
    invertDirection = (bool)atoi( argv[8] );

  vesselTracker->SetInvertDirection( invertDirection );


  // Set the end condition
  
  EndConditionType::Pointer endCondition = EndConditionType::New();
    
  unsigned int maxIterations = 100;
  
  if( argc > 9 )
    maxIterations = atoi( argv[9] );
    
  endCondition->SetMaxIterations( maxIterations );  
  vesselTracker->SetEndCondition( endCondition.GetPointer() );

  
 
  // Set the initial position
  
  bool seedCoordIsPhysical = false;
    
  if( argc > 7 )
    seedCoordIsPhysical = (bool)atoi( argv[7] );

  ImageType::IndexType index;
  ImageType::PointType point;
  VesselTrackerType::InputImagePointType startingPoint;

  ImageType::Pointer image = reader->GetOutput();
  
  if( !seedCoordIsPhysical )
  {
    index[0] = atoi( argv[4] );
    index[1] = atoi( argv[5] );
    index[2] = atoi( argv[6] );

    image->TransformIndexToPhysicalPoint( index, point );
  }
  else
  {
    point[0] = atof( argv[4] );
    point[1] = atof( argv[5] );
    point[2] = atof( argv[6] );

    image->TransformPhysicalPointToIndex( point, index );
  }

  vesselTracker->SetStartingPoint( point );
  
  
  // Set the search parameters
  
  vesselTracker->SetRadialResolution(3);
  vesselTracker->SetAngularResolution(8);
  vesselTracker->SetMaximumSearchDistance( 1.5 * scale );
  
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

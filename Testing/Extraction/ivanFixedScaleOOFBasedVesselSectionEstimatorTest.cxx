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
// File: ivanFixedScaleHessianBasedVesselSectionEstimatorTest.cxx
// Author: Ivan Macia (imacia@vicomtech.org)
// Description: tests the FixedScaleOOFBasedEstimator class with a tracker filter.
// Date: 2010/01/21


#include "ivanVesselGraph.h"
#include "ivanCircularVesselSection.h"
#include "ivanVesselTrackerFilter.h"
#include "ivanMaxIterationsVesselTrackerEndCondition.h"
#include "ivanFixedScaleOOFBasedVesselSectionEstimator.h"
#include "ivanDiscreteGradientGaussianImageFunction.h"

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIterator.h"
#include "itkLinearInterpolateImageFunction.h"

#include <fstream>


int main( int argc, const char *argv[] )
{
  if( argc < 5 )
  {
    std::cerr << "Usage: " << argv[0] << "InputImage OutputTextFile Scale SeedCoord_x SeedCoord_y SeedCoord_z [SeedCoordIsPhysical=0] [InvertDirection=0] [MaxIterations]" << std::endl;
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
    
  typedef ivan::VesselTrackerFilter
    <ImageType, VesselGraphType>                          VesselTrackerType;
    
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
  vesselTracker->SetInitialStepSize( 1.0 );

  
  // Set the section estimator

  typedef ivan::DiscreteGradientGaussianImageFunction<ImageType>   VectorFieldType;
  typedef ivan::FixedScaleOOFBasedVesselSectionEstimator
    <ImageType,VectorFieldType,CenterlineType>                     SectionEstimatorType;

  SectionEstimatorType::Pointer sectionEstimator = SectionEstimatorType::New();
  sectionEstimator->SetImage( reader->GetOutput() );
  double scale = atof( argv[3] );
  sectionEstimator->SetRadius( scale );
  sectionEstimator->SetGradientSigma( 1.0 );
  sectionEstimator->Initialize();
  vesselTracker->SetSectionEstimator( sectionEstimator );

  
  // Set the end condition
  
  typedef ivan::MaxIterationsVesselTrackerEndCondition  EndConditionType;
  EndConditionType::Pointer endCondition = EndConditionType::New();

  // Invert direction if necessary

  bool invertDirection = false;

  if( argc > 8 )
    invertDirection = (bool)atoi( argv[8] );

  vesselTracker->SetInvertDirection( invertDirection );


  unsigned int maxIterations = 20;
 
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

  std::cout << "Seed Image Value: " << (double)image->GetPixel( index ) << std::endl;

  /*startingPoint[0] = image->GetOrigin()[0] + image->GetSpacing()[0] * index[0];
  startingPoint[1] = image->GetOrigin()[1] + image->GetSpacing()[1] * index[1];
  startingPoint[2] = image->GetOrigin()[2] + image->GetSpacing()[2] * index[2];*/

  startingPoint[0] = point[0];
  startingPoint[1] = point[1];
  startingPoint[2] = point[2];
  
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
  fileout.open( argv[2] );
  fileout << "Index Center(x y z) Normal (x y z) Scale" << std::endl;

  CenterlineType::Pointer centerline = vesselBranch->GetCenterline();
  CenterlineType::SectionType *currentSection;

  double accumError = 0.0, currentError = 0.0, currentXYZError = 0.0;
  double maxError = itk::NumericTraits<double>::min();
  double mean = 0.0, stddev = 0.0;
  double targetZPos = 0.0;
  double meanAngleError = 0.0;
  
  std::vector<double> xy_errors ( centerline->Size() );
  std::vector<double> xyz_errors( centerline->Size() );
  std::vector<double> angle_devs( centerline->Size() );
 
  for( unsigned int i=0; i<centerline->Size(); ++i )
  {
    currentSection  = centerline->at(i);
    
    fileout << currentSection->GetCenter()[0] << " "
            << currentSection->GetCenter()[1] << " "
            << currentSection->GetCenter()[2] << " "
            << currentSection->GetNormal()[0] << " "
            << currentSection->GetNormal()[1] << " "
            << currentSection->GetNormal()[2] << " "
            << scale 
            << std::endl;

    // Compute the error as the distance to the centerline. Since this is oriented in
    // the z-axis, we just need to take into account x and y coordinates
    currentError = ( startingPoint[0] - currentSection->GetCenter()[0] ) *
                   ( startingPoint[0] - currentSection->GetCenter()[0] ) +
                   ( startingPoint[1] - currentSection->GetCenter()[1] ) *
                   ( startingPoint[1] - currentSection->GetCenter()[1] );

    // Assume constant step size in Z of 1.0

    targetZPos = startingPoint[2] + (double)i;

    currentXYZError = ( startingPoint[0] - currentSection->GetCenter()[0] ) *
                      ( startingPoint[0] - currentSection->GetCenter()[0] ) +
                      ( startingPoint[1] - currentSection->GetCenter()[1] ) *
                      ( startingPoint[1] - currentSection->GetCenter()[1] ) +
                      ( targetZPos - currentSection->GetCenter()[2] ) *
                      ( targetZPos - currentSection->GetCenter()[2] );
    
    currentError    = sqrt( currentError );
    currentXYZError = sqrt( currentXYZError );

    xy_errors[i]  = currentError;
    xyz_errors[i] = currentXYZError;
    
    accumError += currentError;
    
    if( currentXYZError > maxError )
      maxError = currentXYZError;

    // The angle deviation is measured as 1 - cos(theta) where theta is the angle
    // between both vectors. The centerline vector is 0,0,1 and thus the cos of the
    // angle is -> a dot b = 0*b[0] + 0*b[1] + 1*b[2] = b[2] (assuming unit vectors)
    // If the angle is more than 90� the cos is negative and adds to one. For the
    // best value it gives zero.
    angle_devs[i] = 1.0 - currentSection->GetNormal()[2];
  }

  double samples = (double)centerline->Size();

  mean = accumError / samples;
  
  for( unsigned int i=0; i<xy_errors.size(); ++i )
    stddev += ( xy_errors[i] - mean ) * ( xy_errors[i] - mean );    
  
  stddev /= ( xy_errors.size() );
  stddev  = sqrt( stddev );

  unsigned int oneThirdPos, twoThirdsPos;
  double oneThirdPosError = 0.0, twoThirdsPosError = 0.0, endError = 0.0;

  oneThirdPos  = xyz_errors.size() / 3;
  twoThirdsPos = 2 * oneThirdPos;

  oneThirdPosError  = xyz_errors[oneThirdPos];
  twoThirdsPosError = xyz_errors[twoThirdsPos];
  endError = xyz_errors[xyz_errors.size()-1];

  meanAngleError = std::accumulate( angle_devs.begin(), angle_devs.end(), 0.0 ) / samples;

  double oneThirdAngleError = 0.0, twoThirdsAngleError = 0.0, endAngleError = 0.0;

  oneThirdAngleError  = angle_devs[oneThirdPos];
  twoThirdsAngleError = angle_devs[twoThirdsPos];
  endAngleError = angle_devs[angle_devs.size()-1];

  std::string   outFileName = argv[2];
  std::string   statsFileName = outFileName.substr( 0, outFileName.find_last_of( '.' ) );
  statsFileName += "_stats.txt";

  std::ofstream statsFileout;
  statsFileout.open( statsFileName.c_str() );

  statsFileout << "MeanPosError StdDevPosError MaxPosError OneThirdPosError TwoThirdsPosError "
    "EndPosError MeanAngleError OneThirdAngleError TwoThirdsAngleError EndAngleError" << std::endl;
  statsFileout << mean << " " << stddev << " " << maxError << " " 
    << oneThirdPosError << " " << twoThirdsPosError << " " << endError << " "
    << meanAngleError << " " << oneThirdAngleError << " " << twoThirdsAngleError << " " 
    << endAngleError << " " << std::endl;
  
  fileout.close();
  statsFileout.close();

  return EXIT_SUCCESS; 
}

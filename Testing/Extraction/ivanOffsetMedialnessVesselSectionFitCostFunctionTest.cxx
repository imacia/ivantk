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
// File: CircularGaussianTube.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a Gaussian tube and sets an incorrect normal in the center of the central section.
// The tests uses the OffsetMedialnessVesselSectionFitCostFunction to estimate the correct normal which
// should correspond to the z axis.
// Date: 2010/09/17


#include "ivanOffsetMedialnessVesselSectionFitCostFunction.h"
#include "ivanOptimizedVesselSectionEstimator.h"
#include "ivanCircularGaussianStraightTubeGenerator.h"
#include "ivanVesselCenterline.h"
#include "ivanCircularVesselSection.h"

#include "itkOnePlusOneEvolutionaryOptimizer.h"
#include "itkNormalVariateGenerator.h"

#define MAX_SEARCH_RADIUS 10.0

int main( int argc, char *argv[] )
{
  // Verify the number of parameters in the command line
  if( argc < 3 )
  {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << " TubeSigma TubeHeight" 
    << std::endl;
    return EXIT_FAILURE;
  }

  typedef short    PixelType;
  
  typedef ivan::CircularGaussianStraightTubeGenerator<PixelType>   TubeGeneratorType;
  typedef TubeGeneratorType::ImageType                             TubeImageType;
    
  TubeGeneratorType tubeGenerator;
  tubeGenerator.SetSigma( atof( argv[1] ) );
  tubeGenerator.SetHeight( atof( argv[2] ) );
  tubeGenerator.SetMaxValue( 255.0 );
  tubeGenerator.SetImageSpacing( 1.0 );

  unsigned long sectionImageSize;
  
  sectionImageSize = tubeGenerator.GetSigma() * 10.0 / tubeGenerator.GetImageSpacing();
  if( sectionImageSize % 2 == 0 )
    sectionImageSize += 1;

  tubeGenerator.SetSectionImageSize( sectionImageSize );
   
  TubeImageType::Pointer tubeImage;
  
  try
  {
    tubeImage = tubeGenerator.Create();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "ITK exception caught!!!" << std::endl;
    std::cerr << excpt.GetDescription() << std::endl;
    return EXIT_FAILURE; 
  }
  
  // Define centerline and section types
  
  typedef ivan::CircularVesselSection<>            VesselSectionType;
  typedef ivan::VesselCenterline
    <unsigned int, VesselSectionType>              CenterlineType;
  typedef VesselSectionType::CenterPointType       CenterPointType;

  // Now create a centerline with a single section.
  // Provide a wrong normal for the section (the tube is oriented towards the z-axis).
  // The algorithm will try to estimate the real normal

  CenterlineType::Pointer     centerline = CenterlineType::New();
  VesselSectionType::Pointer  section = VesselSectionType::New();

  VesselSectionType::VectorType initialNormal;
  initialNormal[0] = 1.0;
  initialNormal[1] = 0.0;
  initialNormal[2] = 0.0;

  section->SetNormal( initialNormal );

  // Calculate the center point, where we are going to estimate the section normal
  
  TubeImageType::RegionType::SizeType size = tubeImage->GetLargestPossibleRegion().GetSize();
  
  TubeImageType::IndexType index;
  index[0] = size[0] / 2;
  index[1] = size[1] / 2;
  index[2] = size[2] / 2;
  
  CenterPointType center;
  center[0] = index[0] * tubeImage->GetSpacing()[0];
  center[1] = index[1] * tubeImage->GetSpacing()[1];
  center[2] = index[2] * tubeImage->GetSpacing()[2];

  section->SetCenter( center );
  section->SetRadius( tubeGenerator.GetSigma() );
  
  centerline->push_back( section );

  // Declare cost function and optimizer
  
  typedef ivan::OffsetMedialnessVesselSectionFitCostFunction<TubeImageType>  CostFunctionType;
  typedef CostFunctionType::MedialnessFunctionType                           MedialnessFunctionType;
  typedef CostFunctionType::MedialnessFunctionPointer                        MedialnessFunctionPointer;
  typedef itk::Statistics::NormalVariateGenerator                            GeneratorType;
  typedef itk::OnePlusOneEvolutionaryOptimizer                               OptimizerType;
  
  typedef ivan::OptimizedVesselSectionEstimator
    <CostFunctionType, OptimizerType, CenterlineType>                        SectionEstimatorType;
   
  
  GeneratorType::Pointer generator = GeneratorType::New();
  generator->Initialize(12345);
	      
  OptimizerType::Pointer optimizer = OptimizerType::New();
  optimizer->MaximizeOn();
  optimizer->SetNormalVariateGenerator( generator );
  optimizer->SetEpsilon( 1.0 );
  optimizer->SetMaximumIteration( 1000 );
  optimizer->Initialize( MAX_SEARCH_RADIUS );
  
  CostFunctionType::Pointer costFunction = CostFunctionType::New();
  costFunction->SetImage( tubeImage );

  // Access to the medialness function used by the cost function to set its properties
  costFunction->GetMedialnessFunction()->SetSymmetryCoefficient( 0.5 );
    
  //medialnessFunction->AutoComputeSectionNormalOff();     // this flag is set to off inside the cost function but 
                                                         // illustrates a good usage of the medialness function
  //medialnessFunction->SetSectionNormal( initialNormal );
   
  // WARNING: DO NOT SET THE SECTION CENTER OF THE COST FUNCTION MANUALLY
  // This is done by the section estimator algorithm, and is taken from the centerlines.
  //costFunction->SetSectionCenter( center );

  costFunction->SetMaxRadius( MAX_SEARCH_RADIUS );
  costFunction->Initialize();
  
  SectionEstimatorType::Pointer sectionEstimator = SectionEstimatorType::New();
  sectionEstimator->SetCostFunction( costFunction );
  sectionEstimator->SetOptimizer( optimizer );
  sectionEstimator->SetCenterline( centerline );

  try
  {
    sectionEstimator->Compute();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "ITK exception caught!!!" << std::endl;
    std::cerr << excpt.GetDescription() << std::endl;
    return EXIT_FAILURE;
  }
  
  return EXIT_SUCCESS;
}

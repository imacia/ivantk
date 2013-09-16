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
// File: ivanOptimallyOrientedFluxVesselnessImageFunctionTest.cxx
// Author: Ivan Macia (imacia@vicomtech.org)
// Description: tests optimally oriented flux measure (Law & Chung 2008) in the form of an image function.
// Date: 2010/01/18


#include "ivanOptimallyOrientedFluxVesselnessImageFunction.h"
#include "ivanDiscreteGradientGaussianImageFunction.h"
#include "ivanDetectionTestingHelper.h"

#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkRescaleIntensityImageFilter.h"

#include <fstream>


int main( int argc, const char *argv[] )
{
  if( argc < 3 )
  {
    std::cerr << "Usage: " << argv[0] << "InputImage TestMode(0-1) OutputFileName Radius [Rescale=1]" << std::endl;
    return EXIT_FAILURE;
  }

  typedef float       PixelType;
  const unsigned int Dimension = 3;
  typedef itk::Image<PixelType,Dimension>       ImageType;
  
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
    
  // Create the image function
  
  typedef ivan::DiscreteGradientGaussianImageFunction<ImageType>     GradientFunctionType;
  typedef ivan::GeometricMeanTwoNegativeEigenvalueFunctor<>          EigenvalueFunctorType;
  typedef ivan::OptimallyOrientedFluxVesselnessImageFunction
    <ImageType,GradientFunctionType,EigenvalueFunctorType,double>    VesselnessFunctionType;
  
  VesselnessFunctionType::Pointer vesselness = VesselnessFunctionType::New();
  
  GradientFunctionType::Pointer gradientFunction = vesselness->GetVectorField();
  
  gradientFunction->SetInputImage( reader->GetOutput() );
  gradientFunction->SetSigma( 1.0 );
  gradientFunction->NormalizeAcrossScaleOn();
  gradientFunction->UseImageSpacingOn();
  gradientFunction->Initialize();
  
  double radius = atof( argv[4] );
  vesselness->SetRadius( radius );
  vesselness->SetInputImage( reader->GetOutput() );
  vesselness->Initialize(); // this computes the sphere grid
      
  bool testMode = atoi( argv[2] );
  
  bool rescale = true;
  
  if( argc > 5 )
    rescale = atoi( argv[5] );
    
  // While in test mode only iterate through a row
  
  if( !testMode )
  {
    std::string fileName;
    
    if( argc > 3 )
      fileName = argv[3];
    else
      fileName = "OOFVesselness.mhd";
    
    return DenseComputeVesselness( reader->GetOutput(), vesselness.GetPointer(), fileName.c_str(), rescale );
  }
  else
  {
    std::string fileName;
    
    if( argc > 3 )
      fileName = argv[3];
    else
      fileName = "OOFVesselness.png";
    
    return SparseComputeVesselness( reader->GetOutput(), vesselness.GetPointer(), fileName.c_str(), true );
  }
}

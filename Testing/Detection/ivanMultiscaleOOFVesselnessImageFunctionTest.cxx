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
// File: ivanMultiscaleOOFVesselnessImageFunctionTest.cxx
// Author: Ivan Macia (imacia@vicomtech.org)
// Description: tests MultiscaleImageFunction with Law & Chung's Optimally Oriented Flux (OOF) at multiple scales
// Date: 2012/02/23


#include "ivanMultiscaleImageFunction.h"
#include "ivanDiscreteGradientGaussianImageFunction.h"
#include "ivanOptimallyOrientedFluxVesselnessImageFunction.h"
#include "ivanOptimallyOrientedFluxVesselnessImageFunctionInitializer.h"
#include "ivanDetectionTestingHelper.h"

#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"

#include <fstream>



int main( int argc, const char *argv[] )
{
  if( argc < 6 )
  {
    std::cerr << "Usage: " << argv[0] << "InputImage TestMode(0-1) OutputImage NumScales MinScale MaxScale [Rescale=1]" 
      << std::endl;
    return EXIT_FAILURE;
  }

  typedef float      PixelType;
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

  // Create an output image
  
  ImageType::Pointer output = ImageType::New();
    
  output->SetRegions( reader->GetOutput()->GetLargestPossibleRegion() );
  output->SetSpacing( reader->GetOutput()->GetSpacing() );
  output->SetOrigin( reader->GetOutput()->GetOrigin() );
  output->SetDirection( reader->GetOutput()->GetDirection() );
  output->Allocate();
  output->FillBuffer( itk::NumericTraits<PixelType>::Zero );
    
  
  // Declare the type of base and multiscale vesselness function
  
  typedef ivan::DiscreteGradientGaussianImageFunction<ImageType>          GradientFunctionType;
  typedef ivan::GeometricMeanTwoNegativeEigenvalueFunctor<>               EigenvalueFunctorType;
  typedef ivan::OptimallyOrientedFluxVesselnessImageFunction
    <ImageType,GradientFunctionType,EigenvalueFunctorType,double>         VesselnessFunctionType;
  typedef ivan::OptimallyOrientedFluxVesselnessImageFunctionInitializer
    <VesselnessFunctionType,ImageType>                                    VesselnessFunctionInitializerType;
    
  typedef ivan::MultiscaleImageFunction<VesselnessFunctionType, ImageType, double>   MultiscaleVesselnessFunctionType;
    
    
  MultiscaleVesselnessFunctionType::Pointer multiscaleVesselness = MultiscaleVesselnessFunctionType::New();
    
  multiscaleVesselness->SetInputImage( reader->GetOutput() );
  
  VesselnessFunctionInitializerType::Pointer initializer = VesselnessFunctionInitializerType::New();
  multiscaleVesselness->SetScaledImageFunctionInitializer( initializer );
   
  multiscaleVesselness->SetNumberOfScales( atoi( argv[4] ) );
  multiscaleVesselness->SetMinimumScale( atoi( argv[5] ) );
  multiscaleVesselness->SetMaximumScale( atoi( argv[6] ) );
  multiscaleVesselness->Initialize();

  
  bool testMode = atoi( argv[2] );
  
  bool rescale = true;
  
  if( argc > 7 )
    rescale = atoi( argv[7] );
    
  // While in test mode only iterate through a row
  
  if( !testMode )
  {
    std::string fileName;
    
    if( argc > 3 )
      fileName = argv[3];
    else
      fileName = "MultiscaleOOFVesselness.mhd";
    
    return DenseComputeVesselness( reader->GetOutput(), multiscaleVesselness.GetPointer(), fileName.c_str(), rescale );
  }
  else
  {
    std::string fileName;
    
    if( argc > 3 )
      fileName = argv[3];
    else
      fileName = "MultiscaleOOFVesselness.png";
    
    return SparseComputeVesselness( reader->GetOutput(), multiscaleVesselness.GetPointer(), fileName.c_str(), true );
  }
}

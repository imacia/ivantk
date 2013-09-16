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
// File: ivanMultiscaleOffsetMedialnessImageFunctionTest.cxx
// Author: Ivan Macia (imacia@vicomtech.org)
// Description: tests MultiscaleImageFunction with the OffsetMedialnessImageFunction at multiple scales
// Date: 2012/02/23


#include "ivanMultiscaleImageFunction.h"
#include "ivanOffsetMedialnessImageFunction.h"
#include "ivanOffsetMedialnessImageFunctionInitializer.h"
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
    std::cerr << "Usage: " << argv[0] << "InputImage TestMode(0-1) OutputImage NumScales MinScale MaxScale [ScaleIsRadius] [FixedRadiusOrSigma] [GradientSigma=1.0]"
      "[SymmetryCoefficient(0.0-1.0)=0.5] [Rescale=1]" << std::endl;
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

    
  // Create the base and multiscale image function
  
  typedef ivan::OffsetMedialnessImageFunction<ImageType,double>              MedialnessFunctionType;
  typedef ivan::OffsetMedialnessImageFunctionInitializer<ImageType,double>   MedialnessFunctionInitializerType;
    
  typedef ivan::MultiscaleImageFunction<MedialnessFunctionType, ImageType, double>   MultiscaleMedialnessFunctionType;
    
    
  MultiscaleMedialnessFunctionType::Pointer multiscaleMedialness = MultiscaleMedialnessFunctionType::New();
    
  multiscaleMedialness->SetInputImage( reader->GetOutput() );
  
  MedialnessFunctionInitializerType::Pointer initializer = MedialnessFunctionInitializerType::New();
  initializer->SetGradientImageFunctionType( MedialnessFunctionType::GradientNormalProjectionFunctionHyperIntense );
 
  bool useRadiusAsScale = true;
 
  if( argc > 7 )
  {
    useRadiusAsScale = atoi( argv[7] );
		if( useRadiusAsScale )
			initializer->SetScaleSelectionMethod( MedialnessFunctionInitializerType::UseScaleAsRadius );
		else
			initializer->SetScaleSelectionMethod( MedialnessFunctionInitializerType::UseScaleAsSigma );
  }
  else
    initializer->SetScaleSelectionMethod( MedialnessFunctionInitializerType::UseScaleAsRadius );
 
  if( argc > 8 )
  {
    if( useRadiusAsScale )
      initializer->SetHessianSigma( atof( argv[8] ) );
    else
      initializer->SetRadius( atof( argv[8] ) );
  }
  else
    initializer->SetHessianSigma( 1.0 );
    
  if( argc > 9 )
    initializer->SetGradientSigma( atof( argv[9] ) );
  else
    initializer->SetGradientSigma( 1.0 );
 
  if( argc > 10 )
    initializer->SetSymmetryCoefficient( atof( argv[10] ) );
  else
    initializer->SetSymmetryCoefficient( 0.5 );
 
  multiscaleMedialness->SetScaledImageFunctionInitializer( initializer );
   
  multiscaleMedialness->SetNumberOfScales( atoi( argv[4] ) );
  multiscaleMedialness->SetMinimumScale( atoi( argv[5] ) );
  multiscaleMedialness->SetMaximumScale( atoi( argv[6] ) );
  multiscaleMedialness->Initialize();

  bool testMode = atoi( argv[2] );
  
  bool rescale = true;
  
  if( argc > 11 )
    rescale = atoi( argv[11] );
    
  // While in test mode only iterate through a row
  
  if( !testMode )
  {
    std::string fileName;
    
    if( argc > 3 )
      fileName = argv[3];
    else
      fileName = "MultiscaleOffsetMedialness.mhd";
    
    return DenseComputeVesselness( reader->GetOutput(), multiscaleMedialness.GetPointer(), fileName.c_str(), rescale );
  }
  else
  {
    std::string fileName;
    
    if( argc > 3 )
      fileName = argv[3];
    else
      fileName = "MultiscaleOffsetMedialness.png";
    
    return SparseComputeVesselness( reader->GetOutput(), multiscaleMedialness.GetPointer(), fileName.c_str(), true );
  }
}

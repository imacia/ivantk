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
// File: ivanFrangiVesselnessImageFunctionTest.cxx
// Author: Ivan Macia (imacia@vicomtech.org)
// Description: tests Sato vesselness measure in the form of an image function.
// Date: 2010/08/16

#include "ivanSatoVesselnessImageFunction.h"
#include "ivanDetectionTestingHelper.h"

#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"


#include <fstream>


int main( int argc, const char *argv[] )
{
  if( argc < 5 )
  {
    std::cerr << "Usage: " << argv[0] << "InputImage Testmode(0-1) [OutputImage] [Sigma] [Gamma12] [Gamma23] [Alpha] [Rescale=1]"
      << std::endl;
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
  
  typedef ivan::SatoVesselnessImageFunction<ImageType,double>   VesselnessFunctionType;
  
  VesselnessFunctionType::Pointer vesselness = VesselnessFunctionType::New();
  vesselness->SetInputImage( reader->GetOutput() );

  if( argc > 4 )
    vesselness->SetSigma( atof( argv[4] ) );
  else
    vesselness->SetSigma( 2.0 );
  
  if( argc > 5 )
    vesselness->SetGamma12( atof( argv[5] ) );
  else
    vesselness->SetGamma12( 1.0 );
  
  if( argc > 6 )
    vesselness->SetGamma23( atof( argv[6] ) );
  else
    vesselness->SetGamma23( 1.0 );
  
  if( argc > 7 )
    vesselness->SetAlpha( atof( argv[7] ) );
  else
    vesselness->SetAlpha( 0.25 );
  
  vesselness->Initialize();
    
  bool testMode = atoi( argv[2] );
    
  bool rescale = true;
  
  if( argc > 8 )
    rescale = atoi( argv[8] );
  
  // While in test mode only iterate through a row
  
  if( !testMode )
  {
    std::string fileName;
    
    if( argc > 3 )
      fileName = argv[3];
    else
      fileName = "SatoVesselness.mhd";
    
    return DenseComputeVesselness( reader->GetOutput(), vesselness.GetPointer(), fileName.c_str(), rescale );
  }
  else
  {
    std::string fileName;
    
    if( argc > 3 )
      fileName = argv[3];
    else
      fileName = "SatoVesselness.png";
    
    return SparseComputeVesselness( reader->GetOutput(), vesselness.GetPointer(), fileName.c_str(), true );
  }
}

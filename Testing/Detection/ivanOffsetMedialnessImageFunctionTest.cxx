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
// File: ivanOffsetMedialnessImageFunctionTest.cxx
// Author: Ivan Macia (imacia@vicomtech.org)
// Description: tests modified Krissian offset medialness measure in the form of an image function.
// Date: 2010/07/17


#include "ivanOffsetMedialnessImageFunction.h"
#include "ivanDetectionTestingHelper.h"

#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"

#include <fstream>



int main( int argc, const char *argv[] )
{
  if( argc < 2 )
  {
    std::cerr << "Usage (maria): " << argv[0] << "InputImage Testmode(0-1) [OutputImage] [Radius] [Sigma] [RadialResolution(0=adaptative)]"
      "[SymmetryCoefficient(0.0-1.0)] [Rescale=1]" << std::endl;
    return EXIT_FAILURE;
  }

  typedef float      PixelType;
  const unsigned int Dimension = 3;
  typedef itk::Image<PixelType,Dimension>       ImageType;
  
  typedef itk::ImageFileReader<ImageType>  ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName( argv[1] );

  std::cout << "Reading input image " << argv[1] << "..." << std::endl;

  try
  {
    reader->Update();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "EXCEPTION CAUGHT!!! " << excpt.GetDescription();
    return EXIT_FAILURE;
  }

  std::cout << "Finished reading." << std::endl;

     
  // Create the image function
  
  typedef ivan::OffsetMedialnessImageFunction<ImageType,double>   MedialnessFunctionType;
  
  MedialnessFunctionType::Pointer medialness = MedialnessFunctionType::New();
  medialness->SetInputImage( reader->GetOutput() );

  double sigma = 2.0;

  if( argc > 5 )
    sigma = atof( argv[5] );

  medialness->SetSigma( sigma );

  double radius = vcl_sqrt( 3.0 ) * medialness->GetSigma();
    
  if( argc > 4 )
    radius = atof( argv[4] );

  medialness->SetRadius( radius );

  if( argc > 6 )
  {
    if( atoi( argv[6] ) == 0 )
      medialness->AdaptativeSamplingOn();
    else
      medialness->SetRadialResolution( atoi( argv[6] ) );
  }
  else
    medialness->AdaptativeSamplingOn();    

  if( argc > 7 )
  {
    double symmetryCoeff = atof( argv[7] );
    
    if( symmetryCoeff < 0.0 )
      symmetryCoeff = 0.0;
    else if( symmetryCoeff > 1.0 )
      symmetryCoeff = 1.0;

    medialness->SetSymmetryCoefficient( symmetryCoeff );
  }

  medialness->SetGradientSigma( 1.0 );
  medialness->Initialize();
  
  bool testMode = atoi( argv[2] );
  
  bool rescale = true;
  
  if( argc > 8 )
    rescale = atoi( argv[8] );

  
  /////////////////////////////////////////
  // First try with the gradient projection

  std::cout << "Computing offset medialness..." << std::endl;
  
  medialness->SetGradientImageFunctionType( MedialnessFunctionType::GradientNormalProjectionFunctionHyperIntense );
    
  
  // While in test mode only iterate through a row
  
  if( !testMode )
  {
    std::string fileName;
    
    if( argc > 3 )
    {
      fileName = argv[3];
      
      size_t pos = fileName.find_last_of( '.' );
      
      std::string extension = fileName.substr( pos+1, 3 );
      
      fileName = fileName.substr( 0, pos );
      fileName += "_GradientProj.";
      fileName += extension;
    }
    else
      fileName = "OffsetMedialnessGradientProj.mhd";
    
    if( DenseComputeVesselness( reader->GetOutput(), medialness.GetPointer(), fileName.c_str(), rescale ) == EXIT_FAILURE )
      return EXIT_FAILURE;
  }
  else
  {
    std::string fileName;
    
    if( argc > 3 )
    {
      fileName = argv[3];
      
      size_t pos = fileName.find_last_of( '.' );
      
      std::string extension = fileName.substr( pos+1, 3 );
      
      fileName = fileName.substr( 0, pos );
      fileName += "_GradientProj.";
      fileName += extension;
    }
    else
      fileName = "OffsetMedialnessGradientProj.png";
    
    if( SparseComputeVesselness( reader->GetOutput(), medialness.GetPointer(), fileName.c_str(), true )  == EXIT_FAILURE )
      return EXIT_FAILURE;
  }  
  
  
  //////////////////////////////////
  // Now with the gradient magnitude
  
  medialness->SetGradientImageFunctionType( MedialnessFunctionType::GradientMagnitudeFunction );
  
  if( !testMode )
  {
    std::string fileName;
    
    if( argc > 3 )
    {
      fileName = argv[3];
      
      size_t pos = fileName.find_last_of( '.' );
      
      std::string extension = fileName.substr( pos+1, 3 );
      
      fileName = fileName.substr( 0, pos );
      fileName += "_GradientMag.";
      fileName += extension;
    }
    else
      fileName = "OffsetMedialnessGradientMag.mhd";
    
    if( DenseComputeVesselness( reader->GetOutput(), medialness.GetPointer(), fileName.c_str(), rescale ) == EXIT_FAILURE )
      return EXIT_FAILURE;
  }
  else
  {
    std::string fileName;
    
    if( argc > 3 )
    {
      fileName = argv[3];
      
      size_t pos = fileName.find_last_of( '.' );
      
      std::string extension = fileName.substr( pos+1, 3 );
      
      fileName = fileName.substr( 0, pos );
      fileName += "_GradientMag.";
      fileName += extension;
    }
    else
      fileName = "OffsetMedialnessGradientMag.png";
    
    if( SparseComputeVesselness( reader->GetOutput(), medialness.GetPointer(), fileName.c_str(), true )  == EXIT_FAILURE )
      return EXIT_FAILURE;
  }   
}

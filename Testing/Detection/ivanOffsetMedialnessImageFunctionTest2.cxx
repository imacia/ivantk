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
// File: ivanOffsetMedialnessImageFunctionTest2.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description:
// Date: 2012/07/04

#include "ivanOffsetMedialnessImageFunction.h"
#include "ivanOffsetMedialnessImageFunctionInitializer.h"
#include "ivanImageFunctionBasedImageFilter.h"

#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"

#include <fstream>



int main( int argc, const char *argv[] )
{
  if( argc < 2 )
  {
    std::cerr << "Usage (maria): " << argv[0] << "InputImage [OutputImage] [Radius] [HessianSigma] [SymmetryCoefficient(0.0-1.0)] [Threshold]" << std::endl;
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
  
  
  typedef ivan::OffsetMedialnessImageFunction<ImageType,double>      MedialnessFunctionType;
  typedef ivan::ImageFunctionBasedImageFilter<ImageType,ImageType,
    MedialnessFunctionType>                                          MedialnessFilterType;
    
  MedialnessFilterType::Pointer medialnessFilter = MedialnessFilterType::New();
  medialnessFilter->SetInput( reader->GetOutput() ); 
  

  typedef ivan::OffsetMedialnessImageFunctionInitializer<ImageType>  MedialnessFunctionInitializerType;
    
  MedialnessFunctionInitializerType::Pointer initializer = MedialnessFunctionInitializerType::New();
  initializer->SetScaleSelectionMethod( MedialnessFunctionInitializerType::IgnoreScale );
  initializer->SetGradientSigma( 1.0 );
  initializer->SetGradientImageFunctionType( MedialnessFunctionType::GradientNormalProjectionFunctionHyperIntense );
  
  double sigma = 2.0;

  if( argc > 4 )
    sigma = atof( argv[4] );
  
  initializer->SetHessianSigma( sigma );
  
  double radius = vcl_sqrt( 3.0 ) * sigma;
    
  if( argc > 3 )
    radius = atof( argv[3] );  
  
  initializer->SetRadius( radius );

  if( argc > 5 )
  {
    double symmetryCoeff = atof( argv[5] );
    
    if( symmetryCoeff < 0.0 )
      symmetryCoeff = 0.0;
    else if( symmetryCoeff > 1.0 )
      symmetryCoeff = 1.0;

    initializer->SetSymmetryCoefficient( symmetryCoeff );
  }
  
  double threshold = itk::NumericTraits<double>::min();

  if( argc > 6 )
    threshold = atof( argv[6] );
    
  medialnessFilter->SetThreshold( threshold );
  medialnessFilter->SetImageFunctionInitializer( initializer );
  medialnessFilter->SetNumberOfThreads(4);  
  
  
  std::cout << "Computing offset medialness..." << std::endl;
  
  try
  {
    medialnessFilter->Update();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "EXCEPTION CAUGHT!!! " << excpt.GetDescription();
    return EXIT_FAILURE;
  }
  
  // Write result

  typedef itk::ImageFileWriter<ImageType>  WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetInput( medialnessFilter->GetOutput() );
  
  if( argc > 2 )
    writer->SetFileName( argv[2] );
  else
    writer->SetFileName( "OffsetMedialness.mhd" );

  try
  {
    writer->Update();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "EXCEPTION CAUGHT!!! " << excpt.GetDescription();
    return EXIT_FAILURE;
  }
  

  return EXIT_SUCCESS; 
}

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
// File: ivanMultiscaleOffsetMedialnessImageFunctionTest.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: tests MultiscaleImageFunction with the OffsetMedialnessImageFunction at multiple scales
// Date: 2012/02/23

#include "ivanMultiscaleImageFunction.h"
#include "ivanOffsetMedialnessImageFunction.h"
#include "ivanOffsetMedialnessImageFunctionInitializer.h"

#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"

#include <fstream>


template <class TImage, class TMedialnessFunction>
void ComputeMedialness( const TImage *input, TImage *output, TMedialnessFunction *medialness, double threshold = 0.0 )
{
  typedef itk::ImageRegionConstIterator<TImage>       ConstIteratorType;
  typedef itk::ImageRegionIteratorWithIndex<TImage>   IteratorType;

  ConstIteratorType it ( input,  input->GetRequestedRegion() );
  IteratorType      oit( output, input->GetRequestedRegion() );

  it.GoToBegin();
  oit.GoToBegin();

  typename TImage::IndexType currentIndex;
  unsigned long y = 0, z = 0;

  while( !it.IsAtEnd() )
  { 
    currentIndex = it.GetIndex();

    if( currentIndex[1] > y + 10 )
    {
      y += 10;
      std::cout << "Y = " << y << std::endl;
    }

    if( currentIndex[2] != z )
    {
      z = currentIndex[2];
      y = 0;
      std::cout << std::endl;
      std::cout << "Z = " << z << std::endl;
      std::cout << std::endl;
    }

    if( it.Get() < threshold )
      oit.Set( 0.0 );
    else    
      oit.Set( medialness->EvaluateAtIndex( it.GetIndex() ) );

    ++it;
    ++oit;
  } 
}


int main( int argc, const char *argv[] )
{
  if( argc < 6 )
  {
    std::cerr << "Usage: " << argv[0] << "inputImage outputImage numScales minScale maxScale [scaleIsRadius] [fixedRadiusOrSigma] [gradientSigma=1.0]"
      "[SymmetryCoefficient(0.0-1.0)=0.5] [threshold=0.0]" << std::endl;
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
    
  // Create the base and multiscale image function
  
  typedef ivan::OffsetMedialnessImageFunction<ImageType,double>              MedialnessFunctionType;
  typedef ivan::OffsetMedialnessImageFunctionInitializer<ImageType,double>   MedialnessFunctionInitializerType;
    
  typedef ivan::MultiscaleImageFunction<MedialnessFunctionType, ImageType, double>   MultiscaleMedialnessFunctionType;
    
    
  MultiscaleMedialnessFunctionType::Pointer multiscaleMedialness = MultiscaleMedialnessFunctionType::New();
    
  multiscaleMedialness->SetInputImage( reader->GetOutput() );
  
  MedialnessFunctionInitializerType::Pointer initializer = MedialnessFunctionInitializerType::New();
  initializer->SetGradientImageFunctionType( MedialnessFunctionType::GradientNormalProjectionFunctionHyperIntense );
 
  bool useRadiusAsScale = true;
 
  if( argc > 6 )
  {
    useRadiusAsScale = atoi( argv[6] );
		if( useRadiusAsScale )
			initializer->SetScaleSelectionMethod( MedialnessFunctionInitializerType::UseScaleAsRadius );
		else
			initializer->SetScaleSelectionMethod( MedialnessFunctionInitializerType::UseScaleAsSigma );
  }
  else
    initializer->SetScaleSelectionMethod( MedialnessFunctionInitializerType::UseScaleAsRadius );
 
  if( argc > 7 )
  {
    if( useRadiusAsScale )
      initializer->SetHessianSigma( atof( argv[7] ) );
    else
      initializer->SetRadius( atof( argv[7] ) );
  }
  else
    initializer->SetHessianSigma( 1.0 );
    
  if( argc > 8 )
    initializer->SetGradientSigma( atof( argv[8] ) );
  else
    initializer->SetGradientSigma( 1.0 );
 
  if( argc > 9 )
    initializer->SetSymmetryCoefficient( atof( argv[9] ) );
  else
    initializer->SetSymmetryCoefficient( 0.5 );
 
  multiscaleMedialness->SetScaledImageFunctionInitializer( initializer );
   
  multiscaleMedialness->SetNumberOfScales( atoi( argv[3] ) );
  multiscaleMedialness->SetMinimumScale( atoi( argv[4] ) );
  multiscaleMedialness->SetMaximumScale( atoi( argv[5] ) );
  multiscaleMedialness->Initialize();

  double threshold = 0.0;

  if( argc > 10 )
    threshold = atof( argv[10] );
  
  ComputeMedialness( reader->GetOutput(), output.GetPointer(), multiscaleMedialness.GetPointer(), threshold );


  // Write result

  typedef itk::ImageFileWriter<ImageType>  WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetInput( output );
  
  writer->SetFileName( argv[2] );
  writer->UseCompressionOn();
  
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

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
// File: ivanOffsetMedialnessImageFunctionTest.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: tests Frangi's vesselness measure in the form of an image function.
// Date: 2010/07/17

#include "ivanOffsetMedialnessImageFunction.h"

#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"

#include <fstream>


template <class TImage, class TMedialnessFunction>
void ComputeMedialness( const TImage *input, TImage *output, TMedialnessFunction *medialness, double threshold )
{
  typedef itk::ImageRegionConstIterator<TImage>       ConstIteratorType;
  typedef itk::ImageRegionIteratorWithIndex<TImage>   IteratorType;

  ConstIteratorType it ( input,  input->GetRequestedRegion() );
  IteratorType      oit( output, input->GetRequestedRegion() );

  it.GoToBegin();
  oit.GoToBegin();

  typename TImage::IndexType currentIndex;
  unsigned long z = 0;

  while( !it.IsAtEnd() )
  { 
    currentIndex = it.GetIndex();

    //if( currentIndex[0] == 20 && currentIndex[1] == 7 && currentIndex[2] == 5 ) // for cylinders
    if( currentIndex[0] == 82 && currentIndex[1] == 131 && currentIndex[2] == 13 ) // for liver MRI
    {
      std::cout << "Value: " << it.Get() << std::endl;
    }

    if( currentIndex[2] != z )
    {
      z = currentIndex[2];
      std::cout << "Z = " << z << std::endl;
    }

    if( it.Get() > threshold )
      oit.Set( medialness->EvaluateAtIndex( it.GetIndex() ) );
    else
      oit.Set(0);

    ++it;
    ++oit;
  } 
}


int main( int argc, const char *argv[] )
{
  if( argc < 2 )
  {
    std::cerr << "Usage: " << argv[0] << "InputImage [OutputImage] [Radius] [Sigma] [RadialResolution(0=adaptative)]"
      "[SymmetryCoefficient(0.0-1.0)] [Threshold]" << std::endl;
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
    
  // Create the image function
  
  typedef ivan::OffsetMedialnessImageFunction<ImageType,double>   MedialnessFunctionType;
  
  MedialnessFunctionType::Pointer medialness = MedialnessFunctionType::New();
  medialness->SetInputImage( reader->GetOutput() );

  if( argc > 4 )
    medialness->SetSigma( atof( argv[4] ) );
  else
    medialness->SetSigma( 2.0 );
    
  if( argc > 3 )
    medialness->SetRadius( atof( argv[3] ) );
  else
    medialness->SetRadius( vcl_sqrt( 3.0 ) * medialness->GetSigma() );

  if( argc > 5 )
  {
    if( atoi( argv[5] ) == 0 )
      medialness->AdaptativeSamplingOn();
    else
      medialness->SetRadialResolution( atoi( argv[5] ) );
  }
  else
    medialness->AdaptativeSamplingOn();    

  if( argc > 6 )
  {
    double symmetryCoeff = atof( argv[6] );
    
    if( symmetryCoeff < 0.0 )
      symmetryCoeff = 0.0;
    else if( symmetryCoeff > 1.0 )
      symmetryCoeff = 1.0;

    medialness->SetSymmetryCoefficient( symmetryCoeff );
  }

  medialness->Initialize();
  
  double threshold = itk::NumericTraits<double>::min();

  if( argc > 7 )
    threshold = atof( argv[7] );
  
  /////////////////////////////////////////
  // First try with the gradient projection
  
  medialness->SetGradientImageFunctionType( MedialnessFunctionType::GradientNormalProjectionFunctionHyperIntense );
  ComputeMedialness( reader->GetOutput(), output.GetPointer(), medialness.GetPointer(), threshold );

  // Write result

  typedef itk::ImageFileWriter<ImageType>  WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetInput( output );
  
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
  
  
  //////////////////////////////////
  // Now with the gradient magnitude
  
  medialness->SetGradientImageFunctionType( MedialnessFunctionType::GradientMagnitudeFunction );
  ComputeMedialness( reader->GetOutput(), output.GetPointer(), medialness.GetPointer(), threshold );

  // Write result

  writer->SetInput( output );
  
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

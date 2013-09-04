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
// File: ivanFrangiVesselnessImageFunctionTest.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: tests Sato's vesselness measure in the form of an image function.
// Date: 2010/08/16

#include "ivanSatoVesselnessImageFunction.h"

#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkRescaleIntensityImageFilter.h"

#include <fstream>


int main( int argc, const char *argv[] )
{
  if( argc < 5 )
  {
    std::cerr << "Usage: " << argv[0] << "InputImage [OutputImage] [Sigma] [Gamma12] [Gamma23] [Alpha] [Rescale=1]" << std::endl;
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

  // Create an output image
  
  ImageType::Pointer output = ImageType::New();
    
  output->SetRegions( reader->GetOutput()->GetLargestPossibleRegion() );
  output->SetSpacing( reader->GetOutput()->GetSpacing() );
  output->SetOrigin( reader->GetOutput()->GetOrigin() );
  output->SetDirection( reader->GetOutput()->GetDirection() );
  output->Allocate();
  output->FillBuffer( itk::NumericTraits<PixelType>::Zero );
    
  // Create the image function
  
  typedef ivan::SatoVesselnessImageFunction<ImageType,double>   VesselnessFunctionType;
  
  VesselnessFunctionType::Pointer vesselness = VesselnessFunctionType::New();
  vesselness->SetInputImage( reader->GetOutput() );

  if( argc > 3 )
    vesselness->SetSigma( atof( argv[3] ) );
  else
    vesselness->SetSigma( 2.0 );
  
  if( argc > 4 )
    vesselness->SetGamma12( atof( argv[4] ) );
  else
    vesselness->SetGamma12( 1.0 );
  
  if( argc > 5 )
    vesselness->SetGamma23( atof( argv[5] ) );
  else
    vesselness->SetGamma23( 1.0 );
  
  if( argc > 6 )
    vesselness->SetAlpha( atof( argv[6] ) );
  else
    vesselness->SetAlpha( 0.25 );
  
  vesselness->Initialize();
    
  typedef itk::ImageRegionConstIterator<ImageType>       ConstIteratorType;
  typedef itk::ImageRegionIteratorWithIndex<ImageType>   IteratorType;

  ConstIteratorType it ( reader->GetOutput(), reader->GetOutput()->GetRequestedRegion() );
  IteratorType      oit( output, reader->GetOutput()->GetRequestedRegion() );

  it.GoToBegin();
  oit.GoToBegin();

  while( !it.IsAtEnd() )
  { 
    oit.Set( vesselness->EvaluateAtIndex( it.GetIndex() ) );

    ++it;
    ++oit;
  }

  bool rescale = true; // default

  if( argc > 7 )
    rescale = (bool)atoi( argv[7] );


  // Rescale output so we can see the result clearly
  
  typedef itk::RescaleIntensityImageFilter<ImageType>  RescalerType;
  RescalerType::Pointer rescaler = RescalerType::New();
  rescaler->SetOutputMinimum( 0.0 );
  rescaler->SetOutputMaximum( 255.0 );
  rescaler->SetInput( output );
  
  // Write result

  typedef itk::ImageFileWriter<ImageType>  WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetInput( output );
  
  if( argc > 2 )
    writer->SetFileName( argv[2] );
  else
    writer->SetFileName( "SatoVesselness.mhd" );

  if( rescale )
    writer->SetInput( rescaler->GetOutput() );
  else
    writer->SetInput( output );

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

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
// File: ivanCircularSectionFluxImageFunctionTest.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: tests flux image function from Lesage et al.
// Date: 2010/09/10

#include "ivanCircularSectionFluxImageFunction.h"

#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkRescaleIntensityImageFilter.h"

#include <fstream>


int main( int argc, const char *argv[] )
{
  if( argc < 2 )
  {
    std::cerr << "Usage: " << argv[0] << "InputFileName [OutputFileName] [Sigma] [Threshold]" << std::endl;
    return EXIT_FAILURE;
  }

  typedef float      PixelType;
  typedef short      OutputPixelType;
  const unsigned int Dimension = 3;
  typedef itk::Image<PixelType,Dimension>        ImageType;
  typedef itk::Image<OutputPixelType,Dimension>  OutputImageType;
  
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
  
  typedef ivan::CircularSectionFluxImageFunction<ImageType,double>   VesselnessFunctionType;
  
  VesselnessFunctionType::Pointer vesselness = VesselnessFunctionType::New();
  vesselness->SetInputImage( reader->GetOutput() );

  if( argc > 3 )
    vesselness->SetSigma( atof( argv[3] ) );
  else
    vesselness->SetSigma( 2.0 );

  vesselness->Initialize();
    
  typedef itk::ImageRegionConstIterator<ImageType>       ConstIteratorType;
  typedef itk::ImageRegionIteratorWithIndex<ImageType>   IteratorType;

  ConstIteratorType it ( reader->GetOutput(), reader->GetOutput()->GetRequestedRegion() );
  IteratorType      oit( output, reader->GetOutput()->GetRequestedRegion() );

  it.GoToBegin();
  oit.GoToBegin();

  ImageType::IndexType currentIndex;

  double threshold = itk::NumericTraits<double>::min();

  if( argc > 4 )
    threshold = atof( argv[4] );

  unsigned long z = 0;

  while( !it.IsAtEnd() )
  { 
    currentIndex = it.GetIndex();

    if( currentIndex[0] == 20 && currentIndex[1] == 7 && currentIndex[2] == 5 ) // for cylinders
    //if( currentIndex[0] == 82 && currentIndex[1] == 131 && currentIndex[2] == 13 ) // for liver MRI
    {
      std::cout << "Value: " << it.Get() << std::endl;
    }

    if( currentIndex[2] != z )
    {
      z = currentIndex[2];
      std::cout << "Z = " << z << std::endl;
    }

    if( it.Get() > threshold )
      oit.Set( vesselness->EvaluateAtIndex( it.GetIndex() ) );
    else
      oit.Set(0);

    ++it;
    ++oit;
  }

  typedef itk::RescaleIntensityImageFilter<ImageType,OutputImageType>
    RescalerType;

  RescalerType::Pointer rescaler = RescalerType::New();
  rescaler->SetInput( output );
  rescaler->SetOutputMinimum(0);
  rescaler->SetOutputMaximum(4096);

  // Write result

  typedef itk::ImageFileWriter<OutputImageType>  WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetInput( rescaler->GetOutput() );
  
  if( argc > 2 )
    writer->SetFileName( argv[2] );
  else
    writer->SetFileName( "CircularSectionFluxVesselness.mhd" );

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

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
// File: ivanOptimallyOrientedFluxVesselnessImageFunctionTest.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: tests optimally oriented flux measure (Law & Chung 2008) in the form of an image function.
// Date: 2010/01/18

#include "ivanOptimallyOrientedFluxVesselnessImageFunction.h"
#include "ivanDiscreteGradientGaussianImageFunction.h"

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
    std::cerr << "Usage: " << argv[0] << "InputFileName Radius [OutputFileName] [Rescale=1]" << std::endl;
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
  
  double radius = atof( argv[2] );
  vesselness->SetRadius( radius );
  vesselness->SetInputImage( reader->GetOutput() );
  vesselness->Initialize(); // this computes the sphere grid
      
  typedef itk::ImageRegionConstIterator<ImageType>       ConstIteratorType;
  typedef itk::ImageRegionIteratorWithIndex<ImageType>   IteratorType;

  ConstIteratorType it ( reader->GetOutput(), reader->GetOutput()->GetRequestedRegion() );
  IteratorType      oit( output, reader->GetOutput()->GetRequestedRegion() );

  it.GoToBegin();
  oit.GoToBegin();

  unsigned int z = 0;
  std::cout << "Z = " << z << std::endl;

  while( !it.IsAtEnd() )
  { 
    if( !it.Get() )
      oit.Set(0);
    else
      oit.Set( vesselness->EvaluateAtIndex( it.GetIndex() ) );

    if( it.GetIndex()[2] != z )
    {
      z = it.GetIndex()[2];
      std::cout << "Z = " << z << std::endl;
    }
    
    //if( it.GetIndex()[0] == 10 && it.GetIndex()[1] == 10 && it.GetIndex()[2] == 20 )
      //std::cout << "Index " << it.GetIndex()[0] << " " << it.GetIndex()[1] << " " << it.GetIndex()[2] << std::endl;

    ++it;
    ++oit;
  }

  bool rescale = false; // default

  if( argc > 4 )
    rescale = (bool)atoi( argv[4] );

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
  
  if( argc > 3 )
    writer->SetFileName( argv[3] );
  else
    writer->SetFileName( "OptimallyOrientedFlux.mhd" );

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

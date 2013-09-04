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
// File: ivanMultiscalePolarProfileVesselnessImageFunctionTest.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: tests vesselness function at multiple scales based on polar 
// profile according to the implementation described in:
// X. Qian, M.P. Brennan, D.P. Dione, W.L. Dobrucki, M.P. Jackowski, 
// C.K. Breuer, A.J. Sinusas and X. Papademetris
// A Non-Parametric Vessel Detection Method for Complex Vascular Structures
// http://www.ncbi.nlm.nih.gov/pmc/articles/PMC2614119/
// Date: 2010/09/07


#include "ivanPolarProfileVesselnessImageFunction.h"

#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkStatisticsImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkImageRegionIteratorWithIndex.h"

#include <fstream>
#include <sstream>


int main( int argc, const char *argv[] )
{
  if( argc < 3 )
  {
    std::cerr << "Usage: " << argv[0] << "InputImage OutputImage [NumOfScales] [SphereRadiusMin]"
      "[SphereRadiusMax] [RadialResolution] [Beta] [Threshold]" << std::endl;
    return EXIT_FAILURE;
  }

  typedef float      PixelType;
  const unsigned int Dimension = 3;
  typedef itk::Image<PixelType,Dimension>  ImageType;
  
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
  
  typedef itk::StatisticsImageFilter<ImageType>   StatisticsCalculatorType;
  StatisticsCalculatorType::Pointer statistics = StatisticsCalculatorType::New();
  statistics->SetInput( reader->GetOutput() );
  
  try
  {
    statistics->Update();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "EXCEPTION CAUGHT!!! " << excpt.GetDescription();
    return EXIT_FAILURE;
  }
  
  double sigma = statistics->GetSigma();
  
  // Create an output image
  
  ImageType::Pointer output = ImageType::New();
    
  output->SetRegions( reader->GetOutput()->GetLargestPossibleRegion() );
  output->SetSpacing( reader->GetOutput()->GetSpacing() );
  output->SetOrigin( reader->GetOutput()->GetOrigin() );
  output->SetDirection( reader->GetOutput()->GetDirection() );
  output->Allocate();
  output->FillBuffer( itk::NumericTraits<PixelType>::Zero );
    
  // Create the image function
  
  typedef ivan::PolarProfileVesselnessImageFunction<ImageType,double>   VesselnessFunctionType;
  
  VesselnessFunctionType::Pointer vesselness = VesselnessFunctionType::New();
  vesselness->SetInputImage( reader->GetOutput() );
  
  unsigned int numberOfScales = 5;
  double radiusMin = 5.0, radiusMax = 15.0;
  
  if( argc > 3 )
    numberOfScales = atoi( argv[3] );

  if( argc > 4 )
    radiusMin = atof( argv[4] );
    
  if( argc > 5 )
    radiusMax = atof( argv[5] );
    
  if( argc > 6 )
    vesselness->SetRadialResolution( atoi( argv[6] ) );
  else
    vesselness->SetRadialResolution(5);
    
  if( argc > 7 )
    vesselness->SetBeta( atof( argv[7] ) );
  
  vesselness->SetSigma( sigma );
  
  double currentRadius = radiusMin;
  double radiusStep = ( radiusMax - radiusMin ) / (double)( numberOfScales - 1 );
  
  PixelType threshold = itk::NumericTraits<PixelType>::min();

  if( argc > 8 )
    threshold = static_cast<PixelType>( atof( argv[8] ) );
    
  ImageType::Pointer input  = reader->GetOutput();
  ImageType::Pointer source = reader->GetOutput();
  reader->GetOutput()->DisconnectPipeline();
  
  for( unsigned int i=0; i<numberOfScales; ++i )
  {
    currentRadius = radiusMin + (double)i * radiusStep;
    
    vesselness->SetRadius( currentRadius );
    vesselness->Initialize();
    
    typedef itk::ImageRegionConstIterator<ImageType>       ConstIteratorType;
    typedef itk::ImageRegionIteratorWithIndex<ImageType>   IteratorType;
  
    ConstIteratorType sit ( source, source->GetRequestedRegion() );
    ConstIteratorType it ( input, source->GetRequestedRegion() );
    IteratorType      oit( output, source->GetRequestedRegion() );
  
    sit.GoToBegin();
    it.GoToBegin();
    oit.GoToBegin();
  
    ImageType::IndexType currentIndex;
  
    unsigned int z = 0;
    double value = 0.0;
  
    while( !it.IsAtEnd() )
    { 
      currentIndex = it.GetIndex();
  
  	  if( currentIndex[2] != z )
  	  {
  	    z = currentIndex[2];
        std::cout << "Z = " << z << std::endl;
      }
  
      /*if( currentIndex[0] == 20 && currentIndex[1] == 31 && currentIndex[2] == 30 )
      {
        std::cout << "Value: " << it.Get() << std::endl;
      }*/
  
  	  if( currentIndex[0] == 73 && currentIndex[1] == 256-126 && currentIndex[2] == 60-25 )
      {
        std::cout << "Value: " << it.Get() << std::endl;
      }
      
      if( !i ) // only the first time
      {
        if( sit.Get() >= threshold )
          oit.Set( vesselness->EvaluateAtIndex( sit.GetIndex() ) );
        else
          oit.Set(0);
      }
      else
      {
        if( sit.Get() >= threshold )
        {
          value = vesselness->EvaluateAtIndex( sit.GetIndex() );
          
          if( value > it.Get() )
            oit.Set( value );          
        }
        else
          oit.Set(0);        
      }
  
      ++sit;
      ++it;
      ++oit;
    }
    
    input = output; // after first time operate on the same image      
  }
  
  // Rescale output so we can see the result clearly
  
  typedef itk::RescaleIntensityImageFilter<ImageType>  RescalerType;
  RescalerType::Pointer rescaler = RescalerType::New();
  rescaler->SetOutputMinimum( 0.0 );
  rescaler->SetOutputMaximum( 255.0 );
  rescaler->SetInput( output );
  
  try
  {
    rescaler->Update();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "EXCEPTION CAUGHT!!! " << excpt.GetDescription();
    return EXIT_FAILURE;
  }

  // Write result

  typedef itk::ImageFileWriter<ImageType>  WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetInput( rescaler->GetOutput() );
  writer->SetFileName( argv[2] );

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

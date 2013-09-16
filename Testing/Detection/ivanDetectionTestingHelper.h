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
// File: ivanDetectionTestingHelper.h
// Author: Ivan Macia (imacia@vicomtech.org)
// Description: 
// Date: 2013/09/16


#ifndef __ivanDetectionTestingHelper_h_
#define __ivanDetectionTestingHelper_h_


#include "itkImage.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"


template <class TImage, class TVesselnessImageFunction>
int DenseComputeVesselness( TImage *input, TVesselnessImageFunction *vesselness, 
  const char *outputFileName, bool rescale = true )
{
  TImage::Pointer output = TImage::New();
      
  output->SetRegions( input->GetLargestPossibleRegion() );
  output->SetSpacing( input->GetSpacing() );
  output->SetOrigin( input->GetOrigin() );
  output->SetDirection( input->GetDirection() );
  output->Allocate();
  output->FillBuffer( itk::NumericTraits<typename TImage::PixelType>::Zero );
        
  typedef itk::ImageRegionConstIterator<TImage>       ConstIteratorType;
  typedef itk::ImageRegionIteratorWithIndex<TImage>   IteratorType;
      
  ConstIteratorType it ( input,  input->GetRequestedRegion() );
  IteratorType      oit( output, input->GetRequestedRegion() );

  it.GoToBegin();
  oit.GoToBegin();

  while( !it.IsAtEnd() )
  { 
    oit.Set( vesselness->EvaluateAtIndex( it.GetIndex() ) );
    
    ++it;
    ++oit;
  }

  // Rescale output so we can see the result clearly
  
  typedef itk::RescaleIntensityImageFilter<TImage>  RescalerType;
  RescalerType::Pointer rescaler = RescalerType::New();
  rescaler->SetOutputMinimum( 0.0 );
  rescaler->SetOutputMaximum( 255.0 );
  rescaler->SetInput( output );
  
  // Write result

  typedef itk::ImageFileWriter<TImage>  WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName( outputFileName );
  
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



template <class TImage, class TVesselnessImageFunction>
int SparseComputeVesselness( TImage *input, TVesselnessImageFunction *vesselness, 
  const char *outputFileName, bool rescale = true )
{
  // Create a single row output image
  
  typedef typename TImage::PixelType       PixelType;
  typedef unsigned char                    OutputPixelType;
  typedef itk::Image<PixelType,2>          TestImageType;
  typedef itk::Image<OutputPixelType,2>    TestOutputImageType;
  
  TestImageType::Pointer output = TestImageType::New();
  
  TestImageType::RegionType             testRegion;
  TestImageType::RegionType::SizeType   testSize;
  TestImageType::RegionType::IndexType  testIndex;
  
  testSize[0]  = input->GetLargestPossibleRegion().GetSize()[0];
  testSize[1]  = 1;

  testIndex[0] = 0;
  testIndex[1] = 0;

  testRegion.SetSize ( testSize );
  testRegion.SetIndex( testIndex );
        
  output->SetRegions( testRegion );
  
  TestImageType::SpacingType testSpacing;
  testSpacing[0] = input->GetSpacing()[0];
  testSpacing[1] = input->GetSpacing()[1];
  
  output->SetSpacing( testSpacing );
  
  output->Allocate();
  output->FillBuffer( itk::NumericTraits<PixelType>::Zero );
  
  TImage::RegionType             inputRegion;
  TImage::RegionType::SizeType   inputSize;
  TImage::RegionType::IndexType  inputIndex;
  
  inputSize[0] = input->GetRequestedRegion().GetSize()[0];
  inputSize[1] = 1;
  inputSize[2] = 1;
  
  inputRegion.SetSize( inputSize );
  
  inputIndex[0] = 0;
  inputIndex[1] = input->GetLargestPossibleRegion().GetSize()[1] / 2;
  inputIndex[2] = input->GetLargestPossibleRegion().GetSize()[2] / 2;

  inputRegion.SetIndex( inputIndex );

  typedef itk::ImageRegionConstIterator<TImage>              ConstIteratorType;
  typedef itk::ImageRegionIteratorWithIndex<TestImageType>   IteratorType;
  
  ConstIteratorType it ( input, inputRegion );
  IteratorType      oit( output, output->GetRequestedRegion() );

  it.GoToBegin();
  oit.GoToBegin();

  while( !it.IsAtEnd() )
  { 
    oit.Set( vesselness->EvaluateAtIndex( it.GetIndex() ) );
    
    ++it;
    ++oit;
  }
  
  // Rescale output so we can see the result clearly
  
  typedef itk::RescaleIntensityImageFilter<TestImageType,TestOutputImageType>  RescalerType;
  RescalerType::Pointer rescaler = RescalerType::New();
  rescaler->SetOutputMinimum( 0.0 );
  rescaler->SetOutputMaximum( 255.0 );
  rescaler->SetInput( output );
  
  // Write result

  if( rescale )
  {
    typedef itk::ImageFileWriter<TestOutputImageType>  WriterType;
    
    WriterType::Pointer writer = WriterType::New();
    writer->SetFileName( outputFileName );
    writer->SetInput( rescaler->GetOutput() );
  
    try
    {
      writer->Update();
    }
    catch( itk::ExceptionObject & excpt )
    {
      std::cerr << "EXCEPTION CAUGHT!!! " << excpt.GetDescription();
      return EXIT_FAILURE;
    }
  }
  else
  {
    typedef itk::ImageFileWriter<TestImageType>  WriterType;
    
    WriterType::Pointer writer = WriterType::New();
    writer->SetFileName( outputFileName );
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
  }

  return EXIT_SUCCESS;
}

#endif // __ivanDetectionTestingHelper_h_

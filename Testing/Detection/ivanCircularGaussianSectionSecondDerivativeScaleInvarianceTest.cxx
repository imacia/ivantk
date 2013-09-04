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
// File: ivanCircularGaussianSectionSecondDerivativeScaleInvarianceTest.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: test that the response of the second derivative in x of the center of the tube is normalized
//   accross scales, yielding the same response
// Date: 2012/04/19


#include "ivanCircularGaussianSectionGenerator.h"

#include "itkGaussianDerivativeOperator.h"
#include "itkExtractImageFilter.h"
#include "itkNeighborhoodOperatorImageFilter.h"


int main( int argc, char *argv[] )
{
  // Verify the number of parameters in the command line
  if( argc < 3 )
  {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << " outputFileName sourceSigma numScales minScale maxScale [maxValue] [imageSize]" 
    << std::endl;
    return EXIT_FAILURE;
  }

  typedef float    PixelType;
  
  typedef ivan::CircularGaussianSectionGenerator<PixelType>   SectionGeneratorType;
  typedef SectionGeneratorType::ImageType                     SectionImageType;
    
  SectionGeneratorType sectionGenerator;
  sectionGenerator.SetSigma( atof( argv[2] ) );
  
  if( argc > 6 )
    sectionGenerator.SetMaxValue( atoi( argv[6] ) );
  else
    sectionGenerator.SetMaxValue( 255.0 );
    
  unsigned long imageSize;
  
  if( argc > 7 )
    imageSize = atoi( argv[7] );
  else
    imageSize = sectionGenerator.GetSigma() * 10.0;
    
  if( imageSize % 2 == 0 )
    imageSize += 1; // make it odd
    
  sectionGenerator.SetImageSize( imageSize );

  SectionImageType::Pointer sectionImage;

  try
  {
    sectionImage = sectionGenerator.Create();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "ITK exception caught!!!" << std::endl;
    std::cerr << excpt.GetDescription() << std::endl;
    return EXIT_FAILURE; 
  }

  typedef itk::Image<PixelType,1>   LineImageType;
  typedef itk::ExtractImageFilter<SectionImageType,LineImageType>  ExtractFilterType;

  ExtractFilterType::Pointer extractor = ExtractFilterType::New();
  extractor->SetInput( sectionImage );

  ExtractFilterType::InputImageRegionType             outputRegion;
  ExtractFilterType::InputImageRegionType::IndexType  outputIndex;
  ExtractFilterType::InputImageRegionType::SizeType   outputSize;
  
  outputIndex[0] = 0;
  outputIndex[1] = sectionImage->GetRequestedRegion().GetSize()[1] / 2;

  outputSize[0] = sectionImage->GetRequestedRegion().GetSize()[0];
  outputSize[1] = 0; // collapse

  outputRegion.SetIndex( outputIndex );
  outputRegion.SetSize( outputSize );

  extractor->SetExtractionRegion( outputRegion );
  
  try
  {
    extractor->Update();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "ITK exception caught!!!" << std::endl;
    std::cerr << excpt.GetDescription() << std::endl;
    return EXIT_FAILURE; 
  }

  LineImageType::Pointer lineImage = extractor->GetOutput();
  lineImage->DisconnectPipeline();  
  
  unsigned int numSigmas = atoi( argv[3] );  
  double minSigma = atof( argv[4] );
  double maxSigma = atof( argv[5] );
      
  double maxError = 0.005;
  unsigned int maxKernelWidth = 30;
  
  if( argc > 6 )
  {
    maxError = atof( argv[6] );
  }
  if( argc > 7 )
  {
    maxKernelWidth = atoi( argv[7] );
  }

  std::ofstream fileout;
  double sigma = minSigma;
  double sigmaInc = ( maxSigma - minSigma ) / ( numSigmas - 1 );
  
  LineImageType::IndexType centerIndex;
  LineImageType::SizeType  size = lineImage->GetRequestedRegion().GetSize();
    
  centerIndex[0] = size[0] / 2;
    
  typedef itk::GaussianDerivativeOperator<PixelType,1>    DerivativeOperatorType;
  typedef itk::NeighborhoodOperatorImageFilter<LineImageType, LineImageType, PixelType> OperatorFilterType;
  
  for( unsigned int i=0; i<numSigmas; ++i, sigma += sigmaInc )
  {
    double maxValue = 0.0;
    
    std::string fileName = argv[1];
    fileout.open( fileName.c_str(), std::ios_base::out );
        
    DerivativeOperatorType derivativeOp;
    derivativeOp.SetDirection(0);
    derivativeOp.SetMaximumKernelWidth( maxKernelWidth ); // for testing allow very large kernels, as much as necessary
    derivativeOp.SetMaximumError( maxError );
    derivativeOp.SetVariance( sigma * sigma );
    derivativeOp.SetSpacing( 1.0 );
    derivativeOp.SetOrder( 2 ); // we test the second derivative
    derivativeOp.SetNormalizeAcrossScale( true );
    derivativeOp.CreateDirectional();
  
    // Create a filter that applies the operator to the image
    OperatorFilterType::Pointer operatorFilter = OperatorFilterType::New();
    operatorFilter->SetOperator( derivativeOp );
    operatorFilter->SetInput( lineImage );
    
    try
    {
      operatorFilter->Update();
    }
    catch( itk::ExceptionObject & excpt )
    {
      std::cerr << "ITK exception caught!!!" << std::endl;
      std::cerr << excpt.GetDescription() << std::endl;
      return EXIT_FAILURE; 
    }
    
    // Now access the value of the central pixel
    PixelType centerValue = operatorFilter->GetOutput()->GetPixel( centerIndex );
    
    fileout << sigma << " " << centerValue << std::endl;
    std::cout << sigma << " " << centerValue << std::endl;
  }
  
  fileout.close();
  
  return EXIT_SUCCESS;
}

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
// File   : ivanMultiscaleDiscreteGaussianDerivativeImageFilterTest.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: calculates the gaussian derivative of a 1D image with a single 1 in the middle in order 
//   to test that the derivatives are working correctly and the maximum value accross scales is the same.


#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIterator.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkDiscreteGaussianDerivativeImageFilter.h"

#include <fstream>
#include <sstream>
#include <algorithm>


int main( int argc, char ** argv )
{
  // Verify the number of parameters in the command line
  if( argc < 6 )
  {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << " outputFileNameRoot order numSigmas minSigma maxSigma [maximum_error=0.005] [maximum_kernel_width=30]" << std::endl;
    return -1;
  }
  
  const unsigned int Dimension = 1;
  
  typedef   float             PixelType;
  typedef   itk::Image<PixelType, Dimension>          ImageType; 

  unsigned int order = atoi( argv[2] );
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
  
  // Create a 1D image (array) with a single one in the center
  
  ImageType::Pointer image = ImageType::New();
    
  ImageType::SpacingType spacing;
  spacing.Fill( 1.0 );
  
  image->SetSpacing( spacing );
  
  ImageType::RegionType             region;
  ImageType::RegionType::SizeType   size;
  ImageType::RegionType::IndexType  index;
    
  size[0] = maxKernelWidth;
  
  if( maxKernelWidth % 2 != 0 )
    size[0] += 1;
    
  index.Fill(0);
  
  region.SetSize( size );
  region.SetIndex( index );
  
  image->SetRegions( region );
  image->Allocate();
  image->FillBuffer( itk::NumericTraits<PixelType>::Zero );
  
  ImageType::IndexType centerIndex;
  centerIndex[0] = size[0] / 2;
  
  image->SetPixel( centerIndex, 1.0 );
  
  typedef itk::DiscreteGaussianDerivativeImageFilter< ImageType, ImageType >   DerivativeFilterType;
     
  std::ofstream fileout;
    
  double sigma = minSigma;
  double sigmaInc = ( maxSigma - minSigma ) / ( numSigmas - 1 );
    
  for( unsigned int i=0; i<numSigmas; ++i, sigma += sigmaInc )
  {
    double maxValue = 0.0;
    
    std::string fileName = argv[1];
    std::ostringstream floatStr;
    floatStr << sigma;
    
    fileName += floatStr.str();
    
    std::replace( fileName.begin(), fileName.end(), '.', '_' ); // replace floating point by underscore
    fileName += ".txt";
    
    fileout.open( fileName.c_str(), std::ios_base::out );
    
    fileout << sigma;
      
    DerivativeFilterType::Pointer derivativeFilter = DerivativeFilterType::New();
    derivativeFilter->SetNormalizeAcrossScale( true );
    derivativeFilter->SetInput( image );
    derivativeFilter->SetMaximumError( maxError );
    derivativeFilter->SetMaximumKernelWidth( maxKernelWidth );
    derivativeFilter->SetVariance( sigma * sigma );
    derivativeFilter->SetOrder( order );
    
    try
    {
      derivativeFilter->Update();
    }
    catch ( itk::ExceptionObject & err )
    {
      std::cout << "ExceptionObject caught !" << std::endl; 
      std::cout << err << std::endl; 
      return EXIT_FAILURE;
    }
     
    typedef   itk::ImageRegionConstIterator< ImageType >  ConstIteratorType;
        
    ConstIteratorType it ( derivativeFilter->GetOutput(), derivativeFilter->GetOutput()->GetRequestedRegion() );
    it.GoToBegin();
  
    while( !it.IsAtEnd() )
    {
      fileout << " " << it.Get();
      
      ++it;
    }
    
    fileout << std::endl;
    fileout.close();
  }
}





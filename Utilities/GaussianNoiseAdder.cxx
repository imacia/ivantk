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
// File: GaussianNoiseAdder.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: adds Gaussian noise to the source image. If signal value is provided it will be used as a reference.
//   Otherwise, a sigma will be calculated for the whole image.
// Date: 2010/10/16

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkStatisticsImageFilter.h"
#include "itkAdditiveGaussianNoiseImageFilter.h" // from Insight Journal 721 "Noise Simulation" contribution from G�ethan Lehmann


int main( int argc, char *argv[] )
{
  // Verify the number of parameters in the command line
  if( argc < 4 )
  {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << " InputFileName OutputFileName NoiseLevel(0-100%) [SignalValue]" 
    << std::endl;
    return EXIT_FAILURE;
  }

  const unsigned int Dimension = 3;
  typedef short    PixelType;
  typedef itk::Image<PixelType,Dimension>   ImageType;
    
  typedef itk::ImageFileReader<ImageType>   FileReaderType;
  
  FileReaderType::Pointer reader = FileReaderType::New();
  reader->SetFileName( argv[1] );
   
  try
  {
    reader->Update();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "ITK exception caught!!!" << std::endl;
    std::cerr << excpt.GetDescription() << std::endl;
    return EXIT_FAILURE; 
  }
  
  double sigmaSignal;
  
  if( argc > 4 )
  {
    sigmaSignal = atof( argv[4] );
  }    
  else
  {
    // Compute stdev of input signal
  
    typedef itk::StatisticsImageFilter<ImageType>  StatisticsFilterType;
      
    StatisticsFilterType::Pointer stats = StatisticsFilterType::New();
    stats->SetInput( reader->GetOutput() );
    
    try
    {
      stats->Update();
    }
    catch( itk::ExceptionObject & excpt )
    {
      std::cerr << "ITK exception caught!!!" << std::endl;
      std::cerr << excpt.GetDescription() << std::endl;
      return EXIT_FAILURE; 
    }
    
    sigmaSignal = stats->GetSigma();
  }
    
  double noiseLevel = atof( argv[3] );
  
  if( noiseLevel < 0.0 )
    noiseLevel = 0.0;
  else if( noiseLevel > 100.0 )
    noiseLevel = 100.0;
    
  double sigmaNoise = 0.01 * noiseLevel * sigmaSignal;
  
  typedef itk::AdditiveGaussianNoiseImageFilter<ImageType,ImageType>  GaussianNoiseFilterType;
  
  GaussianNoiseFilterType::Pointer gaussianNoise = GaussianNoiseFilterType::New();
  gaussianNoise->SetInput( reader->GetOutput() );
  gaussianNoise->SetStandardDeviation( sigmaNoise );
  
  typedef itk::ImageFileWriter<ImageType>   FileWriterType;

  FileWriterType::Pointer writer = FileWriterType::New();
  writer->SetFileName( argv[2] );
  writer->SetInput( gaussianNoise->GetOutput() );
  writer->UseCompressionOn();
  
  try
  {
    writer->Update();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "ITK exception caught!!!" << std::endl;
    std::cerr << excpt.GetDescription() << std::endl;
    return EXIT_FAILURE; 
  }
  
  return EXIT_SUCCESS;
}

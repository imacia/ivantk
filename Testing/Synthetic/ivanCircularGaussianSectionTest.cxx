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
// File: ivanCircularGaussianSectionTest.cxx
// Author: Iván Macía (imacia@vicomtech.org)
// Description: creates a Gaussian section of the specified size
// Date: 2010/09/17


#include "ivanCircularGaussianSectionGenerator.h"

#include "itkImageFileWriter.h"


int main( int argc, char *argv[] )
{
  // Verify the number of parameters in the command line
  if( argc < 3 )
  {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << " OutputFileName Sigma [Normalize=0] [MaxValue=255.0] [ImageSpacing] [ImageSize]" 
    << std::endl;
    return EXIT_FAILURE;
  }

  typedef short    PixelType;
  
  typedef ivan::CircularGaussianSectionGenerator<PixelType>   SectionGeneratorType;
  typedef SectionGeneratorType::ImageType                     SectionImageType;
    
  SectionGeneratorType sectionGenerator;
  sectionGenerator.SetSigma( atof( argv[2] ) );

  bool normalize = false;

  if( argc > 3 )
    normalize = atoi( argv[3] );

  sectionGenerator.SetNormalize( normalize );

  // This does not take effect if normalize is true
 
  double maxValue = 255.0;

  if( argc > 4 )
    maxValue = atof( argv[4] );
  
  sectionGenerator.SetMaxValue( maxValue );

  double imageSpacing = 1.0;

  if( argc > 5 )
    imageSpacing = atof( argv[5] );
  
  sectionGenerator.SetImageSpacing( imageSpacing );

  unsigned long sectionImageSize;
  
  if( argc > 6 )
    sectionImageSize = atoi( argv[6] );
  else
  {
    sectionImageSize = sectionGenerator.GetSigma() * 10.0 / imageSpacing;
    if( sectionImageSize % 2 == 0 )
      sectionImageSize += 1;
  }

  sectionGenerator.SetImageSize( sectionImageSize );
    
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
  
  
  typedef itk::ImageFileWriter<SectionImageType>   FileWriterType;
  
  FileWriterType::Pointer writer = FileWriterType::New();
  writer->SetFileName( argv[1] );
  writer->SetInput( sectionImage );
  
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

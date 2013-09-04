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
// File: CircularGaussianTubeSinusoidalRadius.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a set of Gaussian tubes with radius varying following a sinusoidal function
// Date: 2012/03/15


#include "ivanCircularGaussianStraightTubeSinusoidalRadiusGenerator.h"

#include "itkImageFileWriter.h"


int main( int argc, char *argv[] )
{
  // Verify the number of parameters in the command line
  if( argc < 6 )
  {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << " OutputFileName SigmaMin SigmaMax TubeHeight NumLobes [MaxValue] [ImageSpacing] "
      "[SectionImageSize]" 
    << std::endl;
    return EXIT_FAILURE;
  }

  typedef short    PixelType;
  
  typedef ivan::CircularGaussianStraightTubeSinuosidalRadiusGenerator<PixelType>   TubeGeneratorType;
  typedef TubeGeneratorType::ImageType                                             TubeImageType;
    
  TubeGeneratorType tubeGenerator;
  tubeGenerator.SetSigmaMin( atof( argv[2] ) );
  tubeGenerator.SetSigmaMax( atof( argv[3] ) );
  
  double height = atof( argv[4] );
  unsigned int numLobes = atoi( argv[5] );
  
  tubeGenerator.SetHeight( height );
  tubeGenerator.SetSemiPeriod( height / (double)numLobes ); 
  
  if( argc > 6 )
    tubeGenerator.SetMaxValue( atoi( argv[6] ) );
  else
    tubeGenerator.SetMaxValue( 255.0 );

  double imageSpacing = 1.0;

  if( argc > 7 )
    imageSpacing = atof( argv[7] );
  
  tubeGenerator.SetImageSpacing( imageSpacing );

  unsigned long sectionImageSize;
  
  if( argc > 8 )
    sectionImageSize = atoi( argv[8] );
  else
  {
    sectionImageSize = tubeGenerator.GetSigma() * 10.0 / imageSpacing;
    if( sectionImageSize % 2 == 0 )
      sectionImageSize += 1;
  }

  tubeGenerator.SetSectionImageSize( sectionImageSize );
   
  TubeImageType::Pointer tubeImage;
  
  try
  {
    tubeImage = tubeGenerator.Create();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "ITK exception caught!!!" << std::endl;
    std::cerr << excpt.GetDescription() << std::endl;
    return EXIT_FAILURE; 
  }
  
  
  typedef itk::ImageFileWriter<TubeImageType>   FileWriterType;
  
  FileWriterType::Pointer writer = FileWriterType::New();
  writer->SetFileName( argv[1] );
  writer->SetInput( tubeImage );
  writer->SetUseCompression( true );
  
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

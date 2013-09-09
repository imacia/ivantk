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
// File: ivanCircularBarTubeTest.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a set of Gaussian tubes
// Date: 2010/09/17


#include "ivanCircularBarTubeGenerator.h"

#include "itkImageFileWriter.h"


int main( int argc, char *argv[] )
{
  // Verify the number of parameters in the command line
  if( argc < 4 )
  {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << " OutputFileName Radius TubeHeight [MaxValue] [ImageSpacing] [SectionImageSize]" 
    << std::endl;
    return EXIT_FAILURE;
  }

  typedef short    PixelType;
  
  typedef ivan::CircularBarTubeGenerator<PixelType>   TubeGeneratorType;
  typedef TubeGeneratorType::ImageType           TubeImageType;
    
  TubeGeneratorType tubeGenerator;
  tubeGenerator.SetRadius( atof( argv[2] ) );
  tubeGenerator.SetHeight( atof( argv[3] ) );
  
  if( argc > 4 )
    tubeGenerator.SetMaxValue( atoi( argv[4] ) );
  else
    tubeGenerator.SetMaxValue( 255.0 );
    
  double imageSpacing = 1.0;

  if( argc > 5 )
    imageSpacing = atof( argv[5] );
  
  tubeGenerator.SetImageSpacing( imageSpacing );

  unsigned long sectionImageSize;

  if( argc > 6 )
    sectionImageSize = atoi( argv[6] );
  else
  {
    sectionImageSize = tubeGenerator.GetRadius() * 10.0;
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

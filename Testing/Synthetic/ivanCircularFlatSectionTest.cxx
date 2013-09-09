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
// File: ivanCircularBarSectionTest.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a Gaussian section of the specified size
// Date: 2010/09/17


#include "ivanCircularBarSectionGenerator.h"

#include "itkImageFileWriter.h"


int main( int argc, char *argv[] )
{
  // Verify the number of parameters in the command line
  if( argc < 3 )
  {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << " OutputFileName Radius [MaxValue] [ImageSize]" 
    << std::endl;
    return EXIT_FAILURE;
  }

  typedef short    PixelType;
  
  typedef ivan::CircularBarSectionGenerator<PixelType>   SectionGeneratorType;
  typedef SectionGeneratorType::ImageType                 SectionImageType;
    
  SectionGeneratorType sectionGenerator;
  sectionGenerator.SetRadius( atof( argv[2] ) );
  
  if( argc > 3 )
    sectionGenerator.SetMaxValue( atoi( argv[3] ) );
  else
    sectionGenerator.SetMaxValue( 255.0 );
    
  unsigned long imageSize;

  if( argc > 4 )
    imageSize = atoi( argv[4] );
  else
    imageSize = sectionGenerator.GetRadius() * 10.0;
    
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

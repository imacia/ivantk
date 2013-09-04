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
// File: ivanCurvedCircularGaussianTube2Test.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a curved tube with Gaussian profile using SimpleCurvedCenterlineGenerator
// Date: 2010/10/03

#include "ivanSimpleCurvedCenterlineGenerator.h"
#include "ivanCenterlineBasedGaussianTubeGenerator.h"

#include "itkImageFileWriter.h"


int main( int argc, char *argv[] )
{
  // Verify the number of parameters in the command line
  if( argc < 3 )
  {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << " OutputFileName Sigma Height MaxXOffset [SectionImageSizeX] [SectionImageSizeY] [Spacing] [MaxValue]" 
    << std::endl;
    return EXIT_FAILURE;
  }
  
  double sigma = atof( argv[2] );
  double height = atof( argv[3] );
  double maxXOffset = atof( argv[4] );

  unsigned int sectionImageSizeX, sectionImageSizeY;
  
  if( argc > 5 )
    sectionImageSizeX = atoi( argv[5] );
  else
    sectionImageSizeX = sigma * 30.0;

  if( sectionImageSizeX % 2 == 0 )
    sectionImageSizeX += 1; 
  
  if( argc > 6 )
    sectionImageSizeY = atoi( argv[6] );
  else
    sectionImageSizeY = sigma * 30.0;

  if( sectionImageSizeY % 2 == 0 )
    sectionImageSizeY += 1;


  double spacing = 1.0;

  if( argc > 7 )
    spacing = atof( argv[7] );
    
    
  // Create centerline       

  typedef short    PixelType;
  
  typedef ivan::SimpleCurvedCenterlineGenerator<double>   CenterlineGeneratorType;
  
  CenterlineGeneratorType centerlineGenerator;
  
  CenterlineGeneratorType::PointType origin;
  origin[0] = ( sectionImageSizeX / 2 ) * spacing - 0.5 * maxXOffset;
  origin[1] = ( sectionImageSizeY / 2 ) * spacing;
  origin[2] = 0.0;
  
  centerlineGenerator.SetOrigin( origin );
  centerlineGenerator.SetHeight( height );
  centerlineGenerator.SetNumberOfPoints( 101 );
  centerlineGenerator.SetMaxXOffset( maxXOffset );
  centerlineGenerator.RecenterOn();
  
  CenterlineGeneratorType::CenterlinePointer centerline = centerlineGenerator.Create();
  
  
  // Now create the tube itself
  
  typedef ivan::CenterlineBasedGaussianTubeGenerator<PixelType>   TubeGeneratorType;
  typedef TubeGeneratorType::ImageType                            TubeImageType;
    
  TubeGeneratorType tubeGenerator;
  tubeGenerator.SetSigma( sigma );
  tubeGenerator.SetOffset( 3.0 * sigma );
  tubeGenerator.SetSectionImageSize( sectionImageSizeX, sectionImageSizeY );
  tubeGenerator.SetHeight( height );
  tubeGenerator.SetImageSpacing( spacing );
  tubeGenerator.SetCenterline( centerline.GetPointer() );
    
  if( argc > 8 )
    tubeGenerator.SetMaxValue( atoi( argv[8] ) );
  else
    tubeGenerator.SetMaxValue( 255.0 );

    
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

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
// File: CircularGaussianTube.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a set of Gaussian tubes
// Date: 2010/09/17


#include "ivanCircularGaussianStraightTubeGenerator.h"

#include "itkImageFileWriter.h"


int main( int argc, char *argv[] )
{
  // Verify the number of parameters in the command line
  if( argc < 4 )
  {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << " OutputFileName Sigma TubeHeight [Normalize=0] [MaxValue=255.0] [ImageSpacing] "
      "[SectionImageSize(0=auto)] [WriteCenterline]" 
    << std::endl;
    return EXIT_FAILURE;
  }

  //typedef short    PixelType;
  typedef float PixelType;

  typedef ivan::CircularGaussianStraightTubeGenerator<PixelType>   TubeGeneratorType;
  typedef TubeGeneratorType::ImageType                             TubeImageType;
    
  TubeGeneratorType tubeGenerator;

  double sigma = atof( argv[2] );
  tubeGenerator.SetSigma( sigma );
  
  double height = atof( argv[3] );
  tubeGenerator.SetHeight( height );
  
  bool normalize = false;

  if( argc > 4 )
    normalize = atoi( argv[4] );

  tubeGenerator.SetNormalize( normalize );

  // This does not take effect if normalize is true
 
  if( !normalize )
  {
    double maxValue = 255.0;

    if( argc > 5 )
      maxValue = atof( argv[5] );
    
    tubeGenerator.SetRescale( true );
    tubeGenerator.SetMaxValue( maxValue );
  }

  double imageSpacing = 1.0;

  if( argc > 6 )
    imageSpacing = atof( argv[6] );
  
  tubeGenerator.SetImageSpacing( imageSpacing );

  unsigned long sectionImageSize = 0;
  
  if( argc > 7 )
    sectionImageSize = atoi( argv[7] );

  if( sectionImageSize == 0 )
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

  bool writeCenterline = false;

  if( argc > 8 )
    writeCenterline = (bool)atoi( argv[8] );

  if( writeCenterline )
  {
    std::string outputFileName = argv[1];
    std::string centerlineFileName = outputFileName.substr( 0, outputFileName.find_last_of( '.' ) );
    centerlineFileName += "_Centerline.txt";

    std::ofstream fileout;
    fileout.open( centerlineFileName.c_str(), std::ios::out );

    TubeImageType::IndexType tubeCenter;
    tubeCenter[0] = tubeImage->GetLargestPossibleRegion().GetSize()[0] / 2;
    tubeCenter[1] = tubeImage->GetLargestPossibleRegion().GetSize()[1] / 2;
    tubeCenter[2] = 0; // we will increment this

    fileout << "Center(x y z) Normal(x y z) Scale" << std::endl;

    for( unsigned int i=0; i<tubeImage->GetLargestPossibleRegion().GetSize()[2]; ++i, ++tubeCenter[2] )
      fileout << tubeCenter[0] << " " << tubeCenter[1] << " " <<
      tubeCenter[2] << " 0 0 1 " << sigma << std::endl;

    fileout.close();
  }
  
  return EXIT_SUCCESS;
}

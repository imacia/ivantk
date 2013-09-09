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
// File: ivanCircularGaussianHelixTest.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a circular helix (constant radius and torsion) with Gaussian section
// Date: 2010/08/24


#include "ivanCircularGaussianHelixGenerator.h"

#include "itkImageFileWriter.h"


int main( int argc, char *argv[] )
{
  // Verify the number of parameters in the command line
  if( argc < 4 )
  {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << " OutputFileName Sigma Radius UnitPitch Direction Spacing [StartAngle(grad)] [EndAngle(grad)] [ZeroAngleAtBottom] [MaxValue] [FullCircle] [NumberOfPts]" 
    << std::endl;
    return EXIT_FAILURE;
  }

  typedef short    PixelType;
  
  typedef ivan::CircularGaussianHelixGenerator<PixelType>   HelixGeneratorType;
  typedef HelixGeneratorType::ImageType                     HelixImageType;
    
  HelixGeneratorType helixGenerator;
  helixGenerator.SetSigma( atof( argv[2] ) );
  helixGenerator.SetRadius( atof( argv[3] ) );
  helixGenerator.SetUnitPitch( atof( argv[4] ) );
  helixGenerator.SetDirection( atoi( argv[5] ) );
  helixGenerator.SetImageSpacing( atof( argv[6] ) );

  if( argc > 7 )
    helixGenerator.SetStartAngle( vnl_math::pi * atof( argv[7] ) / 180.0 );

  if( argc > 8 )
    helixGenerator.SetEndAngle( vnl_math::pi * atof( argv[8] ) / 180.0 );

  if( argc > 9 )
    helixGenerator.SetZeroAngleStartsAtBottom( atoi( argv[9] ) );

  if( argc > 10 )
    helixGenerator.SetMaxValue( atoi( argv[10] ) );
  else
    helixGenerator.SetMaxValue( 255.0 );

  if( argc > 11 )
    helixGenerator.SetAlwaysImageAreaFullCircle( atoi( argv[11] ) );
  else
    helixGenerator.SetAlwaysImageAreaFullCircle( false );

  if( argc > 12 )
    helixGenerator.SetNumberOfPoints( atoi( argv[12] ) );
  else
    helixGenerator.SetAutoComputeNumberOfPoints( true );

  HelixImageType::Pointer helixImage;
  
  try
  {
    helixImage = helixGenerator.Create();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "ITK exception caught!!!" << std::endl;
    std::cerr << excpt.GetDescription() << std::endl;
    return EXIT_FAILURE; 
  }

  typedef itk::ImageFileWriter<HelixImageType>   FileWriterType;
  
  FileWriterType::Pointer writer = FileWriterType::New();
  writer->SetFileName( argv[1] );
  writer->SetInput( helixImage );
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
  

  // Write also the centerline

  std::string outputFileName = argv[1];
  std::string centerlineFileName;

  size_t extPos = outputFileName.find_last_of( '.' );
  
  if( extPos >= outputFileName.size() )
  {
    std::cerr << "Provided filename with no extension." << std::endl;
    return EXIT_FAILURE;
  }
  else if( extPos == 0 )
  {
    std::cerr << "Provided wrong filename." << std::endl;
    return EXIT_FAILURE;
  }

  centerlineFileName = outputFileName.substr( 0, extPos );
  centerlineFileName += ".txt";
  
  std::ofstream fileout;
  fileout.open( centerlineFileName.c_str() );
  fileout << "Index Center(x y z)" << std::endl;

  HelixGeneratorType::PointContainerType *centerline = helixGenerator.GetCenterline();
  
  for( unsigned int i=0; i<centerline->Size(); ++i )
  {
    fileout << centerline->at(i)[0] << " "
            << centerline->at(i)[1] << " "
            << centerline->at(i)[2] << " "
            << std::endl;    
  }

  fileout.close();


  return EXIT_SUCCESS;
}

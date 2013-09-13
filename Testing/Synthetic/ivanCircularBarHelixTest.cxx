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
// File: ivanCircularBarHelixTest.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a Helix with Gaussian section
// Date: 2010/06/16


#include "ivanCircularBarHelixGenerator.h"

#include "itkImageRegionIteratorWithIndex.h"
#include "itkImageFileWriter.h"

#include <fstream>


int main( int argc, char *argv[] )
{
  // Verify the number of parameters in the command line
  if( argc < 2 )
  {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << " OutputFileName TubeRadius HelixRadius UnitPitch "
    << "[StartAngle(grad)=30] [EndAngle(grad)]"
    << "[MaxValue=255.0] [Spacing=1.0] [Direction=0] [FullCircle=1] [NumberOfPts] " 
    << std::endl;
    return EXIT_FAILURE;
  }

  typedef short    PixelType;
  
  typedef ivan::CircularBarHelixGenerator<PixelType>        HelixGeneratorType;
  typedef HelixGeneratorType::ImageType                     HelixImageType;
    
  HelixGeneratorType helixGenerator;
  helixGenerator.SetTubeRadius( atof( argv[2] ) );
  helixGenerator.SetRadius( atof( argv[3] ) );
  helixGenerator.SetUnitPitch( atof( argv[4] ) );
  

  if( argc > 5 )
    helixGenerator.SetStartAngle( vnl_math::pi * atof( argv[5] ) / 180.0 );
  else
    helixGenerator.SetStartAngle( vnl_math::pi * 0 / 180.0 );

  if( argc > 6 )
    helixGenerator.SetEndAngle( vnl_math::pi * atof( argv[6] ) / 180.0 );
  else
    helixGenerator.SetStartAngle( vnl_math::pi * 180 / 180.0 );

  if( argc > 7 )
    helixGenerator.SetMaxValue( atof( argv[7] ) );
  else
    helixGenerator.SetMaxValue( 255.0 );

  if( argc > 8 )
      helixGenerator.SetImageSpacing( atof( argv[8] ) );
  else
      helixGenerator.SetImageSpacing( 1.0 );

  if( argc > 9 )
    helixGenerator.SetDirection( atoi( argv[9] ) );
  else
     helixGenerator.SetDirection( 0 ) ;

  if( argc > 10 )
    helixGenerator.SetAlwaysImageAreaFullCircle( atoi( argv[10] ) );
  else
    helixGenerator.SetAlwaysImageAreaFullCircle( true );

  if( argc > 11 )
    helixGenerator.SetNumberOfPoints( atoi( argv[11] ) );
  else
  {
    helixGenerator.SetAutoComputeNumberOfPoints( 50 );
    //helixGenerator.SetAutoComputeNumberOfPoints( true );
  }

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

  /* Write Helix tube in a .mhd file. */
  typedef itk::ImageFileWriter<HelixImageType>   FileWriterType;
  
  FileWriterType::Pointer writer = FileWriterType::New();
  writer->SetFileName( argv[1] );
  writer->SetInput( helixImage );
  writer->SetUseCompression( true ); // some datasets are very large due to large black regions
  
  try
  {
    std::cout<<"Writing helix"<<std::endl;
    writer->Update();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "ITK exception caught!!!" << std::endl;
    std::cerr << excpt.GetDescription() << std::endl;
    return EXIT_FAILURE; 
  }

  /* Create an image with the content of the centerline. */
  std::cout<<"Creating a centerline image pointer..."<<std::endl;

  typedef itk::Image<PixelType, 3 >   CenterlineImageType;
  CenterlineImageType::Pointer        centerlineImage = CenterlineImageType::New();

  CenterlineImageType::RegionType     region = helixImage->GetLargestPossibleRegion();
  CenterlineImageType::PointType      origin = helixImage->GetOrigin();
  CenterlineImageType::SpacingType    spacing = helixImage->GetSpacing();

  centerlineImage->SetRegions( region );
  centerlineImage->SetOrigin( origin );
  centerlineImage->SetSpacing( spacing );

  centerlineImage->Allocate();

  centerlineImage->FillBuffer( 0 );

  HelixGeneratorType::PointContainerType *centerlinePoints = helixGenerator.GetCenterline();

  typedef itk::ImageRegionIteratorWithIndex< CenterlineImageType > IteratorType ; 
  IteratorType it ( centerlineImage, region ) ; 

  it.GoToBegin();

  CenterlineImageType::IndexType centerlineIndex;
  
  for( unsigned int i=0; i<centerlinePoints->Size(); ++i )
  {
    // convert physical units to index units
    centerlineIndex[0] = vcl_floor ( ( centerlinePoints->at( i )[0] - origin[0] ) /spacing[0] );
    centerlineIndex[1] = vcl_floor ( ( centerlinePoints->at( i )[1] - origin[1] ) /spacing[1] );
    centerlineIndex[2] = vcl_floor ( ( centerlinePoints->at( i )[2] - origin[2] ) /spacing[2] );
    
    it.SetIndex( centerlineIndex );

    it.Set( 1 );
  }

  /* Write Centerline in a .mhd file. */
  std::cout<<"Creating a new centerline filename path..."<<std::endl;

  // Create new file name for the centerline
  std::string outputFileName = argv[1];
  std::string centerlineImageFileName;

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

  centerlineImageFileName = outputFileName.substr( 0, extPos );
  centerlineImageFileName += "_Centerline.mhd";

  std::cout<<"Writing centerline..."<<std::endl;
  typedef itk::ImageFileWriter<CenterlineImageType>   CenterlineWriterType;
  
  CenterlineWriterType::Pointer centerlineWriter = CenterlineWriterType::New();
  centerlineWriter->SetFileName( centerlineImageFileName );
  centerlineWriter->SetInput( centerlineImage );
  centerlineWriter->SetUseCompression( true ); // some datasets are very large due to large black regions
  
  try
  {
    std::cout<<"Writing centerline"<<std::endl;
    centerlineWriter->Update();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "ITK exception caught!!!" << std::endl;
    std::cerr << excpt.GetDescription() << std::endl;
    return EXIT_FAILURE; 
  }


  // Write also the centerline
  std::string centerlineFileName;

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

  
  /** Save HELIX parameters. */
  std::string helixParametersFileName = outputFileName.substr( 0, extPos );
  helixParametersFileName += "_Parameters.txt";
  
  std::ofstream parametersFileout;
  parametersFileout.open( helixParametersFileName.c_str() );

  parametersFileout<<helixGenerator.GetCenter()[0]<<";"<<helixGenerator.GetCenter()[1]<<"\n";

  parametersFileout<<helixGenerator.GetRadius()<<"\n";

  parametersFileout.close();

  
  return EXIT_SUCCESS;
}

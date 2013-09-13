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
// File: ivanCircularBarToroidTest.cxx
// Author: Iván Macía (imacia@vicomtech.org)
// Description: creates a toroid with Gaussian section
// Date: 2010/06/16


#include "ivanCircularBarToroidGenerator.h"

#include "itkImageFileWriter.h"

#include <fstream>


int main( int argc, char *argv[] )
{
  // Verify the number of parameters in the command line
  if( argc < 5 )
  {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << " OutputFileName "
                         << " TubeRadius "
                         << " Radius "
                         << " Direction "
                         << " Spacing "
                         << " [StartAngle(grad)] "
                         << " [EndAngle(grad)] "
                         << " [MaxValue] "
                         << " [FullCircle] "
                         << " [NumberOfPts]" << std::endl;
    return EXIT_FAILURE;
  }

  typedef short    PixelType;
  
  typedef ivan::CircularBarToroidGenerator<PixelType>        ToroidGeneratorType;
  typedef ToroidGeneratorType::ImageType                     ToroidImageType;
    
  ToroidGeneratorType toroidGenerator;
  toroidGenerator.SetTubeRadius( atof( argv[2] ) );
  toroidGenerator.SetRadius( atof( argv[3] ) );
  toroidGenerator.SetDirection( atoi( argv[4] ) );
  toroidGenerator.SetImageSpacing( atof( argv[5] ) );

  toroidGenerator.SetZOffset( 5 );

  if( argc > 6 )
    toroidGenerator.SetStartAngle( vnl_math::pi * atof( argv[6] ) / 180.0 );
  else 
    toroidGenerator.SetStartAngle( vnl_math::pi * 0 / 180.0 );

  if( argc > 7 )
    toroidGenerator.SetEndAngle( vnl_math::pi * atof( argv[7] ) / 180.0 );
  else
    toroidGenerator.SetEndAngle( vnl_math::pi * 360 / 180.0 );

  if( argc > 8 )
    toroidGenerator.SetMaxValue( atoi( argv[8] ) );
  else
    toroidGenerator.SetMaxValue( 255.0 );

  if( argc > 9 )
    toroidGenerator.SetAlwaysImageAreaFullCircle( atoi( argv[9] ) );
  else
    toroidGenerator.SetAlwaysImageAreaFullCircle( true );

  if( argc > 10 )
    toroidGenerator.SetNumberOfPoints( atoi( argv[10] ) );
  else
    toroidGenerator.SetAutoComputeNumberOfPoints( true );

  // Create toroid
  ToroidImageType::Pointer toroidImage;
  try
  {
    toroidImage = toroidGenerator.Create();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "ITK exception caught!!!" << std::endl;
    std::cerr << excpt.GetDescription() << std::endl;
    return EXIT_FAILURE; 
  }

   
  typedef itk::ImageFileWriter<ToroidImageType>   FileWriterType;
  
  FileWriterType::Pointer writer = FileWriterType::New();
  writer->SetFileName( argv[1] );
  writer->SetInput( toroidImage );
  writer->SetUseCompression( true ); // some datasets are very large due to large black regions
  
  try
  {
    std::cout<<"Writing toroid..."<<std::endl;
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

  CenterlineImageType::RegionType     region = toroidImage->GetLargestPossibleRegion();
  CenterlineImageType::PointType      origin = toroidImage->GetOrigin();
  CenterlineImageType::SpacingType    spacing = toroidImage->GetSpacing();

  centerlineImage->SetRegions( region );
  centerlineImage->SetOrigin( origin );
  centerlineImage->SetSpacing( spacing );

  centerlineImage->Allocate();

  centerlineImage->FillBuffer( 0 );

  // Write also the centerline .txt

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
  centerlineFileName += "_Centerline.txt";
  
  std::ofstream fileout;
  fileout.open( centerlineFileName.c_str() );
  fileout << "Index Center(x y z)" << std::endl;

  ToroidGeneratorType::PointContainerType *centerline = toroidGenerator.GetCenterline();

  /** Iterator to fill the centerline pointer as well as to write them into a .txt file. */

  typedef itk::ImageRegionIteratorWithIndex< CenterlineImageType > IteratorType ; 
  IteratorType it ( centerlineImage, region ) ; 

  it.GoToBegin();

  CenterlineImageType::IndexType centerlineIndex;
  CenterlineImageType::PointType centerlinePoint;
  
  for( unsigned int i=0; i<centerline->Size(); ++i )
  {
    centerlinePoint = centerline->at( i );
    fileout << centerlinePoint[0] << " "
            << centerlinePoint[1] << " "
            << centerlinePoint[2] << " "
            << std::endl; 

    centerlineIndex[0] = vcl_floor( ( centerlinePoint[0] - origin[0] ) / spacing[0] );
    centerlineIndex[1] = vcl_floor( ( centerlinePoint[1] - origin[1] ) / spacing[1] );
    centerlineIndex[2] = vcl_floor( ( centerlinePoint[2] - origin[2] ) / spacing[2] );

    it.SetIndex( centerlineIndex );

    it.Set( 1 );
  }

  fileout.close();

  /** Write centerline image .mhd*/

  std::cout<<"Write centerline.."<<std::endl;

  std::string centerlineImageFileName = outputFileName.substr( 0, outputFileName.find_last_of( '.' ) );
  centerlineImageFileName += "_Centerline.mhd";

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

  double length = toroidGenerator.GetRadius()*(toroidGenerator.GetEndAngle()-toroidGenerator.GetStartAngle())*vnl_math::pi/180;

  std::cout<<"Toroid length: "<<length<<std::endl;

  /** Save toroid parameters. */
  std::string toroidParametersFileName = outputFileName.substr( 0, extPos );
  toroidParametersFileName += "_Parameters.txt";
  
  std::ofstream parametersFileout;
  parametersFileout.open( toroidParametersFileName.c_str() );

  parametersFileout<<toroidGenerator.GetCenter()[0]<<";"<<toroidGenerator.GetCenter()[1]<<";"<<toroidGenerator.GetCenter()[2]<<"\n";

  parametersFileout<<toroidGenerator.GetRadius()<<"\n";

  parametersFileout.close();

  
  return EXIT_SUCCESS;
}

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
// File: CircularBarTube.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a set of Bar tubes
// Date: 2010/09/17


#include "ivanCircularBarStraightTubeGenerator.h"

#include "itkImageFileWriter.h"


int main( int argc, char *argv[] )
{
  std::cout<<"****** CIRCULAR BAR STRAIGHT TUBE TEST ****** "<<std::endl;
  // Verify the number of parameters in the command line
  if( argc < 3 )
  {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << " OutputFileName "
                         << " TubeRadius     "
                         << " TubeHeight     "
                         << " [MaxValue=255.0] [ImageSpacing] [ImageSectionSize] "<< std::endl;
    return EXIT_FAILURE;
  }

  //typedef short    PixelType;
  typedef float PixelType;

  typedef ivan::CircularBarStraightTubeGenerator<PixelType>   TubeGeneratorType;
  typedef TubeGeneratorType::ImageType                        TubeImageType;
    
  TubeGeneratorType tubeGenerator;

  double tubeRadius = atof( argv[2] );
  tubeGenerator.SetTubeRadius( tubeRadius );
  
  double height = atof( argv[3] );
  tubeGenerator.SetHeight( height );
  
  if( argc > 4 )
    tubeGenerator.SetMaxValue( atof( argv[4] ) );
  else
    tubeGenerator.SetMaxValue( 255.0 );

  if( argc > 5 )
     tubeGenerator.SetImageSpacing( atof( argv[5] )  );
  else
    tubeGenerator.SetImageSpacing( 1.0  );

  unsigned long sectionImageSize = 0;
  
  if( argc > 6 )
    sectionImageSize = atoi( argv[6] );

  if( sectionImageSize == 0 )
  {
    sectionImageSize = tubeGenerator.GetTubeRadius() * 10.0 / tubeGenerator.GetImageSpacing();
    if( sectionImageSize % 2 == 0 )
      sectionImageSize += 1;
  }

  tubeGenerator.SetSectionImageSize( sectionImageSize );

  tubeGenerator.SetOffset( 5 );
     
  TubeImageType::Pointer tubeImage;
  
  try
  {
    std::cout<<"Generate straight tube.."<<std::endl;
    tubeImage = tubeGenerator.Create();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "ITK exception caught!!!" << std::endl;
    std::cerr << excpt.GetDescription() << std::endl;
    return EXIT_FAILURE; 
  }

  std::cout<<"Write circular bar straight tube"<<std::endl;

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

  ///* Create an image with the content of the centerline. */
  //std::cout<<"Creating a centerline image pointer..."<<std::endl;

  //typedef itk::Image<PixelType, 3 >   CenterlineImageType;
  //CenterlineImageType::Pointer        centerlineImage = CenterlineImageType::New();

  //CenterlineImageType::RegionType     region = tubeImage->GetLargestPossibleRegion();
  //CenterlineImageType::PointType      origin = tubeImage->GetOrigin();
  //CenterlineImageType::SpacingType    spacing = tubeImage->GetSpacing();

  //centerlineImage->SetRegions( region );
  //centerlineImage->SetOrigin( origin );
  //centerlineImage->SetSpacing( spacing );

  //centerlineImage->Allocate();

  //centerlineImage->FillBuffer( 0 );

  ///** Iterator to fill the centerline pointer. */

  //typedef itk::ImageRegionIteratorWithIndex< CenterlineImageType > IteratorType ; 
  //IteratorType it ( centerlineImage, region ) ; 

  //it.GoToBegin();

  ///* Create a file text. */
  //std::string outputFileName = argv[1];
  //std::string centerlineFileTextName = outputFileName.substr( 0, outputFileName.find_last_of( '.' ) );
  //centerlineFileTextName += "_Centerline.txt";

  //std::ofstream fileout;
  //fileout.open( centerlineFileTextName.c_str(), std::ios::out );

  //// Get centerline
  //TubeGeneratorType::CenterlinePointer centerline = tubeGenerator.GetCenterline();

  //// Centerline index
  //CenterlineImageType::IndexType centerlineIndex;

  //for( unsigned int i = 0; i< centerline->size() ; ++i )
  //{
  //   fileout << centerline->at(i)[0] << ";" << centerline->at(i)[1] << ";" << centerline->at(i)[2] << std::endl;
  //   
  //   // convert physical units to index units
  //   centerlineIndex[0] = vcl_floor ( (  centerline->at(i)[0] - origin[0] ) /spacing[0] );
  //   centerlineIndex[1] = vcl_floor ( (  centerline->at(i)[1] - origin[1] ) /spacing[1] );
  //   centerlineIndex[2] = vcl_floor ( (  centerline->at(i)[2] - origin[2] ) /spacing[2] );
  //  
  //   it.SetIndex( centerlineIndex );

  //   it.Set( 1 ); 
  //}

  //fileout.close();



  ///** Get centerline points. */

  //unsigned int firstCenterlineSlice = tubeGenerator.GetOffset();
  //unsigned int lastCenterlineSlice = tubeGenerator.GetOffset() + tubeGenerator.GetHeight();

  //TubeImageType::IndexType tubeCenter;
  //tubeCenter[0] = tubeImage->GetLargestPossibleRegion().GetSize()[0] / 2;
  //tubeCenter[1] = tubeImage->GetLargestPossibleRegion().GetSize()[1] / 2;
  //tubeCenter[2] = firstCenterlineSlice; // we will increment this

  //CenterlineImageType::IndexType centerlineIndex;

  //
  //std::cout<<"firstCenterline: "<<firstCenterlineSlice<<std::endl;
  //std::cout<<"lastCenterline: "<<lastCenterlineSlice<<std::endl;

  //for( unsigned int i=firstCenterlineSlice; i<lastCenterlineSlice ; ++i, ++tubeCenter[2] )
  //{

  //  fileout << tubeCenter[0] << ";" << tubeCenter[1] << ";" << tubeCenter[2] << std::endl;

  //  // convert physical units to index units
  //  centerlineIndex[0] = vcl_floor ( ( tubeCenter[0] - origin[0] ) /spacing[0] );
  //  centerlineIndex[1] = vcl_floor ( ( tubeCenter[1] - origin[1] ) /spacing[1] );
  //  centerlineIndex[2] = vcl_floor ( ( tubeCenter[2] - origin[2] ) /spacing[2] );
  //  
  //  it.SetIndex( centerlineIndex );

  //  it.Set( 1 );
  //}

  //fileout.close();

  //std::cout<<"Write centerline.."<<std::endl;

  //std::string centerlineFileName = outputFileName.substr( 0, outputFileName.find_last_of( '.' ) );
  //centerlineFileName += "_Centerline.mhd";

  //typedef itk::ImageFileWriter<CenterlineImageType>   CenterlineWriterType;
  //
  //CenterlineWriterType::Pointer centerlineWriter = CenterlineWriterType::New();
  //centerlineWriter->SetFileName( centerlineFileName );
  //centerlineWriter->SetInput( centerlineImage );
  //centerlineWriter->SetUseCompression( true ); // some datasets are very large due to large black regions
  //
  //try
  //{
  //  std::cout<<"Writing centerline"<<std::endl;
  //  centerlineWriter->Update();
  //}
  //catch( itk::ExceptionObject & excpt )
  //{
  //  std::cerr << "ITK exception caught!!!" << std::endl;
  //  std::cerr << excpt.GetDescription() << std::endl;
  //  return EXIT_FAILURE; 
  //}

  //std::cout<<"Total Length = "<< tubeImage->GetLargestPossibleRegion().GetSize()[2] * spacing[2] <<std::endl;

  
  std::cout<<"The end"<<std::endl;
  
  return EXIT_SUCCESS;
}

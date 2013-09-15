/*=========================================================================

Image-based Vascular Analysis Toolkit (IVAN)

Copyright (c) 2012, Ivan Macia Oliver
Vicomtech Foundation, San Sebastian - Donostia (Spain)
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
// File: CircularGaussianNStraightTubesTest.cxx
// Author: Ivan Macia (imacia@vicomtech.org)
// Description: creates a dataset with N Gaussian tubes
// Date: 2013/09/15


#include "ivanCircularGaussianStraightTubeGenerator.h"

#include "itkPasteImageFilter.h"
#include "itkImageFileWriter.h"

#include <fstream>


int main( int argc, char *argv[] )
{
  // Verify the number of parameters in the command line
  if( argc < 4 )
  {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << " OutputFileName NumTubes SigmaMin SigmaMax TubeHeight [Normalize=0] [MaxValue=255.0] [ImageSpacing] "   
      << std::endl;
    return EXIT_FAILURE;
  }

  typedef unsigned char     PixelType;

  typedef ivan::CircularGaussianStraightTubeGenerator<PixelType>   TubeGeneratorType;
  typedef TubeGeneratorType::ImageType                             TubeImageType;
    
  TubeGeneratorType tubeGenerator;

  unsigned int numTubes = atoi( argv[2] );
  double sigmaMin = atof( argv[3] );
  double sigmaMax = atof( argv[4] );
  
  //tubeGenerator.SetSigma( sigma );
  
  double imageSpacing = 1.0;

  if( argc > 8 )
    imageSpacing = atof( argv[8] );
  
  tubeGenerator.SetImageSpacing( imageSpacing );
    
  unsigned long height = (unsigned long) ( atof( argv[5] ) / imageSpacing );
  tubeGenerator.SetHeight( height );
  
  
  bool normalize = false;

  if( argc > 6 )
    normalize = atoi( argv[6] );

  tubeGenerator.SetNormalize( normalize );

  // This does not take effect if normalize is true
 
  if( !normalize )
  {
    double maxValue = 255.0;

    if( argc > 7 )
      maxValue = atof( argv[7] );
    
    tubeGenerator.SetRescale( true );
    tubeGenerator.SetMaxValue( maxValue );
  }

  std::vector<unsigned long> sectionSizes( numTubes );
  std::vector<double>        sigmas( numTubes );
  	
  sigmas[0] = sigmaMin;
  sigmas[ sigmas.size()-1 ] = sigmaMax;
  
  if( sigmas.size() > 2 )
  {
  	double sigmaInc = ( sigmaMax - sigmaMin ) / (double)( sigmas.size() - 1 );
  	double currentSigma = sigmaMin;
  	  	
    for( unsigned int i=1; i < sigmas.size()-1; ++i )
    {
    	currentSigma += sigmaInc;
      sigmas[i] = currentSigma;    	
    }
  }
  	
  // Automatically estimate section sizes to accommodate the tubes
  
  unsigned long largestSectionSize = 0, totalWidth = 0;
  
  for( unsigned int i=0; i < numTubes; ++i )
  {
  	sectionSizes[i] = sigmas[i] * 6.0 / imageSpacing;
  	
    if( sectionSizes[i] % 2 == 0 )
      sectionSizes[i] += 1;
      
    totalWidth += sectionSizes[i];
  }
  
  largestSectionSize = sectionSizes[ sectionSizes.size()-1 ];
  
  
  // Create an image to hold all tubes
  
  TubeImageType::Pointer tubes = TubeImageType::New();
  	
  TubeImageType::SpacingType spacing;
  spacing.Fill( imageSpacing );
  
  tubes->SetSpacing( spacing );
  	
  TubeImageType::RegionType             region;
  TubeImageType::RegionType::SizeType   size;
  TubeImageType::RegionType::IndexType  index;
  	
  size[0] = totalWidth;
  size[1] = largestSectionSize;
  size[2] = height;
  
  index.Fill(0);
  
  region.SetSize( size );
  region.SetIndex( index );
  
  tubes->SetRegions( region );
  
  tubes->Allocate();
  tubes->FillBuffer(0);
  
  
  // Generate each tube and paste it into the final image
  
  unsigned long xindex = 0;

  TubeImageType::Pointer pasteImage = tubes;
  
  for( unsigned int i=0; i < numTubes; ++i )
  {
	  tubeGenerator.SetSectionImageSize( sectionSizes[i] );
    tubeGenerator.SetSigma( sigmas[i] );
	   
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

    typedef itk::PasteImageFilter<TubeImageType,TubeImageType,TubeImageType>  PasteFilterType;
	  	
	  PasteFilterType::Pointer paste = PasteFilterType::New();
	  paste->SetSourceImage( tubeImage );
	  paste->SetDestinationImage( pasteImage );
	  
	  PasteFilterType::InputImageIndexType pasteIndex;
	  	
	  pasteIndex[0] = xindex;
	  pasteIndex[1] = ( pasteImage->GetLargestPossibleRegion().GetSize()[1] 
      - tubeImage->GetLargestPossibleRegion().GetSize()[1] ) / 2;
	  pasteIndex[2] = 0;	
	  
	  paste->SetDestinationIndex( pasteIndex );
    paste->SetSourceRegion( tubeImage->GetLargestPossibleRegion() );
	  
	  try
	  {
	    paste->Update();
	  }
	  catch( itk::ExceptionObject & excpt )
	  {
	    std::cerr << "ITK exception caught!!!" << std::endl;
	    std::cerr << excpt.GetDescription() << std::endl;
	    return EXIT_FAILURE; 
	  }

    pasteImage = paste->GetOutput();
    pasteImage->DisconnectPipeline();

    xindex += sectionSizes[i];
	}

  typedef itk::ImageFileWriter<TubeImageType>   FileWriterType;
  
  FileWriterType::Pointer writer = FileWriterType::New();
  writer->SetFileName( argv[1] );
  writer->SetInput( pasteImage );
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

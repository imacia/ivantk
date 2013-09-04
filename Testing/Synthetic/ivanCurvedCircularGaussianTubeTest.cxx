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
// File: ivanCurvedCircularGaussianTubeTest.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: creates a curved tube with Gaussian profile
// Date: 2010/09/30

#include "ivanCurvedCircularGaussianTubeGenerator.h"

#include "itkImageFileWriter.h"


int main( int argc, char *argv[] )
{
  // Verify the number of parameters in the command line
  if( argc < 3 )
  {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << " OutputFileName Sigma TubeHeight MaxXOffset [SectionImageSize] [Spacing] [MaxValue]" 
    << std::endl;
    return EXIT_FAILURE;
  }

  typedef short    PixelType;
  
  typedef ivan::CurvedCircularGaussianTubeGenerator<PixelType>   TubeGeneratorType;
  typedef TubeGeneratorType::ImageType                           TubeImageType;
    
  TubeGeneratorType tubeGenerator;
  tubeGenerator.SetSigma( atof( argv[2] ) );
  tubeGenerator.SetHeight( atof( argv[3] ) );
  tubeGenerator.SetMaxXOffset( atof( argv[4] ) );
  
  if( argc > 7 )
    tubeGenerator.SetMaxValue( atoi( argv[7] ) );
  else
    tubeGenerator.SetMaxValue( 255.0 );

  double spacing = 1.0;

  if( argc > 6 )
    spacing = atof( argv[6] );
  
  tubeGenerator.SetImageSpacing( spacing );
  
  if( argc > 5 )
    tubeGenerator.SetSectionImageSize( atoi( argv[5] ) );
  else
    tubeGenerator.SetSectionImageSize( tubeGenerator.GetSigma() * 30.0 );
    
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

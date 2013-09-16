/*=========================================================================

Image-based Vascular Analysis Toolkit (IVAN)

Copyright (c) 2012, Ivan Macia Oliver
Vicomtech Foundation, San Sebastian - Donostia (Spain)
University of the Basque Country, San Sebastian - Donostia (Spain)

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
// File: ivanFilterByEigenValuesImageFunctionTest.cxx
// Author: Ivan Macia (imacia@vicomtech.org)
// Description: tests vesselness function that filters by eigenvalues
// Date: 2010/08/16

#include "ivanFilterByEigenValuesVesselnessImageFunction.h"
#include "ivanDetectionTestingHelper.h"

#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"

#include <fstream>


int main( int argc, const char *argv[] )
{
  if( argc < 2 )
  {
    std::cerr << "Usage: " << argv[0] << "InputImage Testmode(0-1) [OutputImage] [Sigma] [OutputValue]" << std::endl;
    return EXIT_FAILURE;
  }

  typedef unsigned char    PixelType;
  const unsigned int Dimension = 3;
  typedef itk::Image<PixelType,Dimension>       ImageType;
  
  typedef itk::ImageFileReader<ImageType>  ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName( argv[1] );

  try
  {
    reader->Update();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "EXCEPTION CAUGHT!!! " << excpt.GetDescription();
    return EXIT_FAILURE;
  }
  
  
  // Create the image function
  
  typedef ivan::FilterByEigenValuesVesselnessImageFunction<ImageType,short>   VesselnessFunctionType;
  
  VesselnessFunctionType::Pointer vesselness = VesselnessFunctionType::New();
  vesselness->SetInputImage( reader->GetOutput() );
  
  double sigma = 2.0;
  
  if( argc > 4 )
    sigma = atof( argv[4] );
    
  bool outputValue = 255.0;
  
  if( argc > 5 )
    outputValue = atof( argv[5] );
  
  vesselness->SetSigma( sigma );
  vesselness->SetOutputValue( outputValue );
  vesselness->Initialize();
  
  bool testMode = atoi( argv[2] );
    

  // While in test mode only iterate through a row
  
  if( !testMode )
  {
    std::string fileName;
    
    if( argc > 3 )
      fileName = argv[3];
    else
      fileName = "FilterByEigenvaluesVesselness.mhd";
    
    return DenseComputeVesselness( reader->GetOutput(), vesselness.GetPointer(), fileName.c_str(), false );
  }
  else
  {
    std::string fileName;
    
    if( argc > 3 )
      fileName = argv[3];
    else
      fileName = "FilterByEigenvaluesVesselness.png";
    
    return SparseComputeVesselness( reader->GetOutput(), vesselness.GetPointer(), fileName.c_str(), false );
  }
}

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
// File   : ivanMultiscaleGaussianDerivativeOperatorTest.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: computes Gaussian derivative operator at several scales to test that the derivatives are 
//   working correctly and the maximum value accross scales is the same.


#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkGaussianDerivativeOperator.h"

#include <fstream>
#include <sstream>
#include <algorithm>


int main( int argc, char ** argv )
{
  // Verify the number of parameters in the command line
  if( argc < 6 )
  {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << " outputFileNameRoot order numSigmas minSigma maxSigma [maximum_error=0.005] [maximum_kernel_width=30]" << std::endl;
    return -1;
  }
  
  const unsigned int Dimension = 1;
  
  typedef   float    PixelType;
  
  unsigned int order = atoi( argv[2] );
  unsigned int numSigmas = atoi( argv[3] );  
  double minSigma = atof( argv[4] );
  double maxSigma = atof( argv[5] );
      
  double maxError = 0.005;
  unsigned int maxKernelWidth = 30;
  
  if( argc > 6 )
  {
    maxError = atof( argv[6] );
  }
  if( argc > 7 )
  {
    maxKernelWidth = atoi( argv[7] );
  }

  std::ofstream fileout;
  double sigma = minSigma;
  double sigmaInc = ( maxSigma - minSigma ) / ( numSigmas - 1 );

  typedef itk::GaussianDerivativeOperator<PixelType,Dimension>   DerivativeOperatorType;
  
  for( unsigned int i=0; i<numSigmas; ++i, sigma += sigmaInc )
  {
    double maxValue = 0.0;
    
    std::string fileName = argv[1];
    std::ostringstream floatStr;
    floatStr << sigma;
    
    fileName += floatStr.str();
    
    std::replace( fileName.begin(), fileName.end(), '.', '_' ); // replace floating point by underscore
    fileName += ".txt";
    
    fileout.open( fileName.c_str(), std::ios_base::out );
    
    fileout << sigma << " ";
        
    DerivativeOperatorType derivativeOp;
    derivativeOp.SetDirection(0);
    derivativeOp.SetMaximumKernelWidth( maxKernelWidth ); // for testing allow very large kernels, as much as necessary
    derivativeOp.SetMaximumError( maxError );
    derivativeOp.SetVariance( sigma * sigma );
    derivativeOp.SetSpacing( 1.0 );
    derivativeOp.SetOrder( order );
    derivativeOp.SetNormalizeAcrossScale( true );
    derivativeOp.CreateDirectional();
  
    for( unsigned int i=0; i<derivativeOp.Size()-1; ++i )
    { 
      fileout   << derivativeOp[i] << " ";
      std::cout << derivativeOp[i] << " ";
    }
    
    fileout   << derivativeOp[derivativeOp.Size()-1] << std::endl;
    std::cout << derivativeOp[derivativeOp.Size()-1] << std::endl;

    fileout.close();
  }
}





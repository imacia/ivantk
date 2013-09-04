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
// File: ivanHessianEigenValuesImageFunctionTest.cxx
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Description: tests Frangi's vesselness measure in the form of an image function.
// Date: 2010/11/20

#include "ivanHessianEigenValuesImageFunction.h"

#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkImageAdaptor.h"
#include "itkCastImageFilter.h"

#include <fstream>


// Eigenvalue pixel accessor to access vector of eigen value pixels
// as individual images 
template< class TInputPixel, class TOutputPixel >
class EigenValueAccessor
{
public:
  typedef TInputPixel    InternalType;
  typedef TOutputPixel   ExternalType;
  
  inline ExternalType Get( const InternalType & input ) const 
    {
    return static_cast<ExternalType>( input[m_EigenIdx] );
    }
    
  inline void Set( InternalType & output, const ExternalType & input ) const
    {
    output[m_EigenIdx] = input;
    }

  void SetEigenIdx( unsigned int i )
    {
    this->m_EigenIdx = i;
    }
  
private:
  unsigned int m_EigenIdx;
};


int main( int argc, const char *argv[] )
{
  if( argc < 2 )
  {
    std::cerr << "Usage: " << argv[0] << "inputImage [scale = 2.0]" << std::endl;
    return EXIT_FAILURE;
  }

  typedef short       InputPixelType;
  const unsigned int Dimension = 3;
  typedef itk::Image<InputPixelType,Dimension>   InputImageType;
  
  typedef itk::ImageFileReader<InputImageType>  ReaderType;
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

  // First define the type of image function, since this determines the EigenValues image type
  
  typedef float   ValueType;
  typedef ivan::HessianEigenValuesImageFunction
    <InputImageType,ValueType>                         EigenValuesFunctionType;
  typedef EigenValuesFunctionType::OutputType          EigenValuesArrayType;
  
  // Create an output image
  
  typedef itk::Image<EigenValuesArrayType,Dimension>   EigenValuesImageType;
  typedef itk::Image<ValueType,Dimension>              EigenValuesComponentImageType;
  
  EigenValuesImageType::Pointer output = EigenValuesImageType::New();
    
  output->SetRegions( reader->GetOutput()->GetLargestPossibleRegion() );
  output->SetSpacing( reader->GetOutput()->GetSpacing() );
  output->SetOrigin( reader->GetOutput()->GetOrigin() );
  output->SetDirection( reader->GetOutput()->GetDirection() );
  output->Allocate();
  //output->FillBuffer( itk::NumericTraits<EigenValuesArrayType>::Zero );
    
  // Create the image function
  
  EigenValuesFunctionType::Pointer eigenValuesFunction = EigenValuesFunctionType::New();
  eigenValuesFunction->SetInputImage( reader->GetOutput() );
  
  if( argc > 2 )
    eigenValuesFunction->SetSigma( atof( argv[2] ) );
  else
    eigenValuesFunction->SetSigma( 2.0 );
    
  eigenValuesFunction->Initialize();
    
  typedef itk::ImageRegionConstIterator<InputImageType>             ConstIteratorType;
  typedef itk::ImageRegionIteratorWithIndex<EigenValuesImageType>   IteratorType;

  ConstIteratorType it ( reader->GetOutput(), reader->GetOutput()->GetRequestedRegion() );
  IteratorType      oit( output, reader->GetOutput()->GetRequestedRegion() );

  it.GoToBegin();
  oit.GoToBegin();

  while( !it.IsAtEnd() )
  { 
    oit.Set( eigenValuesFunction->EvaluateAtIndex( it.GetIndex() ) );

    ++it;
    ++oit;
  }
  
  
  // Create an image adaptor to extract individual eigenvalues 
  
  typedef EigenValueAccessor<EigenValuesArrayType, ValueType>   EigenValuesAccessorType;
  typedef itk::ImageAdaptor< EigenValuesImageType, 
    EigenValuesAccessorType > 		  EigenValuesAdaptorType;
    
        
  // First eigenvalue

	EigenValuesAdaptorType::Pointer eigenAdaptor1 = EigenValuesAdaptorType::New();
  EigenValuesAccessorType accessor1;
  accessor1.SetEigenIdx( 0 );
  eigenAdaptor1->SetImage( output );
  eigenAdaptor1->SetPixelAccessor( accessor1 );

  // Create a cast filter. This is necessary because we cannot connect and adaptor
  // directly to a writer

  typedef itk::Image<ValueType, Dimension>      OutputImageType;
  typedef itk::CastImageFilter<EigenValuesAdaptorType, OutputImageType>  CastFilterType;

  CastFilterType::Pointer caster = CastFilterType::New();
  caster->SetInput( eigenAdaptor1 );
  
  // Create writer
    
  typedef itk::ImageFileWriter<OutputImageType>  WriterType;
  WriterType::Pointer writer = WriterType::New();

  writer->SetInput( caster->GetOutput() );
  writer->SetFileName( "EigenValue1.mhd" );
  
  try
  {
    writer->Update();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "EXCEPTION CAUGHT!!! " << excpt.GetDescription();
    return EXIT_FAILURE;
  }

  // Second eigenvalue

	EigenValuesAdaptorType::Pointer eigenAdaptor2 = EigenValuesAdaptorType::New();
  EigenValuesAccessorType accessor2;
  accessor2.SetEigenIdx( 1 );
  eigenAdaptor2->SetImage( output );
  eigenAdaptor2->SetPixelAccessor( accessor2 );

  caster->SetInput( eigenAdaptor2 );
  
  writer->SetFileName( "EigenValue2.mhd" );
  
  try
  {
    writer->Update();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "EXCEPTION CAUGHT!!! " << excpt.GetDescription();
    return EXIT_FAILURE;
  }

	// Third eigenvalue

	EigenValuesAdaptorType::Pointer eigenAdaptor3 = EigenValuesAdaptorType::New();
  EigenValuesAccessorType accessor3;
  accessor3.SetEigenIdx( 2 );
  eigenAdaptor3->SetImage( output );
  eigenAdaptor3->SetPixelAccessor( accessor3 );
  
  caster->SetInput( eigenAdaptor3 );

  writer->SetFileName( "EigenValue3.mhd" );
  
  try
  {
    writer->Update();
  }
  catch( itk::ExceptionObject & excpt )
  {
    std::cerr << "EXCEPTION CAUGHT!!! " << excpt.GetDescription();
    return EXIT_FAILURE;
  }  

  return EXIT_SUCCESS; 
}

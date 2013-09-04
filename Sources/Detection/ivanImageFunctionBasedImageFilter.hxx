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
// File : ivanImageFunctionBasedImageFilter.hxx
// Author : Iv�n Mac�a (imacia@vicomtech.org)
// Description : applies an image function to the whole input image as an image filter, allowing parallelization.
// Date: 2012/07/04

#ifndef __ivanImageFunctionBasedImageFilter_hxx
#define __ivanImageFunctionBasedImageFilter_hxx

#include "ivanImageFunctionBasedImageFilter.h"

#include "itkImageRegionIterator.h"
#include "itkNumericTraits.h"


namespace ivan
{

template <class TInputImage, class TOutputImage, class TImageFunction>
ImageFunctionBasedImageFilter<TInputImage,TOutputImage,TImageFunction>
::ImageFunctionBasedImageFilter() :
  m_Threshold( itk::NumericTraits<InputImagePixelType>::Zero )
{


}


template <class TInputImage, class TOutputImage, class TImageFunction>
void 
ImageFunctionBasedImageFilter<TInputImage,TOutputImage,TImageFunction>
::BeforeThreadedGenerateData()
{
  typename TInputImage::Pointer inputImage = const_cast<TInputImage*>( this->GetInput() );

  for( unsigned int i=0; i<this->GetNumberOfThreads(); ++i )
  {
    ImageFunctionPointer imageFunction = ImageFunctionType::New();
    this->m_ImageFunctionInitializer->Initialize( imageFunction.GetPointer(), inputImage );
    this->m_ImageFunctionContainer.push_back( imageFunction );
  }
}


template <class TInputImage, class TOutputImage, class TImageFunction>
void 
ImageFunctionBasedImageFilter<TInputImage,TOutputImage,TImageFunction>
::ThreadedGenerateData( const OutputImageRegionType& outputRegionForThread, int threadId )
{
  typedef itk::ImageRegionConstIterator<TInputImage>       ConstIteratorType;
  typedef itk::ImageRegionIteratorWithIndex<TInputImage>   IteratorType;
  
  typename TInputImage::Pointer  input = const_cast<TInputImage*>( this->GetInput() );
  typename TOutputImage::Pointer output = this->GetOutput();

  ConstIteratorType it ( input,  outputRegionForThread );
  IteratorType      oit( output, outputRegionForThread );

  it.GoToBegin();
  oit.GoToBegin();

  typename TInputImage::IndexType currentIndex;
  unsigned long z = 0;

  while( !it.IsAtEnd() )
  { 
    currentIndex = it.GetIndex();

    //if( currentIndex[0] == 20 && currentIndex[1] == 7 && currentIndex[2] == 5 ) // for cylinders
    //if( currentIndex[0] == 82 && currentIndex[1] == 131 && currentIndex[2] == 13 ) // for liver MRI
    //{
      //std::cout << "Value: " << it.Get() << std::endl;
    //}

    if( currentIndex[2] != z )
    {
      z = currentIndex[2];
      std::cout << "Z = " << z << std::endl;
    }

    if( it.Get() > this->m_Threshold )
      oit.Set( this->m_ImageFunctionContainer[threadId]->EvaluateAtIndex( it.GetIndex() ) );
    else
      oit.Set(0);

    ++it;
    ++oit;
  }
}


template <class TInputImage, class TOutputImage, class TImageFunction>
void
ImageFunctionBasedImageFilter<TInputImage,TOutputImage,TImageFunction>
::PrintSelf(std::ostream & os, itk::Indent indent) const
{
  Superclass::PrintSelf(os, indent);

  // MISSING
}

} // end namespace ivan

#endif

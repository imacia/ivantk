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
// File : ivanImageFunctionBasedImageFilter.h
// Author : Iv�n Mac�a (imacia@vicomtech.org)
// Description : applies an image function to the whole input image as an image filter, allowing parallelization.
// Date: 2012/07/04

#ifndef __ivanImageFunctionBasedImageFilter_h
#define __ivanImageFunctionBasedImageFilter_h


#include "itkImageToImageFilter.h"
#include "itkImageFunction.h"

#include <vector>


namespace ivan
{

/** \class ImageFunctionBasedImageFilter
 *  \brief Applies an image function to the whole input image as an image filter, allowing parallelization.
 *
 * 
 * 
 */
template <class TInputImage, class TOutputImage, class TImageFunction>
class ITK_EXPORT ImageFunctionBasedImageFilter : public itk::ImageToImageFilter<TInputImage,TOutputImage>
{
public:

  /** Standard class typedefs. */
  typedef ImageFunctionBasedImageFilter                       Self;
  typedef itk::ImageToImageFilter<TInputImage,TOutputImage>  Superclass;
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;
  
  typedef TInputImage		InputImageType;
  typedef TOutputImage  OutputImageType;
  
  typedef typename InputImageType::PixelType     InputImagePixelType;
  typedef typename OutputImageType::RegionType   OutputImageRegionType;
  
  /** Image dimension = 3. */
  itkStaticConstMacro( ImageDimension, unsigned int, ITKImageDimensionMacro( InputImageType ) );
                      
  /** Image Function type. */
  typedef TImageFunction			                   ImageFunctionType;
  typedef typename ImageFunctionType::Pointer    ImageFunctionPointer;
  
  typedef std::vector<ImageFunctionPointer>      ImageFunctionContainerType;
  
  typedef ImageFunctionInitializerBase<TImageFunction,TInputImage>   ImageFunctionInitializerType; 
  typedef typename ImageFunctionInitializerType::Pointer             ImageFunctionInitializerPointer;
       
public:

  /** Method for creation through the object factory. */
  itkNewMacro( Self );  

  /** Run-time type information (and related methods). */
  itkTypeMacro( ImageFunctionBasedImageFilter, itk::ImageToImageFilter );
  
  itkSetMacro( Threshold, InputImagePixelType );
  itkGetConstMacro( Threshold, InputImagePixelType );
  
  /** Set the image input of this process object. Overriden from ImageToImageFilter.
    * Sets the input also to the existing scale filters. */
  //virtual void SetInput( const InputImageType *image );
  //virtual void SetInput( unsigned int index, const TInputImage *image );
  
  itkSetObjectMacro( ImageFunctionInitializer, ImageFunctionInitializerType );
  itkGetObjectMacro( ImageFunctionInitializer, ImageFunctionInitializerType );
  itkGetConstObjectMacro( ImageFunctionInitializer, ImageFunctionInitializerType );
  
protected:

  ImageFunctionBasedImageFilter();
  ~ImageFunctionBasedImageFilter() {};
  
  /** Compute image function in each thread. */
  virtual void ThreadedGenerateData( const OutputImageRegionType& outputRegionForThread, int threadId );


  /** Compute image function for each of the threads. */
  virtual void BeforeThreadedGenerateData();
  
  /**  */
  virtual void AfterThreadedGenerateData() {}
  
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;
  
private:

  ImageFunctionBasedImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

protected:

	/** Contains the different hessian function for the different scales. */  
  ImageFunctionContainerType            m_ImageFunctionContainer;
  
  /** Provide a means to initialize the image function since we have to create one for each thread. */
  ImageFunctionInitializerPointer       m_ImageFunctionInitializer;
  
  InputImagePixelType                   m_Threshold;
};

  
} // end namespace ivan
  
// Define instantiation macro for this template.
#define ITK_TEMPLATE_ImageFunctionBasedImageFilter(_, EXPORT, x, y) namespace ivan { \
  _(2(class EXPORT ImageFunctionBasedImageFilter< ITK_TEMPLATE_2 x >)) \
  namespace Templates { typedef ImageFunctionBasedImageFilter< ITK_TEMPLATE_2 x > \
                                                  ImageFunctionBasedImageFilter##y; } \
  }

#if ITK_TEMPLATE_EXPLICIT
# include "Templates/ivanImageFunctionBasedImageFilter+-.h"
#endif

#if ITK_TEMPLATE_TXX
# include "ivanImageFunctionBasedImageFilter.hxx"
#endif
  
#endif

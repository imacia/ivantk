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
// File: ivanScaleSpaceImageFunctionInitializer.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2012/02/07


#ifndef __ivanScaleSpaceImageFunctionInitializer_h
#define __ivanScaleSpaceImageFunctionInitializer_h


#include "ivanImageFunctionInitializerBase.h"


namespace ivan
{


/** \class ScaleSpaceImageFunctionInitializer
 *  \brief Initializer for scale-space metric functions.
 *
 * Specific initializations for some scale-space based metric image functions. This is used in 
 * multi-scale approaches in order to initialize the functions that operate at different scales.
 *
 */
template <class TImageFunction, class TImage>
class ITK_EXPORT ScaleSpaceImageFunctionInitializer : 
  public ImageFunctionInitializerBase<TImageFunction,TImage>
{
public:
  
  typedef ScaleSpaceImageFunctionInitializer
    <TImageFunction,TImage>                    Self;
  typedef ImageFunctionInitializerBase
    <TImageFunction,TImage>                    Superclass;
  typedef itk::SmartPointer<Self>              Pointer;
  typedef itk::SmartPointer<const Self>        ConstPointer;

  typedef TImageFunction                       ImageFunctionType;
  typedef TImage                               ImageType;
    
public:
  
  itkNewMacro( Self );
  itkTypeMacro( ScaleSpaceImageFunctionInitializer, ImageFunctionInitializerBase );
  
  virtual void Initialize( ImageFunctionType *imageFunction, const ImageType *image = 0, double scale = 0.0 )
    {
      imageFunction->SetInputImage( image );
      imageFunction->SetSigma( scale );      
      imageFunction->SetNormalizeAcrossScale( true );
      imageFunction->SetUseImageSpacing( true );
      imageFunction->Initialize();
    }
  
protected:
  
  ScaleSpaceImageFunctionInitializer() {}
  virtual ~ScaleSpaceImageFunctionInitializer() {}
  
private:
  
  ScaleSpaceImageFunctionInitializer(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented  
};

} // end namespace ivan

#endif

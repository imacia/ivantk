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
// File: ivanImageFunctionInitializerBase.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2012/02/07


#ifndef __ivanImageFunctionInitializerBase_h
#define __ivanImageFunctionInitializerBase_h


#include "itkLightObject.h"


namespace ivan
{
  
/** \class ImageFunctionInitializerBase
 *  \brief Base class for initializing metric functions.
 *
 * Derive from this class to provide specific initializations for some image functions.
 * This is used for example in multi-scale approaches in order to initialize the functions
 * that operate at different scales.
 *
 * The second parameter TImage is provided for flexibility, but tipically, it should be
 * of the same type of the input image of the image function.
 *
 */
template <class TImageFunction, class TImage>
class ITK_EXPORT ImageFunctionInitializerBase : public itk::LightObject
{
public:
  
  typedef ImageFunctionInitializerBase
    <TImageFunction,TImage>                Self;
  typedef itk::LightObject                 Superclass;
  typedef itk::SmartPointer<Self>          Pointer;
  typedef itk::SmartPointer<const Self>    ConstPointer;

  typedef TImageFunction                   ImageFunctionType;
  typedef TImage                           ImageType;
  
public:
  
  itkNewMacro( Self );
  itkTypeMacro( ImageFunctionInitializerBase, itk::LightObject );
  
  virtual void Initialize( ImageFunctionType *imageFunction, const ImageType *image = 0, double scale = 0.0 ) {}
  
protected:
  
  ImageFunctionInitializerBase() {}
  virtual ~ImageFunctionInitializerBase() {}
  
private:
  
  ImageFunctionInitializerBase(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented  
};

} // end namespace ivan

#endif

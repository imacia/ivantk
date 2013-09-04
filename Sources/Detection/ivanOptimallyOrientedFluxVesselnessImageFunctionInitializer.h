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
// File: ivanOptimallyOrientedFluxVesselnessImageFunctionInitializer.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2012/02/08


#ifndef __ivanOptimallyOrientedFluxVesselnessImageFunctionInitializer_h
#define __ivanOptimallyOrientedFluxVesselnessImageFunctionInitializer_h


#include "ivanImageFunctionInitializerBase.h"

#define OFFSET_MEDIALNESS_INITIALIZER_GRADIENT_SIGMA 1.0


namespace ivan
{
  
/** \class OptimallyOrientedFluxVesselnessImageFunctionInitializer
 *  \brief Initializer for Optimally Oriented Flux (OOF) vesselness metric
 *
 * Specific initialization for OOF vesselness, where the scale affects the radius of the sphere
 * used to calculate the oriented flux and not the scale of the derivative calculations.
 *
 */
template <class TImageFunction, class TImage>
class ITK_EXPORT OptimallyOrientedFluxVesselnessImageFunctionInitializer : 
  public ImageFunctionInitializerBase<TImageFunction,TImage>
{
public:
  
  typedef OptimallyOrientedFluxVesselnessImageFunctionInitializer
    <TImageFunction,TImage>                    Self;
  typedef ImageFunctionInitializerBase
    <TImageFunction,TImage>                    Superclass;
  typedef itk::SmartPointer<Self>              Pointer;
  typedef itk::SmartPointer<const Self>        ConstPointer;

  typedef TImageFunction                       ImageFunctionType;
  typedef TImage                               ImageType;

  typedef typename ImageFunctionType::VectorFieldType      VectorFieldType;
  typedef typename VectorFieldType::Pointer                VectorFieldPointer;
    
public:
  
  itkNewMacro( Self );
  itkTypeMacro( OptimallyOrientedFluxVesselnessImageFunctionInitializer, ImageFunctionInitializerBase );
  
  virtual void Initialize( ImageFunctionType *imageFunction, const ImageType *image = 0, double scale = 0.0 )
    {
      VectorFieldPointer vectorField = imageFunction->GetVectorField();
      vectorField->SetInputImage( image );
      vectorField->SetSigma( 1.0 );
      vectorField->NormalizeAcrossScaleOn();
      vectorField->UseImageSpacingOn();
      vectorField->Initialize();
      
      imageFunction->SetRadius( scale );
      imageFunction->SetInputImage( image );
      imageFunction->Initialize(); // this computes the sphere grid
    }
  
protected:
  
  OptimallyOrientedFluxVesselnessImageFunctionInitializer() {}
  virtual ~OptimallyOrientedFluxVesselnessImageFunctionInitializer() {}
  
private:
  
  OptimallyOrientedFluxVesselnessImageFunctionInitializer(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented
};

} // end namespace ivan

#endif

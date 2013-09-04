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
// File: ivanImageFunctionToCostFunctionAdaptor.h
// Author: Iv�n Mac�a (imacia@vicomtech.org)
// Date: 2010/05/27


#ifndef __ivanImageFunctionToCostFunctionAdaptor_h
#define __ivanImageFunctionToCostFunctionAdaptor_h

#include "itkSingleValuedCostFunction.h"
#include "itkImageFunction.h"


namespace ivan
{
  
/** \class ImageFunctionToCostFunctionAdaptor
 *  \brief Adaptor that adapts an ImageFunction so their values can be used as a CostFunction
 *
 * ImageFunctionToCostFunctionAdaptor is an adaptor that converts an ImageFunction so it can be used 
 * as a CostFunction during optimization procedures. The Parameters array should be interpreted
 * as the N-dimensional point location in image space where the ImageFunction value is interpolated.
 * This works with single-valued images only.
 *
 */

template <class TImage>
class ITK_EXPORT ImageFunctionToCostFunctionAdaptor : public itk::SingleValuedCostFunction
{
public:
  /** Standard class typedefs. */
  typedef ImageFunctionToCostFunctionAdaptor<TImage>  Self;
  typedef itk::SingleValuedCostFunction       Superclass;
  typedef itk::SmartPointer<Self>             Pointer;
  typedef itk::SmartPointer<const Self>       ConstPointer;
  
  typedef typename Superclass::MeasureType    MeasureType;
  
  /** Image related typedefs. */
  typedef TImage                              ImageType;
  typedef typename ImageType::Pointer         ImagePointer;
    
  typedef typename ImageType::PointType       PointType;
    
  /** Image function related typedefs. */
  typedef itk::ImageFunction<ImageType,MeasureType,MeasureType>  ImageFunctionType;
  typedef typename ImageFunctionType::Pointer                    ImageFunctionPointer;
  
public:

  /** Method for creation through the object factory. */
  //itkNewMacro( Self );
  
  /** Run-time type information (and related methods). */
  itkTypeMacro( ImageFunctionToCostFunctionAdaptor, itk::SingleValuedCostFunction );
  
  /** Get the value at the desired point. If the point is out of image bounds, the minimum 
    * value NumericTraits<MeasureType>::min() is returned. The parameters represent the N-D point
    * coordinates. */
  virtual MeasureType GetValue( const ParametersType & parameters ) const;
  
  //virtual void GetDerivative( const ParametersType  &parameters, DerivativeType & derivative ) const
    //{ itkWarningMacro( "Derivative not implemented yet." ); }
    
  virtual unsigned int GetNumberOfParameters(void) const
    { return ImageType::GetImageDimension(); }
  
  void SetImageFunction( ImageFunctionType *func )
    { m_ImageFunction = func; }
  ImageFunctionType * GetImageFunction()
    { return m_ImageFunction; }
  const ImageFunctionType * GetImageFunction() const
    { return m_ImageFunction; }

protected:
  
  ImageFunctionToCostFunctionAdaptor();
  virtual ~ImageFunctionToCostFunctionAdaptor() {}
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;
      
private:
  ImageFunctionToCostFunctionAdaptor(const Self&); //purposely not implemented
  void operator=(const Self&);   //purposely not implemented
  
protected:
  
  ImageFunctionPointer  m_ImageFunction;
};

} // end namespace ivan

#ifndef ITK_MANUAL_INSTANTIATION
#include "ivanImageFunctionToCostFunctionAdaptor.hxx"
#endif

#endif
